"""Measurement core: per-laser workers + the gate state machine.

``LaserWorker`` owns one :class:`leuze_rsl.RSL235Udt` receiver and converts
raw scans -- on the receiver thread -- into immutable ``LaserProfile``
snapshots (foreground filtered, transformed to gate coordinates, ROI
clipped).  ``GateEngine`` ticks at the configured sample rate, fuses the
newest profile of every laser and drives the trigger/measure state
machine.  Everything here is plain stdlib + leuze-rsl so it stays testable
without FastAPI; ``tick(now)`` is public for fake-clock tests.

States: idle -> (POST /trigger) -> armed -> object present -> measuring
        -> object gone -> finalize -> idle        (+ capturing_baseline)
"""

from __future__ import annotations

import json
import logging
import statistics
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from leuze_rsl import RSL235Udt, UdtScan

from config import Config, GateConfig, LaserConfig
from events import EventQueue, WebhookPusher, utc_iso
from geometry import TransformTable, foreground_mask, trimmed_extent

log = logging.getLogger("rsl235_node.gate")

IDLE = "idle"
ARMED = "armed"
MEASURING = "measuring"
CAPTURING_BASELINE = "capturing_baseline"


@dataclass(frozen=True)
class LaserProfile:
    """One processed scan of one laser, ready for fusion."""

    laser_id: str
    scan_number: int
    received_at: float        #: wall clock (scan.received_at)
    monotonic_at: float       #: time.monotonic() when processed, for staleness
    points: Tuple[Tuple[float, float], ...]   #: (x, z) foreground points in ROI
    n_beams: int
    n_foreground: int


class LaserWorker:
    """Wraps one UDP receiver; computes profiles on the rx thread."""

    def __init__(self, laser: LaserConfig,
                 gate_config: Callable[[], GateConfig],
                 receiver_factory: Optional[Callable[..., Any]] = None) -> None:
        self.laser = laser
        self._gate_config = gate_config
        self._latest: Optional[LaserProfile] = None
        self._table: Optional[TransformTable] = None
        self._table_key: Optional[Tuple[int, float]] = None
        self._baseline: Optional[Tuple[int, ...]] = None
        self._baseline_captured_at: Optional[str] = None
        self._collect_lock = threading.Lock()
        self._collect: Optional[List[Tuple[int, ...]]] = None
        self._collect_target = 0
        self._scan_times: deque[float] = deque(maxlen=64)
        factory = receiver_factory or self._default_receiver
        self.receiver = factory(laser, self._on_scan)

    @staticmethod
    def _default_receiver(laser: LaserConfig, on_scan) -> RSL235Udt:
        return RSL235Udt(port=laser.port,
                         source_ip=laser.source_ip or None,
                         on_scan=on_scan,
                         scan_queue_size=4,
                         data_timeout=2.0)

    def start(self) -> None:
        self.receiver.start()

    def stop(self) -> None:
        self.receiver.stop()

    # ------------------------------------------------------------------

    @property
    def laser_id(self) -> str:
        return self.laser.id

    @property
    def latest(self) -> Optional[LaserProfile]:
        return self._latest

    @property
    def is_receiving(self) -> bool:
        return bool(self.receiver.is_receiving)

    # -- baseline ------------------------------------------------------

    @property
    def baseline(self) -> Optional[Tuple[int, ...]]:
        return self._baseline

    @property
    def baseline_ok(self) -> bool:
        return self._baseline is not None

    def set_baseline(self, distances_mm: Optional[Sequence[int]],
                     captured_at: Optional[str] = None) -> None:
        self._baseline = tuple(distances_mm) if distances_mm else None
        self._baseline_captured_at = captured_at if distances_mm else None

    def begin_baseline(self, scans: int) -> None:
        with self._collect_lock:
            self._collect = []
            self._collect_target = max(3, scans)

    def cancel_baseline(self) -> None:
        with self._collect_lock:
            self._collect = None

    @property
    def baseline_progress(self) -> Optional[Tuple[int, int]]:
        with self._collect_lock:
            if self._collect is None:
                return None
            return len(self._collect), self._collect_target

    # -- scan processing (receiver thread) ------------------------------

    def _on_scan(self, scan: UdtScan) -> None:
        now = time.monotonic()
        self._scan_times.append(now)
        gate = self._gate_config()

        with self._collect_lock:
            if self._collect is not None:
                self._collect.append(tuple(scan.distances_mm))
                if len(self._collect) >= self._collect_target:
                    self._baseline = _per_beam_median(self._collect)
                    self._baseline_captured_at = utc_iso()
                    self._collect = None

        angles = scan.angles_deg()
        key = (len(angles), angles[0] if angles else 0.0)
        if self._table is None or self._table_key != key:
            self._table = TransformTable(self.laser.pose, self.laser.sector, angles)
            self._table_key = key

        foreground = foreground_mask(scan.distances_mm, self._baseline,
                                     gate.foreground_margin_mm)
        points = self._table.project(scan.distances_mm, foreground, gate.roi)
        self._latest = LaserProfile(
            laser_id=self.laser.id,
            scan_number=scan.scan_number,
            received_at=scan.received_at,
            monotonic_at=now,
            points=tuple(points),
            n_beams=scan.num_beams,
            n_foreground=sum(foreground),
        )

    # -- diagnostics -----------------------------------------------------

    def scan_rate_hz(self) -> float:
        times = list(self._scan_times)
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        return round((len(times) - 1) / span, 1) if span > 0 else 0.0

    def health(self) -> Dict[str, Any]:
        latest = self._latest
        status = getattr(self.receiver, "latest_status", None)
        stats = dict(getattr(self.receiver, "stats", {}) or {})
        stats.pop("scans_dropped_queue", None)  # callback path; queue is unused
        info: Dict[str, Any] = {
            "id": self.laser.id,
            "name": self.laser.name,
            "port": self.laser.port,
            "is_receiving": self.is_receiving,
            "scan_rate_hz": self.scan_rate_hz(),
            "last_scan_age_s": (round(time.monotonic() - latest.monotonic_at, 3)
                                if latest else None),
            "n_foreground": latest.n_foreground if latest else 0,
            "baseline": {"ok": self.baseline_ok,
                         "captured_at": self._baseline_captured_at},
            "stats": stats,
        }
        if status is not None:
            info["status"] = {
                "operating_mode": status.operating_mode,
                "error": status.error,
                "alarm": status.alarm,
                "screen_contaminated": status.screen_contaminated,
                "ossd_a": status.ossd_a,
                "ossd_b": status.ossd_b,
            }
        else:
            info["status"] = None
        return info


def _per_beam_median(scans: List[Tuple[int, ...]]) -> Tuple[int, ...]:
    beams = min(len(s) for s in scans)
    return tuple(int(statistics.median(s[i] for s in scans))
                 for i in range(beams))


class BaselineStore:
    """baseline.json next to the config file; entries keyed by laser id.

    A stored baseline is only applied when the laser's pose still matches
    -- moving a scanner invalidates its empty-gate reference implicitly.
    """

    def __init__(self, path: Path) -> None:
        self.path = path

    def load_into(self, workers: Sequence[LaserWorker]) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, ValueError):
            return
        for worker in workers:
            entry = data.get(worker.laser.id)
            if not entry:
                continue
            pose = worker.laser.pose
            stored_pose = entry.get("pose", {})
            if (stored_pose.get("x_m") == pose.x_m
                    and stored_pose.get("z_m") == pose.z_m
                    and stored_pose.get("rotation_deg") == pose.rotation_deg
                    and stored_pose.get("mirror", False) == pose.mirror):
                worker.set_baseline(entry.get("distances_mm"),
                                    entry.get("captured_at"))

    def save_from(self, workers: Sequence[LaserWorker]) -> None:
        data: Dict[str, Any] = {}
        for worker in workers:
            if worker.baseline is None:
                continue
            pose = worker.laser.pose
            data[worker.laser.id] = {
                "captured_at": worker._baseline_captured_at,
                "pose": {"x_m": pose.x_m, "z_m": pose.z_m,
                         "rotation_deg": pose.rotation_deg,
                         "mirror": pose.mirror},
                "distances_mm": list(worker.baseline),
            }
        try:
            self.path.write_text(json.dumps(data), encoding="utf-8")
        except OSError as exc:
            log.warning("could not persist baselines to %s: %s", self.path, exc)


def _decimate(seq: List[Any], limit: int) -> List[Any]:
    if len(seq) <= limit:
        return list(seq)
    stride = -(-len(seq) // limit)
    out = seq[::stride]
    if seq and out[-1] is not seq[-1]:
        out.append(seq[-1])
    return out


class GateEngine:
    """Fuses laser profiles at a fixed rate and runs the gate state machine."""

    def __init__(self, config: Config, workers: List[LaserWorker],
                 events: EventQueue,
                 webhook: Optional[WebhookPusher] = None,
                 webhook_extra: Optional[Dict[str, Any]] = None,
                 baseline_store: Optional[BaselineStore] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._config = config
        self.workers = workers
        self._events = events
        self._webhook = webhook
        self._webhook_extra = webhook_extra or {}
        self._store = baseline_store
        self._clock = clock

        self._lock = threading.RLock()
        self._state = IDLE
        self._present = False
        self._on_streak = 0
        self._off_streak = 0
        self._holdoff_until = 0.0
        self._arm_deadline = 0.0
        self._baseline_deadline = 0.0
        self._active: Optional[Dict[str, Any]] = None
        self._history: deque[Dict[str, Any]] = deque(
            maxlen=config.gate.history_max)
        self._live: Dict[str, Any] = {"state": IDLE, "present": False,
                                      "n_points": 0, "points": [],
                                      "width_m": None, "height_m": None}
        self._last_error: Optional[str] = None

        self._running = False
        self._thread: Optional[threading.Thread] = None

        if self._store is not None:
            self._store.load_into(workers)

    # ------------------------------------------------------------------
    # lifecycle

    def start(self) -> "GateEngine":
        if self._running:
            return self
        for worker in self.workers:
            worker.start()
        self._running = True
        self._thread = threading.Thread(target=self._run,
                                        name="rsl235node-gate", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        for worker in self.workers:
            worker.stop()

    def _run(self) -> None:
        period = 1.0 / self._config.gate.sample_rate_hz
        next_time = time.monotonic()
        while self._running:
            try:
                self.tick()
            except Exception:
                log.exception("gate tick failed")
            period = 1.0 / self._config.gate.sample_rate_hz
            next_time += period
            delay = next_time - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.monotonic()

    # ------------------------------------------------------------------
    # commands (called from API threads)

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def config(self) -> Config:
        return self._config

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def trigger(self, source: Optional[str] = None, ref: Optional[str] = None,
                speed_mps: Optional[float] = None) -> Tuple[bool, str]:
        now = self._clock()
        with self._lock:
            if self._state in (MEASURING, CAPTURING_BASELINE):
                return False, "busy: %s" % self._state
            if self._state == IDLE and now < self._holdoff_until:
                return False, "holdoff: retry in %.2f s" % (self._holdoff_until - now)
            if self._state == ARMED:
                # re-arm: refresh deadline and trigger context
                self._arm_deadline = now + self._config.gate.arm_timeout_s
                self._active.update(source=source or "api", ref=ref,
                                    speed_mps=speed_mps)
                return True, "re-armed"
            self._state = ARMED
            self._arm_deadline = now + self._config.gate.arm_timeout_s
            self._active = {
                "id": uuid.uuid4().hex,
                "source": source or "api",
                "ref": ref,
                "speed_mps": speed_mps,
                "armed_mono": now,
                "armed_at": utc_iso(),
                "start_mono": None,
                "started_at": None,
                "samples": [],
                "degraded": set(),
                "truncated": False,
            }
            return True, "armed"

    def abort(self) -> bool:
        now = self._clock()
        with self._lock:
            if self._state not in (ARMED, MEASURING):
                return False
            self._finalize("aborted", ended_mono=now, now=now)
            return True

    def capture_baseline(self, scans: Optional[int] = None) -> Tuple[bool, str]:
        now = self._clock()
        with self._lock:
            if self._state != IDLE:
                return False, "busy: %s" % self._state
            if not any(w.is_receiving for w in self.workers):
                return False, "no laser is receiving data"
            n = scans or self._config.gate.baseline_scans
            for worker in self.workers:
                worker.begin_baseline(n)
            self._state = CAPTURING_BASELINE
            self._baseline_deadline = now + 10.0
            return True, "capturing %d scans" % n

    def set_config(self, config: Config) -> None:
        """Hot-apply new gate parameters; cancels a running measurement."""
        now = self._clock()
        with self._lock:
            self._config = config
            new_history: deque = deque(self._history,
                                       maxlen=config.gate.history_max)
            self._history = new_history
            if self._state in (ARMED, MEASURING):
                self._finalize("config_changed", ended_mono=now, now=now)

    def replace_workers(self, workers: List[LaserWorker]) -> None:
        with self._lock:
            self.workers = workers

    # ------------------------------------------------------------------
    # the tick

    def tick(self, now: Optional[float] = None) -> None:
        if now is None:
            now = self._clock()
        with self._lock:
            self._tick_locked(now)

    def _tick_locked(self, now: float) -> None:
        gate = self._config.gate
        period = 1.0 / gate.sample_rate_hz

        profiles: List[LaserProfile] = []
        stale_ids: List[str] = []
        tagged: List[Tuple[int, float, float]] = []
        points: List[Tuple[float, float]] = []
        for index, worker in enumerate(self.workers):
            profile = worker.latest
            if profile is not None and now - profile.monotonic_at <= gate.stale_after_s:
                profiles.append(profile)
                for x, z in profile.points:
                    tagged.append((index, x, z))
                    points.append((x, z))
            else:
                stale_ids.append(worker.laser_id)
        n_points = len(points)

        # presence with debounce + backdated edges
        rising = falling = False
        edge_time = now
        if n_points >= gate.min_foreground_points:
            self._on_streak += 1
            self._off_streak = 0
            if not self._present and self._on_streak >= gate.on_debounce_samples:
                self._present = True
                rising = True
                edge_time = now - (gate.on_debounce_samples - 1) * period
        else:
            self._off_streak += 1
            self._on_streak = 0
            if self._present and self._off_streak >= gate.off_debounce_samples:
                self._present = False
                falling = True
                edge_time = now - (gate.off_debounce_samples - 1) * period

        width = height = None
        if points:
            x_extent = trimmed_extent([p[0] for p in points], gate.outlier_trim_points)
            z_extent = trimmed_extent([p[1] for p in points], gate.outlier_trim_points)
            width = x_extent[1] - x_extent[0]
            height = max(0.0, z_extent[1] - gate.belt_z_m)

        self._update_live(now, tagged, n_points, width, height)

        if self._state == CAPTURING_BASELINE:
            self._tick_baseline(now)
            return

        if self._state == ARMED:
            if self._present:  # object arrived (or was already inside)
                start = edge_time if rising else now
                self._active.update(start_mono=start,
                                    started_at=utc_iso(time.time() - (now - start)))
                self._state = MEASURING
            elif now >= self._arm_deadline:
                self._finalize("timeout", ended_mono=now, now=now)
                return

        if self._state == MEASURING:
            for laser_id in stale_ids:
                self._active["degraded"].add(laser_id)
            if not profiles:
                self._finalize("sensor_loss", ended_mono=now, now=now)
                return
            if self._present and n_points:
                self._active["samples"].append(
                    (now - self._active["start_mono"], width, height, n_points))
            if falling:
                self._finalize("completed", ended_mono=edge_time, now=now)
            elif now - self._active["start_mono"] >= gate.max_event_s:
                self._active["truncated"] = True
                self._finalize("completed", ended_mono=now, now=now)

    def _tick_baseline(self, now: float) -> None:
        pending = [w for w in self.workers if w.baseline_progress is not None]
        if not pending:
            if self._store is not None:
                self._store.save_from(self.workers)
            self._state = IDLE
            self._last_error = None
            return
        if now >= self._baseline_deadline:
            for worker in self.workers:
                worker.cancel_baseline()
            missing = [w.laser_id for w in pending]
            self._last_error = ("baseline capture timed out, no data from: %s"
                                % ", ".join(missing))
            log.warning("%s", self._last_error)
            self._state = IDLE

    def _update_live(self, now: float, tagged, n_points, width, height) -> None:
        shown = _decimate(tagged, 900)
        self._live = {
            "ts": utc_iso(),
            "state": self._state,
            "present": self._present,
            "n_points": n_points,
            "width_m": round(width, 4) if width is not None else None,
            "height_m": round(height, 4) if height is not None else None,
            "points": [[i, round(x, 3), round(z, 3)] for i, x, z in shown],
        }

    # ------------------------------------------------------------------
    # finalize + outputs

    def _finalize(self, outcome: str, ended_mono: float, now: float) -> None:
        active = self._active or {}
        gate = self._config.gate
        speed = active.get("speed_mps") or gate.conveyor_speed_mps

        started = active.get("start_mono")
        duration = max(0.0, ended_mono - started) if started is not None else None
        length = duration * speed if duration is not None else None

        samples = active.get("samples", [])
        widths = [s[1] for s in samples if s[1] is not None]
        heights = [s[2] for s in samples if s[2] is not None]

        def agg(values: List[float]) -> Optional[Dict[str, float]]:
            if not values:
                return None
            return {"max": round(max(values), 4),
                    "median": round(statistics.median(values), 4)}

        stored_samples = [
            {"t_rel_s": round(t, 3),
             "w_m": round(w, 4) if w is not None else None,
             "h_m": round(h, 4) if h is not None else None,
             "n": n}
            for t, w, h, n in _decimate(samples, 200)]

        lasers_used = sorted({w.laser_id for w in self.workers}
                             - set(active.get("degraded", set())))
        record: Dict[str, Any] = {
            "measurement_id": active.get("id", uuid.uuid4().hex),
            "ref": active.get("ref"),
            "outcome": outcome,
            "trigger_source": active.get("source"),
            "armed_at": active.get("armed_at"),
            "started_at": active.get("started_at"),
            "ended_at": utc_iso(time.time() - (now - ended_mono)),
            "duration_s": round(duration, 3) if duration is not None else None,
            "conveyor_speed_mps": speed,
            "length_m": round(length, 4) if length is not None else None,
            "width_m": agg(widths),
            "height_m": agg(heights),
            "n_samples": len(samples),
            "quality": {
                "degraded_lasers": sorted(active.get("degraded", set())),
                "lasers_used": lasers_used,
                "truncated": bool(active.get("truncated")),
            },
            "samples": stored_samples,
        }

        self._history.appendleft(record)
        self._state = IDLE
        self._active = None
        self._holdoff_until = now + gate.retrigger_holdoff_s

        event_type = "measurement" if outcome == "completed" else outcome
        event_record = dict(record, samples=_decimate(stored_samples, 50))
        self._events.publish(event_type, event_record,
                             event_id=record["measurement_id"])

        webhook_config = self._config.webhook
        if (self._webhook is not None and webhook_config.url
                and (outcome == "completed" or webhook_config.post_all_outcomes)):
            payload = {"schema": "rsl235_node.measurement.v1",
                       **self._webhook_extra, **event_record}
            self._webhook.submit(payload)

    # ------------------------------------------------------------------
    # views

    def live_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            live = dict(self._live)
            state = self._state
            live["state"] = state  # authoritative, not the last tick's view
            active = self._active
            live["roi"] = {
                "x_min_m": self._config.gate.roi.x_min_m,
                "x_max_m": self._config.gate.roi.x_max_m,
                "z_min_m": self._config.gate.roi.z_min_m,
                "z_max_m": self._config.gate.roi.z_max_m,
            }
            live["last_error"] = self._last_error
            if active is not None:
                live["event"] = {
                    "id": active["id"],
                    "ref": active.get("ref"),
                    "source": active.get("source"),
                    "armed_at": active.get("armed_at"),
                    "started_at": active.get("started_at"),
                    "n_samples": len(active.get("samples", [])),
                }
            else:
                live["event"] = None
            if state == CAPTURING_BASELINE:
                live["baseline_progress"] = {
                    w.laser_id: w.baseline_progress for w in self.workers}
        live["lasers"] = [w.health() for w in self.workers]
        return live

    def measurements(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._lock:
            records = list(self._history)[:max(1, limit)]
        return [{k: v for k, v in r.items() if k != "samples"} for r in records]

    def measurement(self, measurement_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            for record in self._history:
                if record["measurement_id"] == measurement_id:
                    return record
        return None

    def health_summary(self) -> Dict[str, Any]:
        receiving = sum(1 for w in self.workers if w.is_receiving)
        total = len(self.workers)
        status = "ok" if receiving == total else (
            "degraded" if receiving else "error")
        return {
            "status": status,
            "state": self.state,
            "lasers_total": total,
            "lasers_receiving": receiving,
            "last_error": self._last_error,
            "lasers": [w.health() for w in self.workers],
        }
