"""Mock mode: a 2D gate scene fed through real UDP datagrams.

``BoxPassScene`` models the gate cross-section (side walls, belt plane,
ceiling) plus one transient box that "rides through" for
``length / speed`` seconds.  ``SceneUdtSimulator`` subclasses the driver's
wire-authentic :class:`leuze_rsl.simulator.UdtSimulator` and only swaps the
distance source for per-scan ray casts of the scene -- so the node under
mock mode exercises the exact same UDP receive path as with real lasers.
"""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from typing import Dict, List, Optional, Tuple

from leuze_rsl.simulator import UdtSimulator

import gate as gate_mod
from config import Config, Pose
from geometry import beam_theta_deg

log = logging.getLogger("rsl235_node.mock")

_EPS = 1e-9
_INF = float("inf")


class BoxPassScene:
    """Cross-section world the simulators ray-cast against (thread-safe)."""

    def __init__(self, half_width_m: float = 2.0, ceiling_m: float = 2.5,
                 floor_z_m: float = 0.0, max_range_mm: int = 25000) -> None:
        self.half_width_m = half_width_m
        self.ceiling_m = ceiling_m
        self.floor_z_m = floor_z_m
        self.max_range_mm = max_range_mm
        self._lock = threading.Lock()
        self._box: Optional[Dict[str, float]] = None

    # -- box lifecycle ---------------------------------------------------

    def spawn(self, width_m: float, height_m: float, length_m: float,
              speed_mps: float, center_x_m: float = 0.0,
              now: Optional[float] = None) -> Dict[str, float]:
        if min(width_m, height_m, length_m, speed_mps) <= 0:
            raise ValueError("box dimensions and speed must be > 0")
        start = time.monotonic() if now is None else now
        box = {
            "x0": center_x_m - width_m / 2.0,
            "x1": center_x_m + width_m / 2.0,
            "z1": self.floor_z_m + height_m,
            "until": start + length_m / speed_mps,
            "width_m": width_m, "height_m": height_m, "length_m": length_m,
        }
        with self._lock:
            self._box = box
        return dict(box)

    def clear(self) -> None:
        with self._lock:
            self._box = None

    def active_box(self, now: Optional[float] = None
                   ) -> Optional[Tuple[float, float, float]]:
        moment = time.monotonic() if now is None else now
        with self._lock:
            box = self._box
            if box is None:
                return None
            if moment > box["until"]:
                self._box = None
                return None
            return box["x0"], box["x1"], box["z1"]

    # -- ray casting -------------------------------------------------------

    def static_distance_m(self, origin: Tuple[float, float],
                          dx: float, dz: float) -> float:
        """Nearest hit on walls / floor / ceiling along (dx, dz)."""
        ox, oz = origin
        best = self.max_range_mm / 1000.0
        if dx > _EPS:
            best = min(best, (self.half_width_m - ox) / dx)
        elif dx < -_EPS:
            best = min(best, (-self.half_width_m - ox) / dx)
        if dz > _EPS:
            best = min(best, (self.ceiling_m - oz) / dz)
        elif dz < -_EPS:
            best = min(best, (self.floor_z_m - oz) / dz)
        return max(best, 0.0)

    @staticmethod
    def box_distance_m(box: Tuple[float, float, float],
                       origin: Tuple[float, float],
                       dx: float, dz: float, floor_z_m: float = 0.0) -> float:
        """Slab-method ray/AABB distance; inf when the ray misses the box."""
        x0, x1, z1 = box
        ox, oz = origin
        tmin, tmax = 0.0, _INF
        for o, d, lo, hi in ((ox, dx, x0, x1), (oz, dz, floor_z_m, z1)):
            if abs(d) < _EPS:
                if not lo <= o <= hi:
                    return _INF
                continue
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                return _INF
        return tmin if tmin > _EPS else _INF

    def distance_mm(self, origin: Tuple[float, float], theta_deg: float,
                    now: Optional[float] = None) -> int:
        theta = math.radians(theta_deg)
        dx, dz = math.cos(theta), math.sin(theta)
        distance = self.static_distance_m(origin, dx, dz)
        box = self.active_box(now)
        if box is not None:
            distance = min(distance,
                           self.box_distance_m(box, origin, dx, dz, self.floor_z_m))
        return min(self.max_range_mm, int(distance * 1000))


class SceneUdtSimulator(UdtSimulator):
    """UdtSimulator whose beam distances come from a BoxPassScene each scan."""

    def __init__(self, scene: BoxPassScene, pose: Pose, target_port: int,
                 target_host: str = "127.0.0.1",
                 contour: Tuple[int, int, int] = (25, 2725, 10),
                 scan_period_s: float = 0.025, **kwargs) -> None:
        super().__init__(target_host, target_port, telegram_id=6,
                         contour=contour, scan_period_s=scan_period_s,
                         max_range_mm=scene.max_range_mm, **kwargs)
        self.scene = scene
        self.pose = pose
        origin = (pose.x_m, pose.z_m)
        self._origin = origin
        self._dirs: List[Tuple[float, float]] = []
        self._static_mm: List[int] = []
        start, stop, interval = self.contour
        for index in range(start, stop + 1, interval):
            angle = index * 0.1 - 137.5  # driver convention (udt.angles_deg)
            theta = math.radians(beam_theta_deg(pose, angle))
            dx, dz = math.cos(theta), math.sin(theta)
            self._dirs.append((dx, dz))
            static = scene.static_distance_m(origin, dx, dz)
            self._static_mm.append(min(scene.max_range_mm, int(static * 1000)))

    def scene_values(self, now: Optional[float] = None) -> List[int]:
        box = self.scene.active_box(now)
        if box is None:
            return list(self._static_mm)
        values: List[int] = []
        floor = self.scene.floor_z_m
        for (dx, dz), static in zip(self._dirs, self._static_mm):
            t = self.scene.box_distance_m(box, self._origin, dx, dz, floor)
            if t < _INF:
                values.append(min(static, int(t * 1000)))
            else:
                values.append(static)
        return values

    def _loop(self) -> None:  # parent computes distances once; we re-cast per scan
        sock = self._sock
        scan_number = 0
        next_time = time.monotonic()
        while self._running and sock is not None:
            distances = self.scene_values()
            try:
                sock.sendto(self._status_datagram(scan_number), self.target)
                self.datagrams_sent += 1
                for datagram in self._measurement_datagrams(scan_number, distances):
                    sock.sendto(datagram, self.target)
                    self.datagrams_sent += 1
            except OSError:
                break
            self.scans_sent += 1
            scan_number += 1
            next_time += self.scan_period_s
            delay = next_time - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.monotonic()


class MockGate:
    """Runs one simulator per configured laser + an optional demo loop."""

    def __init__(self, config: Config, engine: "gate_mod.GateEngine",
                 scene: Optional[BoxPassScene] = None) -> None:
        self.config = config
        self.engine = engine
        self.scene = scene or BoxPassScene()
        self.sims = [SceneUdtSimulator(self.scene, laser.pose,
                                       target_port=laser.port)
                     for laser in config.lasers]
        self._rng = random.Random(config.mock.seed or None)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "MockGate":
        for sim in self.sims:
            sim.start()
        if self.config.mock.auto_trigger and not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._auto_loop,
                                            name="rsl235node-mockauto",
                                            daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        for sim in self.sims:
            sim.stop()

    def spawn(self, width_m: Optional[float] = None,
              height_m: Optional[float] = None,
              length_m: Optional[float] = None) -> Dict[str, float]:
        mock = self.config.mock
        dims = {
            "width_m": width_m or self._rng.uniform(mock.box_min.w, mock.box_max.w),
            "height_m": height_m or self._rng.uniform(mock.box_min.h, mock.box_max.h),
            "length_m": length_m or self._rng.uniform(mock.box_min.l, mock.box_max.l),
        }
        speed = self.engine.config.gate.conveyor_speed_mps
        self.scene.spawn(dims["width_m"], dims["height_m"], dims["length_m"],
                         speed_mps=speed,
                         center_x_m=self._rng.uniform(-0.15, 0.15))
        return {**dims, "speed_mps": speed}

    # ------------------------------------------------------------------

    def _sleep_while_running(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while self._running and time.monotonic() < deadline:
            time.sleep(0.05)

    def _auto_loop(self) -> None:
        deadline = time.monotonic() + 15.0
        while (self._running and time.monotonic() < deadline
               and not all(w.is_receiving for w in self.engine.workers)):
            time.sleep(0.2)
        if self._running and not all(w.baseline_ok for w in self.engine.workers):
            ok, message = self.engine.capture_baseline()
            log.info("mock auto baseline: %s", message)
            while self._running and self.engine.state == gate_mod.CAPTURING_BASELINE:
                time.sleep(0.1)

        while self._running:
            cycle_start = time.monotonic()
            ok, message = self.engine.trigger(source="mock-auto")
            if ok:
                self._sleep_while_running(0.4)
                if not self._running:
                    break
                dims = self.spawn()
                log.info("mock box: %.2f x %.2f x %.2f m",
                         dims["width_m"], dims["height_m"], dims["length_m"])
                waited = time.monotonic()
                while (self._running and self.engine.state != gate_mod.IDLE
                       and time.monotonic() - waited < 40.0):
                    time.sleep(0.1)
            else:
                log.debug("mock auto trigger skipped: %s", message)
            rest = self.config.mock.box_period_s - (time.monotonic() - cycle_start)
            if rest > 0:
                self._sleep_while_running(rest)
