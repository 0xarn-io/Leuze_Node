"""Event queue (long-poll) and optional webhook delivery.

The queue follows the node-family contract of ADS_Node's ``GET /events``:
monotonic sequence numbers, bounded buffer, ``wait(after, timeout)`` blocks
until something newer than ``after`` exists.  The webhook pusher is an
optional push channel on top -- one worker thread, ordered delivery,
bounded retries, never blocks the measurement path.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional

from config import WebhookConfig

log = logging.getLogger("rsl235_node.events")


def utc_iso(ts: Optional[float] = None) -> str:
    moment = (datetime.now(timezone.utc) if ts is None
              else datetime.fromtimestamp(ts, timezone.utc))
    return moment.isoformat(timespec="microseconds").replace("+00:00", "Z")


@dataclass(frozen=True)
class Event:
    seq: int
    id: str
    ts: str
    type: str
    value: Any

    def as_dict(self) -> Dict[str, Any]:
        return {"seq": self.seq, "id": self.id, "ts": self.ts,
                "type": self.type, "value": self.value}


class EventQueue:
    """Thread-safe bounded event buffer with long-poll support."""

    def __init__(self, maxlen: int = 500) -> None:
        self._events: deque[Event] = deque(maxlen=maxlen)
        self._cond = threading.Condition()
        self._seq = 0

    @property
    def latest_seq(self) -> int:
        with self._cond:
            return self._seq

    def publish(self, event_type: str, value: Any, event_id: str = "") -> Event:
        with self._cond:
            self._seq += 1
            event = Event(seq=self._seq, id=event_id or str(self._seq),
                          ts=utc_iso(), type=event_type, value=value)
            self._events.append(event)
            self._cond.notify_all()
        return event

    def collect(self, after: int) -> List[Event]:
        with self._cond:
            return [e for e in self._events if e.seq > after]

    def wait(self, after: int, timeout: float) -> List[Event]:
        """Events with seq > after; blocks up to ``timeout`` for new ones."""
        deadline = time.monotonic() + max(0.0, timeout)
        with self._cond:
            while True:
                pending = [e for e in self._events if e.seq > after]
                if pending:
                    return pending
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return []
                self._cond.wait(remaining)


class WebhookPusher:
    """Delivers JSON payloads to the configured webhook URL, in order.

    ``config_provider`` is read per delivery so config updates apply
    without restarting the worker.  Failures retry per
    ``retry_delays_s``; the final result of the last delivery is kept in
    :attr:`last_result` for diagnostics.
    """

    def __init__(self, config_provider: Callable[[], WebhookConfig]) -> None:
        self._config_provider = config_provider
        self._queue: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue(maxsize=100)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.last_result: Optional[Dict[str, Any]] = None

    def start(self) -> "WebhookPusher":
        if self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._worker,
                                        name="rsl235node-webhook", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def submit(self, payload: Dict[str, Any]) -> bool:
        """Queue a payload; returns False (and logs) when disabled or full."""
        if not self._config_provider().url:
            return False
        try:
            self._queue.put_nowait(payload)
            return True
        except queue.Full:
            log.warning("webhook queue full, dropping payload")
            return False

    # ------------------------------------------------------------------

    def _worker(self) -> None:
        while self._running:
            try:
                payload = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if payload is None:
                continue
            self.last_result = self._deliver(payload)

    def _deliver(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        config = self._config_provider()
        url = config.url
        if not url:
            return {"ok": False, "error": "webhook disabled", "attempts": 0}
        body = json.dumps(payload).encode("utf-8")
        delays = (0.0,) + tuple(config.retry_delays_s)
        attempts = 0
        last_error = ""
        for delay in delays:
            if delay:
                time.sleep(delay)
            if not self._running:
                break
            attempts += 1
            try:
                request = urllib.request.Request(
                    url, data=body, method="POST",
                    headers={"Content-Type": "application/json",
                             "User-Agent": "RSL235_Node"})
                with urllib.request.urlopen(request, timeout=config.timeout_s) as resp:
                    status = resp.status
                if 200 <= status < 300:
                    return {"ok": True, "status": status, "attempts": attempts,
                            "url": url, "ts": utc_iso()}
                last_error = "HTTP %d" % status
            except urllib.error.HTTPError as exc:
                last_error = "HTTP %d" % exc.code
            except Exception as exc:  # URLError, timeout, ...
                last_error = str(exc)
            log.warning("webhook delivery attempt %d failed: %s", attempts, last_error)
        result = {"ok": False, "error": last_error, "attempts": attempts,
                  "url": url, "ts": utc_iso()}
        log.error("webhook delivery gave up after %d attempts: %s",
                  attempts, last_error)
        return result
