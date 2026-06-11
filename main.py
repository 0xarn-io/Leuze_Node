"""RSL235_Node -- restful dimensioning gate for 3x Leuze RSL235 scanners.

Run:  python main.py          (host/port from config.toml [server])
  or: uvicorn main:app --host 0.0.0.0 --port 9020

Set ``mock = true`` in config.toml to run against the built-in scene
simulators instead of real hardware.
"""

from __future__ import annotations

import pathlib
import sys
import time

try:
    import pyrsl235  # noqa: F401
except ImportError:  # sibling checkout fallback, like pyRSL235/examples
    _sibling = pathlib.Path(__file__).resolve().parent.parent / "pyRSL235"
    if (_sibling / "pyrsl235").exists():
        sys.path.insert(0, str(_sibling))
    import pyrsl235  # noqa: F401

import logging
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

import setup_page
from config import (Config, ConfigError, apply_update, config_as_dict,
                    load_config)
from events import EventQueue, WebhookPusher
from gate import BaselineStore, GateEngine, LaserWorker
from mock import MockGate

log = logging.getLogger("rsl235_node")

NODE_NAME = "RSL235_Node"
VERSION = "0.1.0"
DESCRIPTION = ("Dimensioning gate: three Leuze RSL235 laser scanners around "
               "a conveyor measure width, height and length of passing boxes "
               "per externally triggered event.")
META = {
    "name": NODE_NAME,
    "description": DESCRIPTION,
    "version": VERSION,
    "author": "0xarn-io",
    "email": "zimzf23@gmail.com",
    "company": "0xarn-io",
}


class NodeState:
    """Owns workers, engine, event queue, webhook pusher and mock gear."""

    def __init__(self, config: Config, receiver_factory=None) -> None:
        self.config = config
        self.receiver_factory = receiver_factory
        self.started_at = time.monotonic()
        self.events = EventQueue(maxlen=500)
        self.webhook = WebhookPusher(lambda: self.config.webhook)
        store = (BaselineStore(config.path.parent / "baseline.json")
                 if config.path else None)
        self.engine = GateEngine(
            config,
            workers=self._build_workers(config),
            events=self.events,
            webhook=self.webhook,
            webhook_extra={"gate_id": config.node.gate_id,
                           "node_version": VERSION},
            baseline_store=store,
        )
        self.mock: Optional[MockGate] = (MockGate(config, self.engine)
                                         if config.node.mock else None)
        self._running = False

    def _build_workers(self, config: Config) -> List[LaserWorker]:
        return [LaserWorker(laser, lambda: self.config.gate,
                            receiver_factory=self.receiver_factory)
                for laser in config.lasers]

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self.webhook.start()
        self.engine.start()
        if self.mock is not None:
            self.mock.start()

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self.mock is not None:
            self.mock.stop()
        self.engine.stop()
        self.webhook.stop()

    def apply(self, new_config: Config) -> None:
        """Hot-apply a validated config (already persisted to disk)."""
        lasers_changed = new_config.lasers != self.config.lasers
        self.config = new_config
        self.engine.set_config(new_config)
        if not lasers_changed:
            return
        log.info("laser configuration changed; rebuilding receivers")
        if self.mock is not None:
            self.mock.stop()
        for worker in self.engine.workers:
            worker.stop()
        workers = self._build_workers(new_config)
        if self.engine._store is not None:
            self.engine._store.load_into(workers)  # pose-matched baselines only
        self.engine.replace_workers(workers)
        if self._running:
            for worker in workers:
                worker.start()
        if new_config.node.mock:
            self.mock = MockGate(new_config, self.engine)
            if self._running:
                self.mock.start()
        else:
            self.mock = None


# ----------------------------------------------------------------------
# request bodies

class TriggerBody(BaseModel):
    source: Optional[str] = None
    ref: Optional[str] = None
    speed_mps: Optional[float] = Field(default=None, gt=0.0)


class BaselineBody(BaseModel):
    scans: Optional[int] = Field(default=None, ge=3, le=500)


class SpawnBody(BaseModel):
    width_m: Optional[float] = Field(default=None, gt=0.0)
    height_m: Optional[float] = Field(default=None, gt=0.0)
    length_m: Optional[float] = Field(default=None, gt=0.0)


# ----------------------------------------------------------------------

def create_app(config: Optional[Config] = None, receiver_factory=None) -> FastAPI:
    state = NodeState(config or load_config(), receiver_factory=receiver_factory)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        state.start()
        try:
            yield
        finally:
            state.stop()

    app = FastAPI(title=NODE_NAME, version=VERSION, description=DESCRIPTION,
                  lifespan=lifespan)
    app.state.node = state

    @app.get("/")
    def root() -> Dict[str, Any]:
        return dict(META)

    @app.get("/health")
    def health() -> Dict[str, Any]:
        summary = state.engine.health_summary()
        summary["uptime_s"] = round(time.monotonic() - state.started_at, 1)
        summary["mock"] = state.config.node.mock
        summary["gate_id"] = state.config.node.gate_id
        summary["webhook_last_result"] = state.webhook.last_result
        return summary

    @app.post("/trigger")
    def trigger(body: Optional[TriggerBody] = None) -> Dict[str, Any]:
        body = body or TriggerBody()
        ok, message = state.engine.trigger(source=body.source, ref=body.ref,
                                           speed_mps=body.speed_mps)
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"accepted": True, "state": state.engine.state,
                "detail": message}

    @app.post("/abort")
    def abort() -> Dict[str, Any]:
        aborted = state.engine.abort()
        return {"aborted": aborted, "state": state.engine.state}

    @app.get("/events")
    def get_events(after: int = Query(default=0, ge=0),
                   timeout: float = Query(default=25.0, ge=0.0, le=300.0)
                   ) -> Dict[str, Any]:
        events = state.events.wait(after, timeout)
        return {"events": [e.as_dict() for e in events],
                "latest_seq": state.events.latest_seq}

    @app.get("/measurements")
    def measurements(limit: int = Query(default=50, ge=1, le=500)
                     ) -> List[Dict[str, Any]]:
        return state.engine.measurements(limit)

    @app.get("/measurements/{measurement_id}")
    def measurement(measurement_id: str) -> Dict[str, Any]:
        record = state.engine.measurement(measurement_id)
        if record is None:
            raise HTTPException(status_code=404, detail="unknown measurement id")
        return record

    @app.get("/live")
    def live() -> Dict[str, Any]:
        return state.engine.live_snapshot()

    @app.post("/baseline/capture", status_code=202)
    def baseline_capture(body: Optional[BaselineBody] = None) -> Dict[str, Any]:
        scans = body.scans if body else None
        ok, message = state.engine.capture_baseline(scans)
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"accepted": True, "detail": message}

    @app.get("/config")
    def get_config() -> Dict[str, Any]:
        return config_as_dict(state.config)

    @app.post("/config")
    def post_config(update: Dict[str, Any]) -> Dict[str, Any]:
        if state.config.path is None:
            raise HTTPException(status_code=409, detail="no config file to update")
        try:
            new_config = apply_update(state.config.path, update)
        except ConfigError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from None
        state.apply(new_config)
        return config_as_dict(state.config)

    @app.get("/setup", response_class=HTMLResponse)
    def setup() -> str:
        return setup_page.render(NODE_NAME, state.config.node.gate_id,
                                 VERSION, state.config.node.mock)

    @app.post("/mock/spawn")
    def mock_spawn(body: Optional[SpawnBody] = None) -> Dict[str, Any]:
        if state.mock is None:
            raise HTTPException(status_code=404, detail="mock mode is disabled")
        body = body or SpawnBody()
        return state.mock.spawn(width_m=body.width_m, height_m=body.height_m,
                                length_m=body.length_m)

    return app


app = create_app()


if __name__ == "__main__":
    import uvicorn

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    server = app.state.node.config.server
    uvicorn.run(app, host=server.host, port=server.port, log_level="info")
