"""Configuration for RSL235_Node: frozen dataclasses + TOML load/update.

``load_config()`` reads (in order) an explicit path, ``$RSL235_NODE_CONFIG``
or ``config.toml`` next to this file, validates everything fast-fail and
returns an immutable :class:`Config`.  ``apply_update()`` merges a partial
update dict (as accepted by ``POST /config``) into the TOML file via
``tomlkit`` so comments and formatting survive.
"""

from __future__ import annotations

import os
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import tomlkit


class ConfigError(ValueError):
    """Invalid or inconsistent configuration."""


_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")


@dataclass(frozen=True)
class Pose:
    x_m: float = 0.0
    z_m: float = 0.0
    rotation_deg: float = 0.0
    mirror: bool = False


@dataclass(frozen=True)
class Sector:
    min_deg: float = -135.0
    max_deg: float = 135.0


@dataclass(frozen=True)
class LaserConfig:
    id: str
    name: str
    port: int
    source_ip: str = ""
    pose: Pose = field(default_factory=Pose)
    sector: Sector = field(default_factory=Sector)


@dataclass(frozen=True)
class Roi:
    x_min_m: float = -0.9
    x_max_m: float = 0.9
    z_min_m: float = 0.03
    z_max_m: float = 1.9

    def contains(self, x_m: float, z_m: float) -> bool:
        return (self.x_min_m <= x_m <= self.x_max_m
                and self.z_min_m <= z_m <= self.z_max_m)


@dataclass(frozen=True)
class GateConfig:
    roi: Roi = field(default_factory=Roi)
    belt_z_m: float = 0.0
    conveyor_speed_mps: float = 0.5
    sample_rate_hz: float = 40.0
    foreground_margin_mm: int = 60
    min_foreground_points: int = 12
    outlier_trim_points: int = 1
    on_debounce_samples: int = 3
    off_debounce_samples: int = 5
    arm_timeout_s: float = 10.0
    max_event_s: float = 30.0
    retrigger_holdoff_s: float = 0.5
    stale_after_s: float = 0.15
    baseline_scans: int = 25
    history_max: int = 200


@dataclass(frozen=True)
class WebhookConfig:
    url: str = ""
    timeout_s: float = 5.0
    retry_delays_s: Tuple[float, ...] = (1.0, 3.0, 7.0)
    post_all_outcomes: bool = False


@dataclass(frozen=True)
class BoxSize:
    w: float
    h: float
    l: float


@dataclass(frozen=True)
class MockConfig:
    auto_trigger: bool = True
    box_period_s: float = 8.0
    seed: int = 0
    box_min: BoxSize = field(default_factory=lambda: BoxSize(0.2, 0.15, 0.3))
    box_max: BoxSize = field(default_factory=lambda: BoxSize(0.8, 0.6, 1.2))


@dataclass(frozen=True)
class NodeInfo:
    gate_id: str = "gate-1"
    mock: bool = True


@dataclass(frozen=True)
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 9020


@dataclass(frozen=True)
class Config:
    node: NodeInfo
    server: ServerConfig
    lasers: Tuple[LaserConfig, ...]
    gate: GateConfig
    webhook: WebhookConfig
    mock: MockConfig
    path: Optional[Path] = None

    def laser(self, laser_id: str) -> LaserConfig:
        for laser in self.lasers:
            if laser.id == laser_id:
                return laser
        raise KeyError(laser_id)


# ----------------------------------------------------------------------
# building & validation

def _section(raw: Dict[str, Any], name: str) -> Dict[str, Any]:
    value = raw.get(name, {})
    if not isinstance(value, dict):
        raise ConfigError("[%s] must be a table" % name)
    return value


def _take(data: Dict[str, Any], key: str, kind, default, where: str):
    value = data.get(key, default)
    if isinstance(kind, tuple) or kind is not bool:
        if isinstance(value, bool) and kind in (int, float):
            raise ConfigError("%s.%s must be a number" % (where, key))
    if kind is float and isinstance(value, int):
        value = float(value)
    if not isinstance(value, kind):
        raise ConfigError("%s.%s has wrong type (expected %s)"
                          % (where, key, getattr(kind, "__name__", kind)))
    return value


def _build_pose(data: Dict[str, Any], where: str) -> Pose:
    return Pose(
        x_m=_take(data, "x_m", float, 0.0, where),
        z_m=_take(data, "z_m", float, 0.0, where),
        rotation_deg=_take(data, "rotation_deg", float, 0.0, where),
        mirror=_take(data, "mirror", bool, False, where),
    )


def _build_sector(data: Dict[str, Any], where: str) -> Sector:
    sector = Sector(
        min_deg=_take(data, "min_deg", float, -135.0, where),
        max_deg=_take(data, "max_deg", float, 135.0, where),
    )
    if sector.min_deg >= sector.max_deg:
        raise ConfigError("%s: min_deg must be < max_deg" % where)
    return sector


def _build_laser(data: Dict[str, Any], index: int) -> LaserConfig:
    where = "lasers[%d]" % index
    laser_id = _take(data, "id", str, "", where)
    if not _ID_RE.match(laser_id):
        raise ConfigError("%s.id %r must match %s" % (where, laser_id, _ID_RE.pattern))
    port = _take(data, "port", int, 0, where)
    if not 1 <= port <= 65535:
        raise ConfigError("%s.port %r out of range" % (where, port))
    return LaserConfig(
        id=laser_id,
        name=_take(data, "name", str, laser_id, where),
        port=port,
        source_ip=_take(data, "source_ip", str, "", where),
        pose=_build_pose(_section(data, "pose"), where + ".pose"),
        sector=_build_sector(_section(data, "sector"), where + ".sector"),
    )


def _build_roi(data: Dict[str, Any]) -> Roi:
    roi = Roi(
        x_min_m=_take(data, "x_min_m", float, -0.9, "gate.roi"),
        x_max_m=_take(data, "x_max_m", float, 0.9, "gate.roi"),
        z_min_m=_take(data, "z_min_m", float, 0.03, "gate.roi"),
        z_max_m=_take(data, "z_max_m", float, 1.9, "gate.roi"),
    )
    if roi.x_min_m >= roi.x_max_m or roi.z_min_m >= roi.z_max_m:
        raise ConfigError("gate.roi: min bounds must be < max bounds")
    return roi


def _build_gate(data: Dict[str, Any]) -> GateConfig:
    gate = GateConfig(
        roi=_build_roi(_section(data, "roi")),
        belt_z_m=_take(data, "belt_z_m", float, 0.0, "gate"),
        conveyor_speed_mps=_take(data, "conveyor_speed_mps", float, 0.5, "gate"),
        sample_rate_hz=_take(data, "sample_rate_hz", float, 40.0, "gate"),
        foreground_margin_mm=_take(data, "foreground_margin_mm", int, 60, "gate"),
        min_foreground_points=_take(data, "min_foreground_points", int, 12, "gate"),
        outlier_trim_points=_take(data, "outlier_trim_points", int, 1, "gate"),
        on_debounce_samples=_take(data, "on_debounce_samples", int, 3, "gate"),
        off_debounce_samples=_take(data, "off_debounce_samples", int, 5, "gate"),
        arm_timeout_s=_take(data, "arm_timeout_s", float, 10.0, "gate"),
        max_event_s=_take(data, "max_event_s", float, 30.0, "gate"),
        retrigger_holdoff_s=_take(data, "retrigger_holdoff_s", float, 0.5, "gate"),
        stale_after_s=_take(data, "stale_after_s", float, 0.15, "gate"),
        baseline_scans=_take(data, "baseline_scans", int, 25, "gate"),
        history_max=_take(data, "history_max", int, 200, "gate"),
    )
    checks = [
        (gate.conveyor_speed_mps > 0, "conveyor_speed_mps must be > 0"),
        (1.0 <= gate.sample_rate_hz <= 200.0, "sample_rate_hz must be in 1..200"),
        (gate.foreground_margin_mm >= 0, "foreground_margin_mm must be >= 0"),
        (gate.min_foreground_points >= 1, "min_foreground_points must be >= 1"),
        (gate.outlier_trim_points >= 0, "outlier_trim_points must be >= 0"),
        (gate.on_debounce_samples >= 1, "on_debounce_samples must be >= 1"),
        (gate.off_debounce_samples >= 1, "off_debounce_samples must be >= 1"),
        (gate.arm_timeout_s > 0, "arm_timeout_s must be > 0"),
        (gate.max_event_s > 0, "max_event_s must be > 0"),
        (gate.retrigger_holdoff_s >= 0, "retrigger_holdoff_s must be >= 0"),
        (gate.stale_after_s > 0, "stale_after_s must be > 0"),
        (gate.baseline_scans >= 3, "baseline_scans must be >= 3"),
        (1 <= gate.history_max <= 10000, "history_max must be in 1..10000"),
    ]
    for ok, message in checks:
        if not ok:
            raise ConfigError("gate: " + message)
    return gate


def _build_webhook(data: Dict[str, Any]) -> WebhookConfig:
    delays = data.get("retry_delays_s", [1.0, 3.0, 7.0])
    if (not isinstance(delays, list)
            or any(not isinstance(d, (int, float)) or isinstance(d, bool) or d < 0
                   for d in delays)):
        raise ConfigError("webhook.retry_delays_s must be a list of delays >= 0")
    webhook = WebhookConfig(
        url=_take(data, "url", str, "", "webhook"),
        timeout_s=_take(data, "timeout_s", float, 5.0, "webhook"),
        retry_delays_s=tuple(float(d) for d in delays),
        post_all_outcomes=_take(data, "post_all_outcomes", bool, False, "webhook"),
    )
    if webhook.url and not webhook.url.startswith(("http://", "https://")):
        raise ConfigError("webhook.url must start with http:// or https://")
    if webhook.timeout_s <= 0:
        raise ConfigError("webhook.timeout_s must be > 0")
    return webhook


def _build_box(data: Dict[str, Any], default: BoxSize, where: str) -> BoxSize:
    return BoxSize(
        w=_take(data, "w", float, default.w, where),
        h=_take(data, "h", float, default.h, where),
        l=_take(data, "l", float, default.l, where),
    )


def _build_mock(data: Dict[str, Any]) -> MockConfig:
    mock = MockConfig(
        auto_trigger=_take(data, "auto_trigger", bool, True, "mock"),
        box_period_s=_take(data, "box_period_s", float, 8.0, "mock"),
        seed=_take(data, "seed", int, 0, "mock"),
        box_min=_build_box(_section(data, "box_min"), BoxSize(0.2, 0.15, 0.3),
                           "mock.box_min"),
        box_max=_build_box(_section(data, "box_max"), BoxSize(0.8, 0.6, 1.2),
                           "mock.box_max"),
    )
    if mock.box_period_s <= 0:
        raise ConfigError("mock.box_period_s must be > 0")
    for dim in ("w", "h", "l"):
        lo, hi = getattr(mock.box_min, dim), getattr(mock.box_max, dim)
        if not 0 < lo <= hi:
            raise ConfigError("mock.box_min/box_max: need 0 < min <= max for %r" % dim)
    return mock


def build_config(raw: Dict[str, Any], path: Optional[Path] = None) -> Config:
    """Validate a raw TOML dict and return an immutable Config."""
    node_raw = _section(raw, "node")
    node = NodeInfo(
        gate_id=_take(node_raw, "gate_id", str, "gate-1", "node"),
        mock=_take(node_raw, "mock", bool, True, "node"),
    )
    server_raw = _section(raw, "server")
    server = ServerConfig(
        host=_take(server_raw, "host", str, "0.0.0.0", "server"),
        port=_take(server_raw, "port", int, 9020, "server"),
    )
    if not 1 <= server.port <= 65535:
        raise ConfigError("server.port out of range")

    lasers_raw = raw.get("lasers", [])
    if not isinstance(lasers_raw, list) or not lasers_raw:
        raise ConfigError("at least one [[lasers]] entry is required")
    lasers = tuple(_build_laser(entry, i) for i, entry in enumerate(lasers_raw))
    ids = [laser.id for laser in lasers]
    if len(set(ids)) != len(ids):
        raise ConfigError("laser ids must be unique: %r" % ids)
    ports = [laser.port for laser in lasers]
    if len(set(ports)) != len(ports):
        raise ConfigError("laser ports must be unique: %r" % ports)

    return Config(
        node=node,
        server=server,
        lasers=lasers,
        gate=_build_gate(_section(raw, "gate")),
        webhook=_build_webhook(_section(raw, "webhook")),
        mock=_build_mock(_section(raw, "mock")),
        path=path,
    )


def default_config_path() -> Path:
    env = os.environ.get("RSL235_NODE_CONFIG")
    if env:
        return Path(env)
    return Path(__file__).resolve().parent / "config.toml"


def load_config(path: Optional[os.PathLike] = None) -> Config:
    config_path = Path(path) if path is not None else default_config_path()
    try:
        with open(config_path, "rb") as handle:
            raw = tomllib.load(handle)
    except FileNotFoundError:
        raise ConfigError("config file not found: %s" % config_path) from None
    except tomllib.TOMLDecodeError as exc:
        raise ConfigError("malformed TOML in %s: %s" % (config_path, exc)) from None
    return build_config(raw, config_path)


# ----------------------------------------------------------------------
# partial updates (POST /config)

_UPDATABLE_TABLES = ("node", "gate", "webhook", "mock")
_LASER_KEYS = {"id", "name", "port", "source_ip", "pose", "sector"}


def _merge_table(target: Dict[str, Any], update: Dict[str, Any], where: str) -> None:
    for key, value in update.items():
        if isinstance(value, dict):
            sub = target.setdefault(key, {})
            if not isinstance(sub, dict):
                raise ConfigError("%s.%s is not a table" % (where, key))
            _merge_table(sub, value, "%s.%s" % (where, key))
        else:
            target[key] = value


def merge_update(raw: Dict[str, Any], update: Dict[str, Any]) -> Dict[str, Any]:
    """Deep-merge a partial update into a raw config dict (pure, validated

    against structure only -- run :func:`build_config` on the result for the
    full semantic validation).  Lasers are matched by ``id`` and cannot be
    added or removed through updates.
    """
    if not isinstance(update, dict):
        raise ConfigError("update must be an object")
    for key, value in update.items():
        if key in _UPDATABLE_TABLES:
            if not isinstance(value, dict):
                raise ConfigError("%r must be an object" % key)
            section = raw.setdefault(key, {})
            _merge_table(section, value, key)
        elif key == "lasers":
            if not isinstance(value, list):
                raise ConfigError("'lasers' must be a list of partial laser objects")
            existing = {entry.get("id"): entry for entry in raw.get("lasers", [])}
            for partial in value:
                if not isinstance(partial, dict) or "id" not in partial:
                    raise ConfigError("each laser update needs an 'id'")
                unknown = set(partial) - _LASER_KEYS
                if unknown:
                    raise ConfigError("unknown laser keys: %s" % sorted(unknown))
                target = existing.get(partial["id"])
                if target is None:
                    raise ConfigError("unknown laser id %r" % partial["id"])
                _merge_table(target, partial, "lasers[%s]" % partial["id"])
        else:
            raise ConfigError("unknown config section %r" % key)
    return raw


def apply_update(path: Path, update: Dict[str, Any]) -> Config:
    """Validate ``update`` against the file at ``path``, persist and return.

    The merged result is fully validated *before* anything is written; the
    TOML document is edited with tomlkit so comments/formatting survive.
    """
    with open(path, "rb") as handle:
        raw = tomllib.load(handle)
    merged = merge_update(raw, update)
    config = build_config(merged, path)

    document = tomlkit.parse(path.read_text(encoding="utf-8"))
    plain = {key: value for key, value in update.items() if key != "lasers"}
    for key, value in plain.items():
        section = document.setdefault(key, tomlkit.table())
        _merge_document(section, value)
    for partial in update.get("lasers", []):
        for entry in document.get("lasers", []):
            if entry.get("id") == partial["id"]:
                _merge_document(entry, {k: v for k, v in partial.items() if k != "id"})
    path.write_text(tomlkit.dumps(document), encoding="utf-8")
    return config


def _merge_document(target, update: Dict[str, Any]) -> None:
    for key, value in update.items():
        if isinstance(value, dict):
            if key not in target:
                target[key] = tomlkit.inline_table()
            _merge_document(target[key], value)
        else:
            target[key] = value


def config_as_dict(config: Config) -> Dict[str, Any]:
    """JSON-friendly dump of the active configuration."""
    import dataclasses

    def convert(value):
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return {f.name: convert(getattr(value, f.name))
                    for f in dataclasses.fields(value)}
        if isinstance(value, tuple):
            return [convert(v) for v in value]
        if isinstance(value, Path):
            return str(value)
        return value

    data = convert(config)
    data.pop("path", None)
    return data
