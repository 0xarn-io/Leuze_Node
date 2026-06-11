# RSL235_Node

Restful dimensioning gate built on three Leuze **RSL235** safety laser
scanners mounted around a conveyor (left / top / right, all scanning the
same vertical cross-section plane). Per externally triggered event the node
measures a passing box or parcel pile:

- **width** — trimmed x-extent of the fused laser points,
- **height** — trimmed top of the fused points above the belt,
- **length** — object presence duration × conveyor speed.

Results are published on a long-poll **`GET /events`** stream (same
contract as ADS_Node, ready for the orchestrator) and optionally pushed to
a configurable **webhook**. A built-in **setup page** visualizes the live
cross-section and edits the configuration.

> The node only *listens* to the scanners' UDP data telegrams via
> [pyRSL235](https://github.com/0xarn-io/pyRSL235). It is **not a safety
> function** — OSSD/protective-field states are shown for diagnosis only.

## Quick start (no hardware required)

```bash
pip install -r requirements.txt
python main.py                      # config.toml has mock = true by default
```

Open <http://127.0.0.1:9020/setup> — three simulated lasers stream
wire-authentic UDP telegrams and a demo loop pushes random boxes through
the gate every few seconds.

```bash
curl localhost:9020/health
curl -X POST localhost:9020/trigger -H 'content-type: application/json' \
     -d '{"ref":"unit-123","source":"plc"}'
curl 'localhost:9020/events?after=0&timeout=30'
```

## Real hardware

1. In **Sensor Studio** (per scanner): *SETTINGS ▸ Data telegrams* —
   enable the UDP data telegram, set this machine's IP as destination and
   give **each scanner its own destination port** (defaults: left 3050,
   top 3051, right 3052). Enable measurement value transmission.
2. Measure each scanner's mounting position in the gate frame
   (x across the belt from center, z above the belt, rotation of the
   device front) and enter it in `config.toml` / the setup page.
3. Set `mock = false`, start the node, check `/health` shows all lasers
   receiving at ~40 Hz.
4. With an **empty conveyor**, press *Capture baseline* on the setup page
   (persisted to `baseline.json`; auto-invalidated when a pose changes).
5. Set `gate.conveyor_speed_mps` (or pass `speed_mps` per trigger).

## API

| Method & path | Purpose |
|---|---|
| `GET /` | node metadata |
| `GET /health` | status, uptime, per-laser receive state/rate/flags |
| `POST /trigger` | arm the gate: `{source?, ref?, speed_mps?}` → 409 when busy. `ref` is echoed in the result (e.g. the PLC `unit_uuid`); `speed_mps` overrides the configured belt speed for this event |
| `POST /abort` | cancel an armed/running measurement |
| `GET /events?after=0&timeout=25` | long-poll event stream; types `measurement`, `timeout`, `aborted`, `sensor_loss`, `config_changed` — `value` carries the full measurement record |
| `GET /measurements?limit=50` | recent records (newest first); `GET /measurements/{id}` includes the per-sample profile |
| `GET /live` | snapshot for the setup page: state, fused points, live W/H |
| `POST /baseline/capture` | record the empty-gate reference (`{scans?}`) |
| `GET /config` / `POST /config` | read / partially update + persist `config.toml` (hot-applied; receivers rebuilt when laser settings change) |
| `GET /setup` | the setup & visualization page |
| `POST /mock/spawn` | mock mode only: push a box through `{width_m?, height_m?, length_m?}` |

### Measurement record

```json
{
  "measurement_id": "9f0c…", "ref": "unit-123", "outcome": "completed",
  "trigger_source": "plc", "armed_at": "…Z", "started_at": "…Z",
  "ended_at": "…Z", "duration_s": 2.025, "conveyor_speed_mps": 0.5,
  "length_m": 1.0125,
  "width_m":  {"max": 0.612, "median": 0.598},
  "height_m": {"max": 0.405, "median": 0.401},
  "n_samples": 79,
  "quality": {"degraded_lasers": [], "lasers_used": ["left","top","right"],
              "truncated": false},
  "samples": [{"t_rel_s": 0.05, "w_m": 0.6, "h_m": 0.4, "n": 64}, …]
}
```

`max` suits irregular piles ("does it fit through…"), `median` suits clean
cartons. Length accuracy is bounded by the speed accuracy plus ~±2 sample
ticks; with a steady belt expect ±3–5 cm.

### Webhook (optional)

Set `[webhook] url` to additionally POST every completed record
(`schema: "rsl235_node.measurement.v1"`, plus `gate_id`/`node_version`).
Retries follow `retry_delays_s`; delivery state appears in `/health`.
`/events` keeps working either way.

## Configuration

Everything lives in [`config.toml`](config.toml) (env override:
`RSL235_NODE_CONFIG=/path/to/config.toml`). The setup page and
`POST /config` edit the same file via tomlkit, so comments survive.

## Orchestrator integration (FastApiHelloWorld)

Add the node URL to the orchestrator config (`rsl235_url =
"http://…:9020"`) and build a flow like CameraToDb: on the ADS photo-eye
event `POST /trigger {"ref": unit_uuid}`, consume `GET /events`, write the
dimensions to DB_Node keyed by the same uuid as the camera images.

## Tests

```bash
python -m unittest discover -s tests -v
```

Geometry/state-machine/scene tests are dependency-free (stdlib +
pyrsl235); the end-to-end API tests need `fastapi` + `httpx` and skip
otherwise.
