"""End-to-end API tests: FastAPI TestClient + mock scene simulators.

Skipped when fastapi/httpx are not installed; the measurement core is
covered without them by the other test modules.
"""

import http.server
import json
import pathlib
import socket
import sys
import threading
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    import fastapi  # noqa: F401
    from fastapi.testclient import TestClient
    HAVE_FASTAPI = True
except ImportError:
    HAVE_FASTAPI = False

from config import load_config


def free_udp_ports(count):
    sockets, ports = [], []
    for _ in range(count):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        sockets.append(sock)
        ports.append(sock.getsockname()[1])
    for sock in sockets:
        sock.close()
    return ports


CONFIG_TEMPLATE = """
# test config (comment must survive updates)
[node]
gate_id = "test-gate"
mock = true

[server]
host = "127.0.0.1"
port = 9020

[[lasers]]
id = "left"
name = "Left"
port = {p0}
pose = {{ x_m = -1.0, z_m = 0.5, rotation_deg = 0.0 }}
sector = {{ min_deg = -110.0, max_deg = 110.0 }}

[[lasers]]
id = "top"
name = "Top"
port = {p1}
pose = {{ x_m = 0.0, z_m = 2.0, rotation_deg = -90.0 }}
sector = {{ min_deg = -110.0, max_deg = 110.0 }}

[[lasers]]
id = "right"
name = "Right"
port = {p2}
pose = {{ x_m = 1.0, z_m = 0.5, rotation_deg = 180.0 }}
sector = {{ min_deg = -110.0, max_deg = 110.0 }}

[gate]
roi = {{ x_min_m = -0.9, x_max_m = 0.9, z_min_m = 0.03, z_max_m = 1.9 }}
conveyor_speed_mps = 0.5
arm_timeout_s = 3.0
retrigger_holdoff_s = 0.1
baseline_scans = 5
history_max = 50

[webhook]
url = "{webhook_url}"
timeout_s = 2.0
retry_delays_s = [0.1, 0.1, 0.1]

[mock]
auto_trigger = false
seed = 7
"""


class _CapturingHandler(http.server.BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length) or b"{}")
        server = self.server
        server.received.append(body)
        status = (server.status_plan.pop(0) if server.status_plan else 200)
        self.send_response(status)
        self.end_headers()

    def log_message(self, *args):
        pass


def start_capture_server(status_plan):
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0),
                                             _CapturingHandler)
    server.received = []
    server.status_plan = list(status_plan)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


@unittest.skipUnless(HAVE_FASTAPI, "fastapi/httpx not installed")
class ApiTests(unittest.TestCase):
    webhook_status_plan = ()

    @classmethod
    def setUpClass(cls):
        import tempfile

        cls.capture = start_capture_server(cls.webhook_status_plan)
        webhook_url = ("http://127.0.0.1:%d/hook" % cls.capture.server_port
                       if cls.webhook_status_plan else "")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.config_path = pathlib.Path(cls.tmp.name) / "config.toml"
        p0, p1, p2 = free_udp_ports(3)
        cls.config_path.write_text(CONFIG_TEMPLATE.format(
            p0=p0, p1=p1, p2=p2, webhook_url=webhook_url))

        import main as main_mod
        cls.app = main_mod.create_app(load_config(cls.config_path))
        cls.client = TestClient(cls.app)
        cls.client.__enter__()  # run lifespan: start workers/sims/engine

    @classmethod
    def tearDownClass(cls):
        cls.client.__exit__(None, None, None)
        cls.capture.shutdown()
        cls.capture.server_close()
        cls.tmp.cleanup()

    # -- helpers --------------------------------------------------------

    def wait_for(self, predicate, timeout=10.0, message="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.1)
        self.fail("timed out waiting for " + message)

    def all_receiving(self):
        health = self.client.get("/health").json()
        return health["lasers_receiving"] == health["lasers_total"]

    def ensure_baseline(self):
        health = self.client.get("/health").json()
        if all(l["baseline"]["ok"] for l in health["lasers"]):
            return
        response = self.client.post("/baseline/capture", json={"scans": 5})
        self.assertEqual(response.status_code, 202, response.text)
        self.wait_for(
            lambda: all(l["baseline"]["ok"]
                        for l in self.client.get("/health").json()["lasers"]),
            timeout=8.0, message="baseline capture")

    def post_trigger(self, body, timeout=3.0):
        """POST /trigger, retrying through the post-event holdoff window."""
        deadline = time.monotonic() + timeout
        while True:
            response = self.client.post("/trigger", json=body)
            if response.status_code == 200 or time.monotonic() > deadline:
                return response
            time.sleep(0.1)

    def run_box(self, ref, width, height, length, after):
        response = self.post_trigger({"source": "test", "ref": ref})
        self.assertEqual(response.status_code, 200, response.text)
        spawn = self.client.post("/mock/spawn", json={
            "width_m": width, "height_m": height, "length_m": length})
        self.assertEqual(spawn.status_code, 200, spawn.text)
        deadline = time.monotonic() + 20.0
        cursor = after
        while time.monotonic() < deadline:
            data = self.client.get(
                "/events", params={"after": cursor, "timeout": 5}).json()
            for event in data["events"]:
                cursor = event["seq"]
                if event["type"] == "measurement":
                    return event
        self.fail("no measurement event arrived")

    # -- the tests ------------------------------------------------------

    def test_node_end_to_end(self):
        client = self.client

        root = client.get("/").json()
        self.assertEqual(root["name"], "RSL235_Node")

        self.wait_for(self.all_receiving, timeout=10.0,
                      message="all simulators receiving")
        self.ensure_baseline()

        # 1) full measurement via trigger + spawned box
        event = self.run_box("unit-001", 0.6, 0.4, 0.5, after=0)
        record = event["value"]
        self.assertEqual(event["id"], record["measurement_id"])
        self.assertEqual(record["ref"], "unit-001")
        self.assertEqual(record["trigger_source"], "test")
        self.assertEqual(record["outcome"], "completed")
        self.assertAlmostEqual(record["width_m"]["median"], 0.6, delta=0.05)
        self.assertAlmostEqual(record["height_m"]["median"], 0.4, delta=0.05)
        self.assertAlmostEqual(record["length_m"], 0.5, delta=0.08)
        self.assertEqual(record["quality"]["degraded_lasers"], [])

        # history endpoints
        listing = client.get("/measurements").json()
        self.assertEqual(listing[0]["measurement_id"], record["measurement_id"])
        self.assertNotIn("samples", listing[0])
        full = client.get("/measurements/" + record["measurement_id"]).json()
        self.assertIn("samples", full)
        self.assertGreater(len(full["samples"]), 5)
        self.assertEqual(client.get("/measurements/nope").status_code, 404)

        # 2) trigger with nothing on the belt -> timeout outcome
        cursor = event["seq"]
        response = self.post_trigger({"source": "test"})
        self.assertEqual(response.status_code, 200)
        data = client.get("/events",
                          params={"after": cursor, "timeout": 10}).json()
        self.assertTrue(data["events"], "expected a timeout event")
        self.assertEqual(data["events"][-1]["type"], "timeout")

        # 3) live + setup page
        live = client.get("/live").json()
        self.assertIn(live["state"], ("idle", "armed"))
        self.assertIn("points", live)
        self.assertEqual(len(live["lasers"]), 3)
        page = client.get("/setup")
        self.assertEqual(page.status_code, 200)
        self.assertIn("text/html", page.headers["content-type"])
        self.assertIn("RSL235_Node", page.text)

        # 4) config read + update persists to the TOML file
        config = client.get("/config").json()
        self.assertEqual(config["gate"]["conveyor_speed_mps"], 0.5)
        response = client.post("/config", json={
            "gate": {"conveyor_speed_mps": 0.8}})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["gate"]["conveyor_speed_mps"], 0.8)
        text = self.config_path.read_text()
        self.assertIn("0.8", text)
        self.assertIn("comment must survive", text)

        # invalid updates are rejected and not written
        self.assertEqual(client.post("/config", json={
            "gate": {"conveyor_speed_mps": -2}}).status_code, 422)
        self.assertEqual(client.post("/config", json={
            "lasers": [{"id": "ghost", "port": 1}]}).status_code, 422)

        # 5) trigger rejected while busy
        self.post_trigger({})
        spawn = client.post("/mock/spawn",
                            json={"length_m": 2.0, "width_m": 0.5,
                                  "height_m": 0.3})
        self.assertEqual(spawn.status_code, 200)
        self.wait_for(lambda: client.get("/live").json()["state"] == "measuring",
                      timeout=5.0, message="measuring state")
        busy = client.post("/trigger", json={})
        self.assertEqual(busy.status_code, 409)
        client.post("/abort")
        self.assertIn(client.get("/live").json()["state"], ("idle",))


@unittest.skipUnless(HAVE_FASTAPI, "fastapi/httpx not installed")
class WebhookApiTests(ApiTests):
    """Same fixture but with a webhook configured that fails twice."""

    webhook_status_plan = (500, 500, 200)

    def test_node_end_to_end(self):  # replace: only the webhook flow here
        self.wait_for(self.all_receiving, timeout=10.0,
                      message="all simulators receiving")
        self.ensure_baseline()
        event = self.run_box("hook-001", 0.5, 0.3, 0.4, after=0)

        self.wait_for(lambda: len(self.capture.received) >= 3,
                      timeout=10.0, message="webhook retries")
        payload = self.capture.received[-1]
        self.assertEqual(payload["schema"], "rsl235_node.measurement.v1")
        self.assertEqual(payload["gate_id"], "test-gate")
        self.assertEqual(payload["ref"], "hook-001")
        self.assertEqual(payload["measurement_id"],
                         event["value"]["measurement_id"])
        # all three attempts carried the same payload
        self.assertEqual(self.capture.received[0]["measurement_id"],
                         payload["measurement_id"])

        self.wait_for(
            lambda: (self.client.get("/health").json()["webhook_last_result"]
                     or {}).get("ok") is True,
            timeout=5.0, message="webhook result in /health")
        result = self.client.get("/health").json()["webhook_last_result"]
        self.assertEqual(result["attempts"], 3)


if __name__ == "__main__":
    unittest.main()
