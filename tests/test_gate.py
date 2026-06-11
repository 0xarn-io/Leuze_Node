"""GateEngine state machine tests with a fake clock + stub laser sources,
plus LaserWorker scan processing and EventQueue semantics."""

import pathlib
import sys
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from pyrsl235.udt import MeasurementContour, UdtScan

from config import LaserConfig, Pose, Sector, build_config, load_config
from events import EventQueue
from gate import (ARMED, IDLE, MEASURING, GateEngine, LaserProfile,
                  LaserWorker)

DT = 0.025  # 40 Hz tick


def make_config(**gate_overrides):
    import tomllib
    with open(ROOT / "config.toml", "rb") as handle:
        raw = tomllib.load(handle)
    raw["gate"].update(gate_overrides)
    return build_config(raw)


class StubSource:
    """Minimal LaserWorker stand-in for engine tests."""

    def __init__(self, laser_id):
        self.laser_id = laser_id
        self.latest = None
        self.is_receiving = True
        self.baseline_ok = True
        self.baseline_progress = None

    def health(self):
        return {"id": self.laser_id, "is_receiving": self.is_receiving}

    def begin_baseline(self, scans):
        pass

    def cancel_baseline(self):
        pass


class FakeClock:
    def __init__(self, start=1000.0):
        self.t = start

    def __call__(self):
        return self.t


def present_points(width=0.6, height=0.4):
    """12 fused points whose trimmed extents give exactly width x height."""
    half = width / 2.0
    return tuple([(-half, 0.05)] * 4 + [(half, 0.05)] * 4 + [(0.0, height)] * 4)


class EngineHarness:
    def __init__(self, **gate_overrides):
        self.config = make_config(**gate_overrides)
        self.clock = FakeClock()
        self.events = EventQueue()
        self.sources = [StubSource("left"), StubSource("top"),
                        StubSource("right")]
        self.engine = GateEngine(self.config, self.sources, self.events,
                                 clock=self.clock)

    def feed(self, points_per_source, ticks):
        """Advance `ticks` ticks with every source publishing fresh points."""
        for _ in range(ticks):
            self.clock.t += DT
            for source, pts in zip(self.sources, points_per_source):
                if pts is None:
                    continue  # leave stale
                source.latest = LaserProfile(
                    laser_id=source.laser_id, scan_number=0,
                    received_at=time.time(), monotonic_at=self.clock.t,
                    points=pts, n_beams=271, n_foreground=len(pts))
            self.engine.tick(self.clock.t)

    def feed_all(self, points, ticks):
        share = len(points) // 3
        split = [tuple(points[:share]), tuple(points[share:2 * share]),
                 tuple(points[2 * share:])]
        self.feed(split, ticks)

    def feed_empty(self, ticks):
        self.feed([(), (), ()], ticks)


class GateEngineTests(unittest.TestCase):
    def test_arm_then_timeout(self):
        h = EngineHarness(arm_timeout_s=1.0)
        ok, msg = h.engine.trigger(source="plc", ref="u-1")
        self.assertTrue(ok)
        self.assertEqual(h.engine.state, ARMED)
        h.feed_empty(int(1.2 / DT))
        self.assertEqual(h.engine.state, IDLE)
        events = h.events.collect(0)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "timeout")
        record = events[0].value
        self.assertEqual(record["outcome"], "timeout")
        self.assertEqual(record["ref"], "u-1")
        self.assertIsNone(record["length_m"])

    def test_full_pass_measures_w_h_l(self):
        h = EngineHarness()
        h.feed_empty(3)
        ok, _ = h.engine.trigger(source="test", ref="abc")
        self.assertTrue(ok)
        h.feed_all(list(present_points(0.6, 0.4)), 81)
        self.assertEqual(h.engine.state, MEASURING)
        h.feed_empty(6)
        self.assertEqual(h.engine.state, IDLE)

        events = h.events.collect(0)
        self.assertEqual(events[-1].type, "measurement")
        record = events[-1].value
        self.assertEqual(record["outcome"], "completed")
        self.assertEqual(record["ref"], "abc")
        self.assertEqual(record["trigger_source"], "test")
        # edges are backdated, so duration == 81 ticks exactly
        self.assertAlmostEqual(record["duration_s"], 81 * DT, places=6)
        self.assertAlmostEqual(record["length_m"], 81 * DT * 0.5, places=4)
        self.assertAlmostEqual(record["width_m"]["max"], 0.6, places=4)
        self.assertAlmostEqual(record["height_m"]["max"], 0.4, places=4)
        self.assertFalse(record["quality"]["truncated"])
        self.assertEqual(record["quality"]["degraded_lasers"], [])
        self.assertEqual(sorted(record["quality"]["lasers_used"]),
                         ["left", "right", "top"])
        self.assertGreater(record["n_samples"], 70)

    def test_speed_override_scales_length(self):
        h = EngineHarness()
        h.engine.trigger(source="plc", speed_mps=0.25)
        h.feed_all(list(present_points()), 81)
        h.feed_empty(6)
        record = h.events.collect(0)[-1].value
        self.assertAlmostEqual(record["conveyor_speed_mps"], 0.25)
        self.assertAlmostEqual(record["length_m"], 81 * DT * 0.25, places=4)

    def test_trigger_rejected_while_measuring_and_rearm_while_armed(self):
        h = EngineHarness()
        ok, msg = h.engine.trigger()
        self.assertTrue(ok)
        ok, msg = h.engine.trigger(ref="second")     # re-arm allowed
        self.assertTrue(ok)
        self.assertIn("re-armed", msg)
        h.feed_all(list(present_points()), 5)
        self.assertEqual(h.engine.state, MEASURING)
        ok, msg = h.engine.trigger()
        self.assertFalse(ok)
        self.assertIn("busy", msg)
        h.feed_empty(6)
        self.assertEqual(h.events.collect(0)[-1].value["ref"], "second")

    def test_abort(self):
        h = EngineHarness()
        h.engine.trigger()
        h.feed_all(list(present_points()), 10)
        self.assertTrue(h.engine.abort())
        self.assertEqual(h.engine.state, IDLE)
        self.assertEqual(h.events.collect(0)[-1].type, "aborted")
        self.assertFalse(h.engine.abort())  # nothing to abort anymore

    def test_sensor_loss_when_all_lasers_go_stale(self):
        h = EngineHarness()
        h.engine.trigger()
        h.feed_all(list(present_points()), 10)
        self.assertEqual(h.engine.state, MEASURING)
        # nobody publishes anymore -> profiles age out
        for _ in range(10):
            h.clock.t += DT
            h.engine.tick(h.clock.t)
        self.assertEqual(h.engine.state, IDLE)
        self.assertEqual(h.events.collect(0)[-1].type, "sensor_loss")

    def test_degraded_laser_recorded_but_completes(self):
        h = EngineHarness()
        h.engine.trigger()
        points = list(present_points())
        share = len(points) // 3
        # right laser never updates -> stale; the others carry the event
        h.feed([tuple(points[:share + 4]), tuple(points[share + 4:]), None], 81)
        self.assertEqual(h.engine.state, MEASURING)
        h.feed([(), (), None], 6)
        record = h.events.collect(0)[-1].value
        self.assertEqual(record["outcome"], "completed")
        self.assertEqual(record["quality"]["degraded_lasers"], ["right"])
        self.assertEqual(record["quality"]["lasers_used"], ["left", "top"])

    def test_max_event_truncates(self):
        h = EngineHarness(max_event_s=0.5)
        h.engine.trigger()
        h.feed_all(list(present_points()), 30)  # 0.75 s present
        record = h.events.collect(0)[-1].value
        self.assertEqual(record["outcome"], "completed")
        self.assertTrue(record["quality"]["truncated"])

    def test_holdoff_blocks_immediate_retrigger(self):
        h = EngineHarness(retrigger_holdoff_s=0.5, arm_timeout_s=0.2)
        h.engine.trigger()
        h.feed_empty(int(0.3 / DT))      # -> timeout outcome
        self.assertEqual(h.engine.state, IDLE)
        ok, msg = h.engine.trigger()
        self.assertFalse(ok)
        self.assertIn("holdoff", msg)
        h.clock.t += 0.6
        ok, _ = h.engine.trigger()
        self.assertTrue(ok)

    def test_aggregates_use_max_and_median(self):
        h = EngineHarness()
        h.engine.trigger()
        h.feed_all(list(present_points(0.5, 0.3)), 40)
        h.feed_all(list(present_points(0.6, 0.3)), 41)
        h.feed_empty(6)
        record = h.events.collect(0)[-1].value
        self.assertAlmostEqual(record["width_m"]["max"], 0.6, places=4)
        self.assertAlmostEqual(record["width_m"]["median"], 0.6, places=4)
        self.assertAlmostEqual(record["height_m"]["max"], 0.3, places=4)


class EventQueueTests(unittest.TestCase):
    def test_seq_collect_and_latest(self):
        q = EventQueue()
        q.publish("measurement", {"a": 1}, event_id="m1")
        q.publish("timeout", {"b": 2}, event_id="m2")
        self.assertEqual(q.latest_seq, 2)
        events = q.collect(after=1)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].seq, 2)
        self.assertEqual(events[0].id, "m2")
        self.assertTrue(events[0].ts.endswith("Z"))

    def test_wait_times_out_quickly(self):
        q = EventQueue()
        start = time.monotonic()
        self.assertEqual(q.wait(after=0, timeout=0.1), [])
        self.assertLess(time.monotonic() - start, 1.0)

    def test_wait_wakes_on_publish(self):
        import threading
        q = EventQueue()
        threading.Timer(0.05, lambda: q.publish("measurement", {})).start()
        events = q.wait(after=0, timeout=2.0)
        self.assertEqual(len(events), 1)

    def test_bounded_buffer_drops_oldest(self):
        q = EventQueue(maxlen=3)
        for i in range(5):
            q.publish("measurement", i)
        seqs = [e.seq for e in q.collect(0)]
        self.assertEqual(seqs, [3, 4, 5])
        self.assertEqual(q.latest_seq, 5)


class LaserWorkerTests(unittest.TestCase):
    """Scan -> profile pipeline with hand-built UdtScans (no sockets)."""

    class FakeReceiver:
        is_receiving = True
        latest_status = None
        stats = {}

        def start(self):
            pass

        def stop(self):
            pass

    def make_worker(self):
        config = load_config(ROOT / "config.toml")
        laser = config.laser("top")
        return LaserWorker(laser, lambda: config.gate,
                           receiver_factory=lambda l, cb: self.FakeReceiver())

    @staticmethod
    def make_scan(distances, scan_number=1):
        contour = MeasurementContour(start_index=1325, stop_index=1425,
                                     index_interval=10, reserved=0)
        return UdtScan(scan_number=scan_number, telegram_id=6,
                       distances_mm=tuple(distances), signal_strengths=None,
                       num_blocks=1, contour=contour, status=None,
                       received_at=time.time())

    def test_baseline_then_foreground_point(self):
        worker = self.make_worker()
        # 11 beams at -5..+5 deg around straight down; empty gate = 2000 mm
        empty = [2000] * 11
        worker.begin_baseline(3)
        for i in range(3):
            worker._on_scan(self.make_scan(empty, scan_number=i))
        self.assertIsNone(worker.baseline_progress)
        self.assertTrue(worker.baseline_ok)
        self.assertEqual(worker.baseline, tuple(empty))
        # baseline scans produce no foreground
        self.assertEqual(worker.latest.n_foreground, 0)

        # a 0.4 m tall box under the center beam (index 5 = 0 deg)
        scan = [2000] * 11
        scan[5] = 1600
        worker._on_scan(self.make_scan(scan, scan_number=10))
        profile = worker.latest
        self.assertEqual(profile.n_foreground, 1)
        self.assertEqual(len(profile.points), 1)
        x, z = profile.points[0]
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(z, 0.4, places=6)

    def test_without_baseline_roi_still_filters_belt(self):
        worker = self.make_worker()
        worker._on_scan(self.make_scan([2000] * 11))
        # all echoes hit the belt plane (z=0) -> outside ROI z_min 0.03
        self.assertEqual(len(worker.latest.points), 0)


if __name__ == "__main__":
    unittest.main()
