"""Geometry + configuration unit tests (no FastAPI, no sockets)."""

import math
import pathlib
import shutil
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import config as config_mod
from config import ConfigError, Pose, Roi, Sector, build_config, load_config
from geometry import (TransformTable, beam_point, beam_theta_deg,
                      foreground_mask, trimmed_extent)


class TransformTests(unittest.TestCase):
    def test_top_laser_straight_down(self):
        pose = Pose(x_m=0.0, z_m=2.0, rotation_deg=-90.0)
        x, z = beam_point(pose, angle_deg=0.0, distance_m=1.6)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(z, 0.4, places=9)

    def test_left_laser_horizontal(self):
        pose = Pose(x_m=-1.0, z_m=0.5, rotation_deg=0.0)
        x, z = beam_point(pose, angle_deg=0.0, distance_m=0.7)
        self.assertAlmostEqual(x, -0.3, places=9)
        self.assertAlmostEqual(z, 0.5, places=9)

    def test_right_laser_mirrored_vs_plain(self):
        plain = Pose(x_m=1.0, z_m=0.5, rotation_deg=180.0, mirror=False)
        mirrored = Pose(x_m=1.0, z_m=0.5, rotation_deg=180.0, mirror=True)
        # +30 deg from a mirrored device equals -30 deg from a plain one
        self.assertAlmostEqual(beam_theta_deg(mirrored, 30.0),
                               beam_theta_deg(plain, -30.0))
        x_up, z_up = beam_point(plain, 30.0, 1.0)     # 210 deg: down-left
        self.assertLess(z_up, 0.5)
        x_m, z_m = beam_point(mirrored, 30.0, 1.0)    # 150 deg: up-left
        self.assertGreater(z_m, 0.5)

    def test_transform_table_sector_and_roi(self):
        pose = Pose(x_m=0.0, z_m=2.0, rotation_deg=-90.0)
        sector = Sector(min_deg=-10.0, max_deg=10.0)
        angles = [-20.0, -5.0, 0.0, 5.0, 20.0]
        table = TransformTable(pose, sector, angles)
        self.assertEqual(table.mask, [False, True, True, True, False])
        roi = Roi(x_min_m=-0.9, x_max_m=0.9, z_min_m=0.03, z_max_m=1.9)
        distances = [1600, 1600, 1600, 1600, 1600]
        points = table.project(distances, roi=roi)
        self.assertEqual(len(points), 3)  # out-of-sector beams dropped
        x, z = points[1]
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(z, 0.4, places=9)
        # invalid (0) distances are skipped
        self.assertEqual(len(table.project([0, 0, 1600, 0, 0], roi=roi)), 1)


class MaskAndExtentTests(unittest.TestCase):
    def test_foreground_mask_without_baseline(self):
        self.assertEqual(foreground_mask([0, 500, 2000], None, 60),
                         [False, True, True])

    def test_foreground_mask_against_baseline(self):
        baseline = [3000, 3000, 0, 3000]
        distances = [2950, 2930, 500, 0]
        # 2950 is within the 60 mm margin -> background;
        # 2930 < 2940 -> foreground; echo where baseline had none -> foreground;
        # no echo now -> never foreground.
        self.assertEqual(foreground_mask(distances, baseline, 60),
                         [False, True, True, False])

    def test_foreground_mask_baseline_shorter_than_scan(self):
        self.assertEqual(foreground_mask([1000, 1000], [3000], 60),
                         [True, True])  # missing baseline beam == no echo

    def test_trimmed_extent(self):
        values = [5.0, 0.0, 1.0, 2.0, 3.0, 4.0, 10.0]
        self.assertEqual(trimmed_extent(values, 0), (0.0, 10.0))
        self.assertEqual(trimmed_extent(values, 1), (1.0, 5.0))
        self.assertEqual(trimmed_extent([1.0, 2.0], 5), (1.0, 2.0))
        self.assertIsNone(trimmed_extent([], 1))


class ConfigTests(unittest.TestCase):
    def test_repo_default_config_loads(self):
        config = load_config(ROOT / "config.toml")
        self.assertEqual(len(config.lasers), 3)
        self.assertEqual([l.id for l in config.lasers], ["left", "top", "right"])
        self.assertEqual(config.server.port, 9020)
        self.assertTrue(config.node.mock)
        self.assertEqual(config.laser("top").pose.rotation_deg, -90.0)

    def _raw(self):
        import tomllib
        with open(ROOT / "config.toml", "rb") as handle:
            return tomllib.load(handle)

    def test_duplicate_ports_rejected(self):
        raw = self._raw()
        raw["lasers"][1]["port"] = raw["lasers"][0]["port"]
        with self.assertRaisesRegex(ConfigError, "unique"):
            build_config(raw)

    def test_bad_roi_rejected(self):
        raw = self._raw()
        raw["gate"]["roi"]["x_min_m"] = 2.0
        with self.assertRaisesRegex(ConfigError, "roi"):
            build_config(raw)

    def test_negative_speed_rejected(self):
        raw = self._raw()
        raw["gate"]["conveyor_speed_mps"] = -1.0
        with self.assertRaisesRegex(ConfigError, "conveyor_speed"):
            build_config(raw)

    def test_apply_update_persists_and_keeps_comments(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "config.toml"
            shutil.copy(ROOT / "config.toml", path)
            updated = config_mod.apply_update(path, {
                "gate": {"conveyor_speed_mps": 0.75,
                         "roi": {"x_max_m": 1.1}},
                "webhook": {"url": "http://example.invalid/hook"},
                "lasers": [{"id": "top", "port": 4051,
                            "pose": {"z_m": 2.2}}],
            })
            self.assertEqual(updated.gate.conveyor_speed_mps, 0.75)
            self.assertEqual(updated.gate.roi.x_max_m, 1.1)
            self.assertEqual(updated.laser("top").port, 4051)
            self.assertEqual(updated.laser("top").pose.z_m, 2.2)
            self.assertEqual(updated.laser("top").pose.rotation_deg, -90.0)
            text = path.read_text()
            self.assertIn("# RSL235_Node configuration.", text)  # comments survive
            reloaded = load_config(path)
            self.assertEqual(reloaded.webhook.url, "http://example.invalid/hook")

    def test_apply_update_rejects_invalid_without_writing(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "config.toml"
            shutil.copy(ROOT / "config.toml", path)
            before = path.read_text()
            with self.assertRaises(ConfigError):
                config_mod.apply_update(path, {"gate": {"sample_rate_hz": 0.0}})
            with self.assertRaises(ConfigError):
                config_mod.apply_update(path, {"lasers": [{"id": "nope", "port": 1}]})
            with self.assertRaises(ConfigError):
                config_mod.apply_update(path, {"unknown_section": {}})
            self.assertEqual(path.read_text(), before)


if __name__ == "__main__":
    unittest.main()
