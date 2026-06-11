"""Scene ray-cast truths + simulator-to-receiver wire test (loopback UDP)."""

import pathlib
import socket
import sys
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from pyrsl235 import RSL235Udt

from config import Pose
from mock import BoxPassScene, SceneUdtSimulator

TOP = Pose(x_m=0.0, z_m=2.0, rotation_deg=-90.0)
LEFT = Pose(x_m=-1.0, z_m=0.5, rotation_deg=0.0)


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.scene = BoxPassScene(half_width_m=2.0, ceiling_m=2.5)

    def test_empty_gate_distances(self):
        # top laser straight down hits the belt plane 2 m below
        self.assertEqual(self.scene.distance_mm((0.0, 2.0), -90.0, now=0.0), 2000)
        # left laser straight across hits the far wall 3 m away
        self.assertEqual(self.scene.distance_mm((-1.0, 0.5), 0.0, now=0.0), 3000)
        # straight up hits the ceiling
        self.assertEqual(self.scene.distance_mm((0.0, 2.0), 90.0, now=0.0), 500)

    def test_box_shortens_beams_for_its_lifetime(self):
        self.scene.spawn(width_m=0.6, height_m=0.4, length_m=0.5,
                         speed_mps=0.5, now=100.0)
        # active for length/speed = 1.0 s
        self.assertEqual(self.scene.distance_mm((0.0, 2.0), -90.0, now=100.1),
                         1600)
        # left beam at z=0.5 passes over the 0.4 m box -> far wall
        self.assertEqual(self.scene.distance_mm((-1.0, 0.5), 0.0, now=100.1),
                         3000)
        # but a 0.6 m tall box would catch it at its near side (x=-0.3)
        self.scene.spawn(width_m=0.6, height_m=0.6, length_m=0.5,
                         speed_mps=0.5, now=100.0)
        self.assertEqual(self.scene.distance_mm((-1.0, 0.5), 0.0, now=100.1),
                         700)
        # expired after 1.0 s
        self.assertEqual(self.scene.distance_mm((0.0, 2.0), -90.0, now=101.05),
                         2000)

    def test_box_miss_returns_static(self):
        self.scene.spawn(width_m=0.2, height_m=0.2, length_m=1.0,
                         speed_mps=0.5, now=0.0)
        # beam pointing up never sees the box
        self.assertEqual(self.scene.distance_mm((0.0, 2.0), 90.0, now=0.1), 500)


class SceneSimulatorWireTests(unittest.TestCase):
    """SceneUdtSimulator -> real RSL235Udt over loopback UDP."""

    def test_receiver_sees_box_appear(self):
        scene = BoxPassScene()
        port = free_udp_port()
        simulator = SceneUdtSimulator(scene, TOP, target_port=port)
        receiver = RSL235Udt(port=port, bind_address="127.0.0.1")

        # contour (25, 2725, 10): beam k points at (25 + 10k) * 0.1 - 137.5 deg
        center = (1375 - 25) // 10  # 0 deg -> straight down for the top pose

        with receiver:
            with simulator:
                scan = receiver.get_scan(timeout=2.0)
                self.assertEqual(scan.num_beams, 271)
                self.assertAlmostEqual(scan.angles_deg()[center], 0.0, places=6)
                self.assertEqual(scan.distances_mm[center], 2000)

                scene.spawn(width_m=0.6, height_m=0.4, length_m=5.0,
                            speed_mps=0.5)
                deadline = time.monotonic() + 2.0
                seen = None
                while time.monotonic() < deadline:
                    scan = receiver.get_scan(timeout=1.0)
                    if scan.distances_mm[center] != 2000:
                        seen = scan.distances_mm[center]
                        break
                self.assertEqual(seen, 1600)
                # a beam outside the box (45 deg out) still sees background
                outside = center + 450 // 10
                self.assertGreater(scan.distances_mm[outside], 2000)


if __name__ == "__main__":
    unittest.main()
