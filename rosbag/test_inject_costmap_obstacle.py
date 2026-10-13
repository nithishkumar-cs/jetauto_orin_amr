"""Geometry tests for the isolated synthetic costmap scan fixture."""

import math
import unittest

from sensor_msgs.msg import LaserScan

from inject_costmap_obstacle import target_beam


class InjectCostmapObstacleTest(unittest.TestCase):
    def setUp(self):
        self.scan = LaserScan()
        self.scan.angle_min = -math.pi
        self.scan.angle_increment = math.pi / 360
        self.scan.range_min = 0.15
        self.scan.range_max = 8.0
        self.scan.ranges = [8.0] * 720

    def test_target_ahead_with_lidar_offset(self):
        beam = target_beam(
            self.scan, (1.0, 0.0, 0.0, 0.0), (2.0, 0.0),
            (0.0, 0.0, 0.0), 0.15,
        )
        self.assertIsNotNone(beam)
        self.assertEqual(beam[0], 360)
        self.assertAlmostEqual(beam[1], 1.85)

    def test_map_alignment_and_robot_yaw(self):
        beam = target_beam(
            self.scan, (1.0, 1.0, 2.0, math.pi / 2), (0.0, 5.0),
            (-1.0, 1.0, 0.0), 0.15,
        )
        self.assertIsNotNone(beam)
        self.assertEqual(beam[0], 360)
        self.assertAlmostEqual(beam[1], 1.85)

    def test_out_of_range_target_is_skipped(self):
        self.assertIsNone(target_beam(
            self.scan, (1.0, 0.0, 0.0, 0.0), (10.0, 0.0),
            (0.0, 0.0, 0.0), 0.15,
        ))


if __name__ == "__main__":
    unittest.main()
