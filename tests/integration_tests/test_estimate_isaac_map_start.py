"""Regression checks for independent LiDAR-to-occupancy-map start estimation."""

import unittest

import numpy as np
from scipy.ndimage import distance_transform_edt

from estimate_isaac_map_start import estimate_pose, score_pose


class MapStartTest(unittest.TestCase):
    """Check that map pixel geometry distinguishes a true pose from a shifted one."""

    def test_asymmetric_map_recovers_known_pose_with_turn(self):
        """Recover translation and nonzero yaw without localizer input."""
        occupied = np.zeros((80, 80), dtype=bool)
        occupied[14, 16] = True
        occupied[35, 65] = True
        occupied[67, 27] = True
        field = distance_transform_edt(~occupied) * 0.05
        config = {"resolution": 0.05, "origin": [-2.0, -2.0, 0.0]}
        world = np.array([
            [-2.0 + (16.5) * 0.05, -2.0 + (80 - 14 - 0.5) * 0.05],
            [-2.0 + (65.5) * 0.05, -2.0 + (80 - 35 - 0.5) * 0.05],
            [-2.0 + (27.5) * 0.05, -2.0 + (80 - 67 - 0.5) * 0.05],
        ])
        expected_xy = (0.10, -0.05)
        expected_yaw = 0.17
        delta = world - np.asarray(expected_xy)
        cosine, sine = np.cos(expected_yaw), np.sin(expected_yaw)
        points = np.column_stack((
            cosine * delta[:, 0] + sine * delta[:, 1],
            -sine * delta[:, 0] + cosine * delta[:, 1],
        ))

        cost, _, x, y, yaw = estimate_pose(
            field, config, points, radius=0.2, center_yaw=0.16
        )

        self.assertAlmostEqual(x, expected_xy[0], delta=0.015)
        self.assertAlmostEqual(y, expected_xy[1], delta=0.015)
        self.assertAlmostEqual(yaw, expected_yaw, delta=0.02)
        self.assertLess(cost, 0.01)
        shifted_cost, _ = score_pose(field, config, points, 0.35, -0.05, expected_yaw)
        self.assertGreater(shifted_cost, 0.1)


if __name__ == "__main__":
    unittest.main()
