"""Check the offline static-scene LiDAR scorer's geometric and metric rules."""

import unittest

import numpy as np
from sensor_msgs.msg import LaserScan

from score_lidar_scene_clusters import (
    cluster_indices,
    prepare_scan,
    raycast,
    score_one,
    score_thin_feature,
)


class ScoreLidarSceneClustersTest(unittest.TestCase):
    """Exercise the scorer without depending on a local recording."""

    def test_tf_offset_changes_scene_label(self):
        """A scan's geometry is held fixed while the TF query stamp shifts."""
        scan = LaserScan()
        scan.header.stamp.nanosec = 250_000_000
        scan.angle_min = 0.0
        scan.angle_increment = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [1.25]
        times = np.arange(11) / 10
        poses = np.column_stack((times, times, np.full(11, -2.2), np.zeros(11)))
        self.assertEqual(prepare_scan(scan, poses, 0.12)[3].tolist(), [6])
        self.assertEqual(prepare_scan(scan, poses, 0.12, 0.25)[3].tolist(), [0])

    def test_raycast_picks_first_scene_surface(self):
        """A beam through the near box reaches that box before the wall."""
        distances, labels = raycast(0.0, -2.2, np.array([1.0]), np.array([0.0]))
        self.assertAlmostEqual(distances[0], 1.65)
        self.assertEqual(labels[0], 6)

    def test_cluster_splits_and_joins_full_circle_seam(self):
        """Invalid beams split candidates and adjacent end beams join."""
        x = np.array([0.0, 0.1, np.nan, 2.0, 2.1, 0.0])
        y = np.zeros(6)
        valid = np.array([True, True, False, True, True, True])
        groups = cluster_indices(x, y, valid, 0.2, 2, True)
        self.assertEqual([group.tolist() for group in groups], [[5, 0, 1], [3, 4]])
        self.assertEqual(len(cluster_indices(x, y, valid, 0.2, 4, True)), 0)

    def test_box_capture_fragment_and_merge(self):
        """Two substantial box pieces count as a split; mixed surfaces merge."""
        split = np.full(10, 6, dtype=np.int8)
        self.assertEqual(score_one(split, [np.arange(5), np.arange(5, 10)], 6),
                         (1, 1, 0, 1, 0, 0.5))
        mixed = np.array([6] * 6 + [7] * 3, dtype=np.int8)
        self.assertEqual(score_one(mixed, [np.arange(9)], 6),
                         (1, 0, 0, 0, 1, 0.0))

    def test_two_beam_feature_requires_retained_cluster(self):
        """A matching two-beam chair leg is lost at a three-point minimum."""
        labels = np.array([8, 8], dtype=np.int8)
        self.assertEqual(score_thin_feature(labels, [np.array([0, 1])], 8), (1, 1))
        self.assertEqual(score_thin_feature(labels, [], 8), (1, 0))


if __name__ == "__main__":
    unittest.main()
