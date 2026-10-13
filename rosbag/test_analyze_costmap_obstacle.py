"""Unit tests for costmap target-cell counting."""

import math
import unittest

from nav_msgs.msg import OccupancyGrid

from analyze_costmap_obstacle import lethal_near


class AnalyzeCostmapObstacleTest(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid()
        self.grid.info.resolution = 0.1
        self.grid.info.width = 20
        self.grid.info.height = 20
        self.grid.info.origin.orientation.w = 1.0
        self.grid.data = [0] * 400

    def test_counts_only_lethal_cells_inside_target_radius(self):
        self.grid.data[5 * 20 + 5] = 100
        self.grid.data[5 * 20 + 6] = 99
        self.grid.data[8 * 20 + 8] = 100
        self.assertEqual(lethal_near(self.grid, 0.55, 0.55, 0.15), 1)

    def test_rotated_origin_is_respected(self):
        self.grid.info.origin.position.x = 1.0
        self.grid.info.origin.position.y = 2.0
        self.grid.info.origin.orientation.z = math.sin(math.pi / 4)
        self.grid.info.origin.orientation.w = math.cos(math.pi / 4)
        self.grid.data[5 * 20 + 5] = 100
        self.assertEqual(lethal_near(self.grid, 0.45, 2.55, 0.08), 1)
        self.assertEqual(lethal_near(self.grid, 1.55, 2.55, 0.08), 0)


if __name__ == "__main__":
    unittest.main()
