"""Pure preflight checks for the bounded Isaac FollowPath runner."""

import unittest

from tests.integration_tests.run_isaac_follow_path import verify_start_pose


class StartPoseTest(unittest.TestCase):
    """Require independent map/scene agreement before executing motion."""

    def test_rejects_observed_live_mismatch(self):
        """The 0.260 m mismatch from the live preflight blocks execution."""
        with self.assertRaisesRegex(ValueError, "exceeds"):
            verify_start_pose((0.9575, -0.0163), (1.2175, 0.0))

    def test_accepts_aligned_pose(self):
        """A map pose within the admission tolerance remains usable."""
        self.assertAlmostEqual(verify_start_pose((1.20, 0.01), (1.2175, 0.0)), 0.0201556, places=5)


if __name__ == "__main__":
    unittest.main()
