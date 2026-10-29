"""Regression checks for bounded Isaac collision-zone action teardown."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from tests.integration_tests.measure_collision_zones import finish_action


class FinishActionTest(unittest.TestCase):
    """A time limit must stop the robot and cancel the live Nav2 goal."""

    def test_pending_goal_is_cancelled_on_timeout(self):
        """An elapsed time limit follows the same stop path as an obstacle abort."""
        node = Mock()
        result_future = Mock()
        result_future.done.return_value = False
        cancel_future = Mock()
        cancel_future.done.return_value = True
        handle = Mock()
        handle.cancel_goal_async.return_value = cancel_future

        status, reason = finish_action(node, handle, result_future, None, 50.0)

        self.assertIsNone(status)
        self.assertEqual(reason, "timed out after 50.0 s")
        node.latch_estop.assert_called_once_with()
        handle.cancel_goal_async.assert_called_once_with()

    def test_completed_goal_needs_no_cancel(self):
        """A finished action returns its result without sending a cancel request."""
        node = Mock()
        result_future = Mock()
        result_future.done.return_value = True
        result_future.result.return_value = SimpleNamespace(status=4)
        handle = Mock()

        status, reason = finish_action(node, handle, result_future, None, 50.0)

        self.assertEqual(status, 4)
        self.assertIsNone(reason)
        node.latch_estop.assert_not_called()
        handle.cancel_goal_async.assert_not_called()


if __name__ == "__main__":
    unittest.main()
