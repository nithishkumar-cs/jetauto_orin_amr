"""Unit tests for diagnostic loop-match geometry and log parsing."""

import math
import unittest

from audit_karto_loop_matches import Pose2, audit, parse_events, relative_pose, sensor_pose


class AuditKartoLoopMatchesTest(unittest.TestCase):
    def test_relative_pose_uses_reference_heading(self):
        relative = relative_pose(Pose2(1.0, 2.0, math.pi / 2), Pose2(1.0, 3.0, math.pi))
        self.assertAlmostEqual(relative.x, 1.0)
        self.assertAlmostEqual(relative.y, 0.0)
        self.assertAlmostEqual(relative.yaw, math.pi / 2)

    def test_sensor_offset_rotates_with_base(self):
        sensor = sensor_pose(Pose2(2.0, 3.0, math.pi / 2), Pose2(0.15, 0.0, 0.0))
        self.assertAlmostEqual(sensor.x, 2.0)
        self.assertAlmostEqual(sensor.y, 3.15)

    def test_parse_and_audit_match_against_truth(self):
        log = "\n".join(
            [
                "KARTO_LOOP_MATCH scan_id=7 scan_time=10 closest_id=2 closest_time=2 "
                "fine_response=0.8 prior_sensor_pose=1 0 0 matched_sensor_pose=2.15 0 0 "
                "closest_sensor_pose=0.15 0 0",
                "KARTO_LOOP_LINK_GATE scan_id=7 closest_id=2 squared_distance=1 "
                "limit_squared=2.25 passed=1",
                "KARTO_FINE_ACCEPTED_LOOP_CANDIDATE scan_id=7 scan_time=10 candidate_chain_size=5",
            ]
        )
        events = parse_events(log)
        self.assertEqual(len(events), 1)
        rows = audit(events, [(2.0, 0.0, 0.0, 0.0), (10.0, 2.0, 0.0, 0.0)],
                     Pose2(0.15, 0.0, 0.0), 0.02)
        self.assertEqual(rows[0]["relative_position_error_m"], 0.0)
        self.assertEqual(rows[0]["relative_heading_error_rad"], 0.0)
        self.assertEqual(rows[0]["truth_center_distance_m"], 2.0)
        self.assertTrue(rows[0]["gate_passed"])
        self.assertIsNone(rows[0]["new_graph_edge"])

    def test_new_edge_marker_is_correlated_with_loop_pair(self):
        log = "\n".join(
            [
                "KARTO_GRAPH_EDGE_CREATED source_id=1 target_id=2",
                "KARTO_LOOP_MATCH scan_id=7 scan_time=10 closest_id=2 closest_time=2 "
                "fine_response=0.8 prior_sensor_pose=1 0 0 matched_sensor_pose=2 0 0 "
                "closest_sensor_pose=0 0 0",
                "KARTO_LOOP_LINK_GATE scan_id=7 closest_id=2 squared_distance=1 "
                "limit_squared=2.25 passed=1",
                "KARTO_GRAPH_EDGE_CREATED source_id=2 target_id=7",
                "KARTO_FINE_ACCEPTED_LOOP_CANDIDATE scan_id=7 scan_time=10 candidate_chain_size=5",
            ]
        )
        self.assertTrue(parse_events(log)[0]["new_graph_edge"])

    def test_gate_pass_without_new_edge_is_not_counted_as_created(self):
        log = "\n".join(
            [
                "KARTO_GRAPH_EDGE_CREATED source_id=1 target_id=2",
                "KARTO_LOOP_MATCH scan_id=7 scan_time=10 closest_id=2 closest_time=2 "
                "fine_response=0.8 prior_sensor_pose=1 0 0 matched_sensor_pose=2 0 0 "
                "closest_sensor_pose=0 0 0",
                "KARTO_LOOP_LINK_GATE scan_id=7 closest_id=2 squared_distance=1 "
                "limit_squared=2.25 passed=1",
                "KARTO_GRAPH_EDGE_CREATED source_id=3 target_id=7",
                "KARTO_FINE_ACCEPTED_LOOP_CANDIDATE scan_id=7 scan_time=10 candidate_chain_size=5",
            ]
        )
        self.assertFalse(parse_events(log)[0]["new_graph_edge"])

    def test_mismatched_gate_is_rejected(self):
        log = "\n".join(
            [
                "KARTO_LOOP_MATCH scan_id=7 scan_time=10 closest_id=2 closest_time=2 "
                "fine_response=0.8 prior_sensor_pose=1 0 0 matched_sensor_pose=2 0 0 "
                "closest_sensor_pose=0 0 0",
                "KARTO_LOOP_LINK_GATE scan_id=7 closest_id=3 squared_distance=1 "
                "limit_squared=2.25 passed=1",
            ]
        )
        with self.assertRaisesRegex(ValueError, "differ"):
            parse_events(log)


if __name__ == "__main__":
    unittest.main()
