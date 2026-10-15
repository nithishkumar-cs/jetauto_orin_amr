"""Unit tests for tracker scoring against synthetic ground truth."""

import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from score_trackers import (
    GroundTruth,
    GroundTruthSample,
    PoseTimeline,
    ScoringConfig,
    TrackFrame,
    TrackObservation,
    associate,
    ground_truth_from_csv,
    ground_truth_from_json,
    main,
    parse_field_overrides,
    score_frames,
)

PERIOD_S = 0.1
STEPS = 50
ROS_AVAILABLE = all(
    importlib.util.find_spec(name) for name in ("rosbag2_py", "amr_interfaces", "tf2_msgs")
)


def times():
    """Return the synthetic sample times."""
    return [round(step * PERIOD_S, 6) for step in range(STEPS)]


def person_at(time_s):
    """Return the synthetic person position (moving +y at 0.5 m/s)."""
    return 2.0, -1.0 + 0.5 * time_s


def robot_at(time_s):
    """Return the synthetic robot pose (driving +x while turning)."""
    return 0.1 * time_s, 0.0, 0.05 * time_s


def ground_truth_samples():
    """Return a static box and a moving person sampled at 10 Hz."""
    samples = []
    for time_s in times():
        samples.append(GroundTruthSample(time_s, "box", 3.0, 0.5))
        samples.append(GroundTruthSample(time_s, "person", *person_at(time_s)))
    return samples


def pose_timeline():
    """Return robot poses at 10 Hz."""
    return PoseTimeline([(time_s, *robot_at(time_s)) for time_s in times()], max_gap_s=0.2)


def to_base(time_s, x, y):
    """Express an odom point in the robot base frame at `time_s`."""
    robot_x, robot_y, yaw = robot_at(time_s)
    dx, dy = x - robot_x, y - robot_y
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def perfect_frames(person_id=lambda step: 2, person_dynamic=True, skip=()):
    """Return base_link frames that track both obstacles exactly, omitting the person at `skip` steps."""
    frames = []
    for step, time_s in enumerate(times()):
        tracks = [TrackObservation(1, *to_base(time_s, 3.0, 0.5), False)]
        if step not in skip:
            tracks.append(TrackObservation(person_id(step), *to_base(time_s, *person_at(time_s)), person_dynamic))
        frames.append(TrackFrame(time_s, "base_link", tuple(tracks)))
    return frames


class InterpolationTest(unittest.TestCase):
    def test_ground_truth_interpolates_position_and_speed(self):
        truth = GroundTruth(ground_truth_samples(), max_gap_s=0.25)
        state = truth.states_at(1.05)["person"]
        self.assertAlmostEqual(state.x, 2.0)
        self.assertAlmostEqual(state.y, -1.0 + 0.5 * 1.05)
        self.assertAlmostEqual(state.speed_mps, 0.5)
        self.assertAlmostEqual(truth.states_at(1.05)["box"].speed_mps, 0.0)
        self.assertEqual(truth.states_at(-0.01), {})
        self.assertEqual(set(truth.states_at(times()[-1])), {"box", "person"})

    def test_ground_truth_gap_is_not_bridged(self):
        samples = [GroundTruthSample(0.0, "a", 0.0, 0.0), GroundTruthSample(1.0, "a", 1.0, 0.0)]
        self.assertEqual(GroundTruth(samples, max_gap_s=0.25).states_at(0.5), {})

    def test_single_sample_obstacle_is_rejected(self):
        with self.assertRaises(ValueError):
            GroundTruth([GroundTruthSample(0.0, "a", 0.0, 0.0)], max_gap_s=0.25)

    def test_pose_yaw_interpolates_across_wrap(self):
        poses = PoseTimeline([(0.0, 0.0, 0.0, 3.1), (1.0, 2.0, 0.0, -3.1)], max_gap_s=2.0)
        pose = poses.pose_at(0.5)
        self.assertAlmostEqual(pose.x, 1.0)
        self.assertAlmostEqual(abs(pose.yaw), math.pi, places=6)
        self.assertIsNone(poses.pose_at(1.5))


class AssociationTest(unittest.TestCase):
    def test_previous_correspondence_survives_a_closer_track(self):
        tracks = np.array([[0.3, 0.0], [0.05, 0.0]])
        matches = associate({"a": (0.0, 0.0)}, tracks, [7, 8], {"a": 7}, gate_m=0.5)
        self.assertEqual(matches, {"a": 0})

    def test_assignment_maximises_gated_matches(self):
        tracks = np.array([[0.5, 0.0], [1.6, 0.0]])
        matches = associate({"a": (0.0, 0.0), "b": (1.0, 0.0)}, tracks, [1, 2], {}, gate_m=0.65)
        self.assertEqual(matches, {"a": 0, "b": 1})

    def test_tracks_outside_gate_stay_unmatched(self):
        self.assertEqual(associate({"a": (0.0, 0.0)}, np.array([[1.0, 0.0]]), [1], {}, gate_m=0.5), {})


class ScoringTest(unittest.TestCase):
    def setUp(self):
        self.truth = GroundTruth(ground_truth_samples(), max_gap_s=0.25)
        self.poses = pose_timeline()

    def test_perfect_tracker(self):
        result = score_frames(perfect_frames(), self.truth, self.poses, ScoringConfig())
        self.assertEqual(result["frames_scored"], STEPS)
        self.assertEqual(result["recall"], 1.0)
        self.assertEqual(result["id_switches"], 0)
        self.assertEqual(result["fragmentations"], 0)
        self.assertEqual(result["unmatched_track_observations"], 0)
        self.assertLess(result["position_error_m"]["max"], 1e-9)
        self.assertEqual(result["motion"]["accuracy"], 1.0)
        self.assertEqual(result["motion"]["moving_dynamic"], STEPS)
        self.assertEqual(result["motion"]["static_static"], STEPS)

    def test_id_switch_after_gap_is_also_a_fragmentation(self):
        frames = perfect_frames(person_id=lambda step: 2 if step < 20 else 9, skip=range(15, 20))
        person = score_frames(frames, self.truth, self.poses, ScoringConfig())["obstacles"]["person"]
        self.assertEqual(person["id_switches"], 1)
        self.assertEqual(person["fragmentations"], 1)
        self.assertEqual(person["distinct_track_ids"], 2)
        self.assertEqual(person["expected"] - person["expected_matched"], 5)

    def test_same_id_after_gap_is_fragmentation_only(self):
        person = score_frames(perfect_frames(skip=range(15, 20)), self.truth, self.poses,
                              ScoringConfig())["obstacles"]["person"]
        self.assertEqual((person["id_switches"], person["fragmentations"]), (0, 1))

    def test_id_switch_without_gap(self):
        frames = perfect_frames(person_id=lambda step: 2 if step < 20 else 9)
        person = score_frames(frames, self.truth, self.poses, ScoringConfig())["obstacles"]["person"]
        self.assertEqual((person["id_switches"], person["fragmentations"]), (1, 0))

    def test_motion_misclassification(self):
        result = score_frames(perfect_frames(person_dynamic=False), self.truth, self.poses, ScoringConfig())
        self.assertEqual(result["motion"]["moving_static"], STEPS)
        self.assertEqual(result["motion"]["accuracy"], 0.5)

    def test_unscored_ground_truth_absorbs_tracks(self):
        config = ScoringConfig(scored_ids=frozenset({"person"}))
        result = score_frames(perfect_frames(), self.truth, self.poses, config)
        self.assertEqual(list(result["obstacles"]), ["person"])
        self.assertEqual(result["matches_to_unscored_ground_truth"], STEPS)
        self.assertEqual(result["unmatched_track_observations"], 0)

    def test_obstacle_outside_view_is_not_a_miss(self):
        # Behind the robot (x < robot_x) with a 90 degree field: never expected.
        samples = [GroundTruthSample(t, "behind", -2.0, 0.0) for t in times()]
        truth = GroundTruth(samples, max_gap_s=0.25)
        frames = [TrackFrame(t, "base_link", ()) for t in times()]
        behind = score_frames(frames, truth, self.poses, ScoringConfig(fov_deg=90.0))["obstacles"]["behind"]
        self.assertEqual((behind["expected"], behind["recall"]), (0, None))
        in_range = score_frames(frames, truth, self.poses, ScoringConfig(max_range_m=10.0))
        self.assertEqual(in_range["expected"], STEPS)

    def test_frames_without_ground_truth_or_pose_are_counted(self):
        frames = [TrackFrame(-1.0, "base_link", ()), TrackFrame(2.0, "base_link", ())]
        result = score_frames(frames, self.truth, self.poses, ScoringConfig())
        self.assertEqual((result["frames_without_ground_truth"], result["frames_scored"]), (1, 1))
        # Ground truth continues after the last odom -> base_link transform at 4.9 s.
        late_truth = GroundTruth(
            [GroundTruthSample(t, "a", 0.0, 0.0) for t in (5.0, 5.1)], max_gap_s=0.25,
        )
        late_frames = [TrackFrame(5.05, "base_link", ())]
        self.assertEqual(
            score_frames(late_frames, late_truth, self.poses, ScoringConfig())["frames_without_pose"], 1,
        )

    def test_unknown_track_frame_is_rejected(self):
        with self.assertRaises(ValueError):
            score_frames([TrackFrame(1.0, "map", ())], self.truth, self.poses, ScoringConfig())


class GroundTruthParsingTest(unittest.TestCase):
    def test_csv_with_default_and_overridden_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ground_truth.csv"
            path.write_text("stamp,id,x,y,motion\n1.5,box,3.0,0.5,static\n")
            self.assertEqual(
                ground_truth_from_csv(path, parse_field_overrides("")),
                [GroundTruthSample(1.5, "box", 3.0, 0.5)],
            )
            path.write_text("sim_time_s,name,x,y\n2.0,chair,1.0,2.0\n")
            with self.assertRaisesRegex(ValueError, "sim_time_s"):
                ground_truth_from_csv(path, parse_field_overrides(""))
            fields = parse_field_overrides("time=sim_time_s,id=name")
            self.assertEqual(ground_truth_from_csv(path, fields), [GroundTruthSample(2.0, "chair", 1.0, 2.0)])

    def test_json_nested_fields_and_ros_stamp(self):
        text = json.dumps({
            "header": {"stamp": {"sec": 3, "nanosec": 500_000_000}},
            "obstacles": [{"name": "person", "position": {"x": 2.0, "y": -1.0}}],
        })
        fields = parse_field_overrides("time=header.stamp,id=name,x=position.x,y=position.y")
        self.assertEqual(ground_truth_from_json(text, fields), [GroundTruthSample(3.5, "person", 2.0, -1.0)])
        with self.assertRaisesRegex(ValueError, "top-level keys"):
            ground_truth_from_json(text, parse_field_overrides(""))

    def test_unknown_field_key_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_field_overrides("frame=odom")


@unittest.skipUnless(ROS_AVAILABLE, "source ROS 2 and this workspace's install/setup.bash")
class BagRoundTripTest(unittest.TestCase):
    """Write a real bag and score it through the command-line entry point."""

    def write_bag(self, path, frames):
        """Write /tf, ground truth JSON, and one track topic to a sqlite3 bag."""
        import rosbag2_py
        from amr_interfaces.msg import TrackedObstacle, TrackedObstacleArray
        from geometry_msgs.msg import TransformStamped
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("", ""))
        for name, type_name in (("/tf", "tf2_msgs/msg/TFMessage"),
                                ("/ground_truth/obstacles", "std_msgs/msg/String"),
                                ("/perception/lidar/tracks", "amr_interfaces/msg/TrackedObstacleArray")):
            writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format="cdr"))

        def stamp(message, time_s):
            message.sec = int(time_s)
            message.nanosec = round((time_s - int(time_s)) * 1e9)

        for frame in frames:
            receipt_ns = round(frame.stamp_s * 1e9) + 1
            transform = TransformStamped()
            stamp(transform.header.stamp, frame.stamp_s)
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            x, y, yaw = robot_at(frame.stamp_s)
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            writer.write("/tf", serialize_message(TFMessage(transforms=[transform])), receipt_ns)

            truth = String(data=json.dumps({"stamp": frame.stamp_s, "obstacles": [
                {"id": "box", "x": 3.0, "y": 0.5},
                {"id": "person", "x": person_at(frame.stamp_s)[0], "y": person_at(frame.stamp_s)[1]},
            ]}))
            writer.write("/ground_truth/obstacles", serialize_message(truth), receipt_ns)

            array = TrackedObstacleArray()
            stamp(array.header.stamp, frame.stamp_s)
            array.header.frame_id = "base_link"
            for track in frame.tracks:
                obstacle = TrackedObstacle(track_id=track.track_id, is_dynamic=track.is_dynamic)
                obstacle.pose.pose.position.x, obstacle.pose.pose.position.y = track.x, track.y
                array.obstacles.append(obstacle)
            writer.write("/perception/lidar/tracks", serialize_message(array), receipt_ns + 1)
        del writer

    def test_cli_scores_bag_and_flags_absent_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory) / "bag"
            output = Path(directory) / "scores.json"
            self.write_bag(bag, perfect_frames(skip=range(15, 20)))
            self.assertEqual(main([str(bag), "--topics", "/perception/lidar/tracks", "--json", str(output)]), 0)
            result = json.loads(output.read_text())["topics"]["/perception/lidar/tracks"]
            self.assertEqual(result["frames_scored"], STEPS)
            self.assertEqual(result["fragmentations"], 1)
            self.assertLess(result["position_error_m"]["max"], 1e-6)
            self.assertEqual(main([str(bag), "--json", str(output)]), 1)
            self.assertEqual(
                json.loads(output.read_text())["absent_topics"],
                ["/perception/camera/tracks", "/perception/fused_obstacles"],
            )


if __name__ == "__main__":
    unittest.main()
