#!/usr/bin/env python3
"""Score recorded tracker output against Isaac obstacle ground truth.

Each `amr_interfaces/TrackedObstacleArray` (normally in `base_link`) is moved
into the ground-truth `odom` frame with the bag's `odom -> base_link` TF at the
array stamp, then matched to ground truth interpolated to that stamp. Matching
follows CLEAR-MOT: last frame's correspondences persist while inside the gate,
and the rest use optimal assignment. Ground truth comes from the bag's
`/ground_truth/obstacles` JSON topic or from the Isaac `ground_truth.csv`.
"""

import argparse
import bisect
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment


DEFAULT_TRACK_TOPICS = (
    "/perception/camera/tracks",
    "/perception/lidar/tracks",
    "/perception/fused_obstacles",
)
GROUND_TRUTH_TOPIC = "/ground_truth/obstacles"
TRACK_TYPE = "amr_interfaces/msg/TrackedObstacleArray"
GROUND_TRUTH_TYPE = "std_msgs/msg/String"

# Isaac's ground-truth schema (Windows Isaac repo: the `ground_truth` entry of
# config/topic_contract.yaml and GroundTruthRecorder._CSV_HEADER). Both formats
# stamp samples with `stamp_sim`, float simulation seconds that probably trail
# `/clock` by ~0.067 s, uncorrected here (rosbag/README.md). The CSV flattens the
# position into x/y columns, while each JSON `obstacles` item nests it as
# `"position": [x, y, z]`. `--gt-fields` overrides them without a code change.
# CSV uses time/id/x/y as column names; JSON reads time and list at the message
# top level and id/x/y inside each list item. Dotted names address nested
# objects and list indices.
CSV_GT_FIELDS = {"time": "stamp_sim", "list": "obstacles", "id": "id", "x": "x", "y": "y"}
JSON_GT_FIELDS = {**CSV_GT_FIELDS, "x": "position.0", "y": "position.1"}

MOTION_KEYS = ("moving_dynamic", "moving_static", "static_dynamic", "static_static")


@dataclass(frozen=True)
class GroundTruthSample:
    """One obstacle position in the ground-truth frame at one simulation time."""

    stamp_s: float
    obstacle_id: str
    x: float
    y: float


@dataclass(frozen=True)
class ObstacleState:
    """Interpolated ground-truth position and speed of one obstacle."""

    x: float
    y: float
    speed_mps: float


@dataclass(frozen=True)
class Pose2D:
    """Planar pose of the base frame in the ground-truth frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrackObservation:
    """One published track in its array's frame."""

    track_id: int
    x: float
    y: float
    is_dynamic: bool


@dataclass(frozen=True)
class TrackFrame:
    """One published TrackedObstacleArray reduced to the fields scoring needs."""

    stamp_s: float
    frame_id: str
    tracks: tuple


@dataclass(frozen=True)
class ScoringConfig:
    """Matching and expectation settings shared by every scored topic.

    `scored_ids` limits which ground-truth obstacles count (None = all); other
    obstacles still absorb nearby tracks so they are not counted as unmatched.
    A scored obstacle outside `max_range_m` or the base-frame `fov_deg` is not
    expected, so missing it is neither a miss nor a fragmentation break.
    """

    gate_m: float = 0.75
    moving_speed_mps: float = 0.20
    scored_ids: frozenset = None
    max_range_m: float = math.inf
    fov_deg: float = 360.0
    odom_frame: str = "odom"
    base_frame: str = "base_link"


def wrap_angle(angle):
    """Return `angle` wrapped to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    """Return the Z-axis yaw of a unit quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _sorted_unique(rows):
    """Sort (time, ...) rows by time, keeping the last row for a repeated time."""
    by_time = {}
    for row in rows:
        by_time[row[0]] = row
    return [by_time[time] for time in sorted(by_time)]


def _sorted_consistent(rows, obstacle_id):
    """Sort (time, x, y) rows by time, collapsing repeats that agree within 1 um.

    Raises ValueError when one time carries two positions: the input then mixes
    runs (for example a ground_truth.csv appended across Isaac launches), and
    keeping either row would silently score against the wrong trajectory.
    """
    by_time = {}
    for row in rows:
        kept = by_time.setdefault(row[0], row)
        if math.hypot(kept[1] - row[1], kept[2] - row[2]) > 1e-6:
            raise ValueError(
                f"ground-truth obstacle {obstacle_id!r} has two positions at t={row[0]}: "
                f"({kept[1]}, {kept[2]}) and ({row[1]}, {row[2]}); the input mixes more than one run"
            )
    return [by_time[time] for time in sorted(by_time)]


def _bracket(times, stamp_s, max_gap_s):
    """Return (index, fraction) interpolating `stamp_s` between samples.

    Returns None outside the sampled span or across a gap wider than
    `max_gap_s`; `times` must be sorted, unique, and hold at least two entries.
    """
    if stamp_s < times[0] or stamp_s > times[-1]:
        return None
    index = min(bisect.bisect_right(times, stamp_s) - 1, len(times) - 2)
    start, end = times[index], times[index + 1]
    if end - start > max_gap_s:
        return None
    return index, (stamp_s - start) / (end - start)


class GroundTruth:
    """Per-obstacle trajectories with bounded linear interpolation."""

    def __init__(self, samples, max_gap_s):
        """Group samples by obstacle.

        Raises ValueError when there are no samples, an obstacle has fewer
        than two distinct times (its speed would be undefined), or one time
        carries two different positions for an obstacle.
        """
        grouped = defaultdict(list)
        for sample in samples:
            grouped[sample.obstacle_id].append((sample.stamp_s, sample.x, sample.y))
        if not grouped:
            raise ValueError("ground truth contains no samples")
        self.max_gap_s = max_gap_s
        self._tracks = {}
        for obstacle_id, rows in grouped.items():
            rows = _sorted_consistent(rows, obstacle_id)
            if len(rows) < 2:
                raise ValueError(f"ground-truth obstacle {obstacle_id!r} has fewer than two samples")
            self._tracks[obstacle_id] = tuple(list(column) for column in zip(*rows))

    @property
    def ids(self):
        """Return the sorted obstacle IDs."""
        return sorted(self._tracks)

    @property
    def span_s(self):
        """Return the (first, last) sample time over all obstacles."""
        return (
            min(times[0] for times, _, _ in self._tracks.values()),
            max(times[-1] for times, _, _ in self._tracks.values()),
        )

    def states_at(self, stamp_s):
        """Return {obstacle_id: ObstacleState} for obstacles sampled around `stamp_s`."""
        states = {}
        for obstacle_id, (times, xs, ys) in self._tracks.items():
            bracket = _bracket(times, stamp_s, self.max_gap_s)
            if bracket is None:
                continue
            index, fraction = bracket
            dx = xs[index + 1] - xs[index]
            dy = ys[index + 1] - ys[index]
            states[obstacle_id] = ObstacleState(
                xs[index] + fraction * dx,
                ys[index] + fraction * dy,
                math.hypot(dx, dy) / (times[index + 1] - times[index]),
            )
        return states


class PoseTimeline:
    """Base poses from TF with bounded linear (shortest-arc yaw) interpolation."""

    def __init__(self, samples, max_gap_s):
        """Store (time, x, y, yaw) samples; raises ValueError with fewer than two."""
        rows = _sorted_unique(samples)
        if len(rows) < 2:
            raise ValueError("fewer than two odom -> base transforms were found")
        self.times, self.xs, self.ys, self.yaws = (list(column) for column in zip(*rows))
        self.max_gap_s = max_gap_s

    def pose_at(self, stamp_s):
        """Return the interpolated Pose2D at `stamp_s`, or None when not covered."""
        bracket = _bracket(self.times, stamp_s, self.max_gap_s)
        if bracket is None:
            return None
        index, fraction = bracket
        return Pose2D(
            self.xs[index] + fraction * (self.xs[index + 1] - self.xs[index]),
            self.ys[index] + fraction * (self.ys[index + 1] - self.ys[index]),
            wrap_angle(self.yaws[index] + fraction * wrap_angle(self.yaws[index + 1] - self.yaws[index])),
        )


def associate(truth_positions, track_positions, track_ids, previous, gate_m):
    """Match ground truth to tracks one-to-one within `gate_m`.

    `truth_positions` maps obstacle ID to (x, y); `track_positions` is an
    (N, 2) array aligned with `track_ids`. A correspondence from `previous`
    (obstacle ID -> track ID) is kept while the track exists and stays inside
    the gate; remaining pairs minimise total distance with the most matches.
    Returns {obstacle_id: track index}.
    """
    matches = {}
    used = set()
    index_by_id = {}
    for index, track_id in enumerate(track_ids):
        index_by_id.setdefault(track_id, index)
    for obstacle_id in sorted(truth_positions):
        index = index_by_id.get(previous.get(obstacle_id))
        if index is None or index in used:
            continue
        x, y = truth_positions[obstacle_id]
        if math.hypot(track_positions[index, 0] - x, track_positions[index, 1] - y) <= gate_m:
            matches[obstacle_id] = index
            used.add(index)

    free_truth = [obstacle_id for obstacle_id in sorted(truth_positions) if obstacle_id not in matches]
    free_tracks = [index for index in range(len(track_ids)) if index not in used]
    if not free_truth or not free_tracks:
        return matches
    truth = np.array([truth_positions[obstacle_id] for obstacle_id in free_truth])
    tracks = track_positions[free_tracks]
    distances = np.hypot(truth[:, None, 0] - tracks[None, :, 0], truth[:, None, 1] - tracks[None, :, 1])
    # Any out-of-gate pair costs more than every in-gate pair combined, so the
    # solver first maximises the number of gated matches, then minimises distance.
    penalty = gate_m * (min(distances.shape) + 1) + 1.0
    rows, columns = linear_sum_assignment(np.where(distances <= gate_m, distances, penalty))
    for row, column in zip(rows, columns):
        if distances[row, column] <= gate_m:
            matches[free_truth[row]] = free_tracks[column]
    return matches


def is_expected(state, pose, config):
    """Return whether a scored obstacle lies inside the configured range and FOV."""
    dx, dy = state.x - pose.x, state.y - pose.y
    if math.hypot(dx, dy) > config.max_range_m:
        return False
    if config.fov_deg >= 360.0:
        return True
    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
    bearing = math.atan2(-sin_yaw * dx + cos_yaw * dy, cos_yaw * dx + sin_yaw * dy)
    return abs(bearing) <= math.radians(config.fov_deg) / 2.0


def summarize_errors(errors):
    """Return count, mean, RMSE, median, p95, and max of position errors."""
    if not errors:
        return {"count": 0}
    values = np.asarray(errors)
    return {
        "count": int(values.size),
        "mean": float(values.mean()),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95.0)),
        "max": float(values.max()),
    }


def summarize_motion(counts):
    """Return the moving/static confusion counts with accuracy (None if empty)."""
    total = sum(counts[key] for key in MOTION_KEYS)
    correct = counts["moving_dynamic"] + counts["static_static"]
    return {**{key: counts[key] for key in MOTION_KEYS}, "accuracy": correct / total if total else None}


@dataclass
class _ObstacleScore:
    """Running per-obstacle counters for one topic."""

    expected: int = 0
    expected_matched: int = 0
    matches: int = 0
    id_switches: int = 0
    fragmentations: int = 0
    track_ids: set = field(default_factory=set)
    errors: list = field(default_factory=list)
    motion: Counter = field(default_factory=Counter)
    last_track_id: int = None
    lost: bool = False

    def report(self):
        """Return the JSON-serialisable summary."""
        return {
            "expected": self.expected,
            "expected_matched": self.expected_matched,
            "recall": self.expected_matched / self.expected if self.expected else None,
            "matches": self.matches,
            "id_switches": self.id_switches,
            "fragmentations": self.fragmentations,
            "distinct_track_ids": len(self.track_ids),
            "position_error_m": summarize_errors(self.errors),
            "motion": summarize_motion(self.motion),
        }


def score_frames(frames, ground_truth, poses, config):
    """Score one topic's TrackFrames against ground truth.

    A match to a scored obstacle counts its position error, motion label, and
    an ID switch when the track ID differs from the obstacle's previous match.
    A fragmentation is a scored obstacle matched again after an expected but
    unmatched frame. Raises ValueError for an array frame other than the
    configured odom or base frame.
    """
    scores = {obstacle_id: _ObstacleScore() for obstacle_id in ground_truth.ids
              if config.scored_ids is None or obstacle_id in config.scored_ids}
    counts = Counter()
    previous = {}
    for frame in sorted(frames, key=lambda item: item.stamp_s):
        counts["frames"] += 1
        states = ground_truth.states_at(frame.stamp_s)
        if not states:
            counts["frames_without_ground_truth"] += 1
            continue
        pose = poses.pose_at(frame.stamp_s)
        if pose is None:
            counts["frames_without_pose"] += 1
            continue
        counts["frames_scored"] += 1
        counts["track_observations"] += len(frame.tracks)

        positions = np.array([(track.x, track.y) for track in frame.tracks], dtype=float).reshape(-1, 2)
        if frame.frame_id == config.base_frame:
            cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
            positions = positions @ np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]]) + (pose.x, pose.y)
        elif frame.frame_id != config.odom_frame:
            raise ValueError(
                f"track frame {frame.frame_id!r} is neither {config.base_frame!r} nor {config.odom_frame!r}"
            )

        track_ids = [track.track_id for track in frame.tracks]
        matches = associate(
            {obstacle_id: (state.x, state.y) for obstacle_id, state in states.items()},
            positions, track_ids, previous, config.gate_m,
        )
        previous = {obstacle_id: track_ids[index] for obstacle_id, index in matches.items()}
        counts["unmatched_track_observations"] += len(frame.tracks) - len(matches)

        for obstacle_id, state in states.items():
            score = scores.get(obstacle_id)
            index = matches.get(obstacle_id)
            if score is None:
                counts["matches_to_unscored_ground_truth"] += index is not None
                continue
            expected = is_expected(state, pose, config)
            score.expected += expected
            if index is None:
                score.lost = score.lost or (expected and score.last_track_id is not None)
                continue
            track = frame.tracks[index]
            score.expected_matched += expected
            score.matches += 1
            score.track_ids.add(track.track_id)
            score.errors.append(math.hypot(positions[index, 0] - state.x, positions[index, 1] - state.y))
            moving = "moving" if state.speed_mps >= config.moving_speed_mps else "static"
            score.motion[f"{moving}_{'dynamic' if track.is_dynamic else 'static'}"] += 1
            if score.last_track_id is not None and track.track_id != score.last_track_id:
                score.id_switches += 1
            if score.lost:
                score.fragmentations += 1
                score.lost = False
            score.last_track_id = track.track_id

    errors = [error for score in scores.values() for error in score.errors]
    motion = sum((score.motion for score in scores.values()), Counter())
    expected = sum(score.expected for score in scores.values())
    expected_matched = sum(score.expected_matched for score in scores.values())
    return {
        **{key: counts[key] for key in (
            "frames", "frames_scored", "frames_without_ground_truth", "frames_without_pose",
            "track_observations", "unmatched_track_observations", "matches_to_unscored_ground_truth",
        )},
        "expected": expected,
        "expected_matched": expected_matched,
        "recall": expected_matched / expected if expected else None,
        "matches": sum(score.matches for score in scores.values()),
        "id_switches": sum(score.id_switches for score in scores.values()),
        "fragmentations": sum(score.fragmentations for score in scores.values()),
        "position_error_m": summarize_errors(errors),
        "motion": summarize_motion(motion),
        "obstacles": {obstacle_id: score.report() for obstacle_id, score in sorted(scores.items())},
    }


def parse_field_overrides(text, defaults):
    """Return a copy of `defaults` (CSV_GT_FIELDS or JSON_GT_FIELDS) updated by `key=name,...`.

    Raises ValueError on an unknown key or an entry without a name.
    """
    fields = dict(defaults)
    for item in filter(None, (part.strip() for part in text.split(","))):
        key, separator, name = item.partition("=")
        if not separator or key not in fields or not name:
            raise ValueError(f"invalid --gt-fields entry {item!r}; keys: {', '.join(fields)}")
        fields[key] = name
    return fields


def _lookup(record, dotted):
    """Return the value at a dotted path of dict keys and list indices; raises KeyError when absent."""
    value = record
    for part in dotted.split("."):
        if isinstance(value, dict) and part in value:
            value = value[part]
        elif isinstance(value, list) and part.isdigit() and int(part) < len(value):
            value = value[int(part)]
        else:
            raise KeyError(dotted)
    return value


def _seconds(value):
    """Convert a number, numeric string, or {sec, nanosec} stamp to float seconds."""
    if isinstance(value, dict) and {"sec", "nanosec"} <= value.keys():
        return value["sec"] + value["nanosec"] * 1e-9
    return float(value)


def ground_truth_from_csv(path, fields):
    """Read GroundTruthSamples from a ground_truth.csv file.

    Raises ValueError naming the missing and available columns when the header
    lacks a configured time/id/x/y column.
    """
    samples = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = [fields[key] for key in ("time", "id", "x", "y")]
        missing = [column for column in columns if column not in (reader.fieldnames or ())]
        if missing:
            raise ValueError(
                f"{path}: missing ground-truth column(s) {missing}; header has {reader.fieldnames}; "
                "map them with --gt-fields"
            )
        for row in reader:
            samples.append(GroundTruthSample(
                _seconds(row[fields["time"]]), row[fields["id"]], float(row[fields["x"]]), float(row[fields["y"]]),
            ))
    return samples


def ground_truth_from_json(text, fields):
    """Parse one /ground_truth/obstacles message into GroundTruthSamples.

    Raises ValueError listing the keys present when a configured field is absent.
    """
    message = json.loads(text)
    try:
        stamp_s = _seconds(_lookup(message, fields["time"]))
        return [
            GroundTruthSample(
                stamp_s, str(_lookup(item, fields["id"])),
                float(_lookup(item, fields["x"])), float(_lookup(item, fields["y"])),
            )
            for item in _lookup(message, fields["list"])
        ]
    except (KeyError, TypeError) as error:
        items = message.get(fields["list"]) if isinstance(message, dict) else None
        first_item = items[0] if isinstance(items, list) and items else None
        raise ValueError(
            f"ground-truth JSON lacks field {error}; top-level keys {_describe_keys(message)}, "
            f"first item keys {_describe_keys(first_item)}; map them with --gt-fields"
        ) from error


def _describe_keys(value):
    """Return sorted dict keys, or the type name of a non-dict, for error messages."""
    return sorted(value) if isinstance(value, dict) else type(value).__name__


def read_bag(bag_path, track_topics, odom_frame, base_frame, gt_fields, read_ground_truth):
    """Read track frames, odom -> base poses, and optionally ground truth from a bag.

    Returns ({topic: [TrackFrame]}, [(t, x, y, yaw)], [GroundTruthSample] or None,
    sorted absent track topics). Raises ValueError when a topic has an unexpected
    type or ground truth was requested but is absent.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ModuleNotFoundError as error:
        raise SystemExit(
            "ROS 2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and this workspace's install/setup.bash before running this command."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    present = [topic for topic in track_topics if topic in types]
    for topic in present:
        if types[topic] != TRACK_TYPE:
            raise ValueError(f"{topic} has type {types[topic]}, expected {TRACK_TYPE}")
    if "/tf" not in types:
        raise ValueError(f"{bag_path} has no /tf topic")
    wanted = present + ["/tf"]
    if read_ground_truth:
        if types.get(GROUND_TRUTH_TOPIC) != GROUND_TRUTH_TYPE:
            raise ValueError(
                f"{bag_path} has no {GROUND_TRUTH_TOPIC} ({GROUND_TRUTH_TYPE}); pass --ground-truth-csv"
            )
        wanted.append(GROUND_TRUTH_TOPIC)
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    message_types = {topic: get_message(types[topic]) for topic in wanted}

    frames = {topic: [] for topic in present}
    poses = []
    ground_truth = [] if read_ground_truth else None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        if topic == "/tf":
            for transform in message.transforms:
                if (transform.header.frame_id.lstrip("/") == odom_frame
                        and transform.child_frame_id.lstrip("/") == base_frame):
                    stamp = transform.header.stamp
                    translation, rotation = transform.transform.translation, transform.transform.rotation
                    poses.append((
                        stamp.sec + stamp.nanosec * 1e-9, translation.x, translation.y,
                        yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                    ))
        elif topic == GROUND_TRUTH_TOPIC:
            ground_truth.extend(ground_truth_from_json(message.data, gt_fields))
        else:
            stamp = message.header.stamp
            frames[topic].append(TrackFrame(
                stamp.sec + stamp.nanosec * 1e-9,
                message.header.frame_id.lstrip("/"),
                tuple(
                    TrackObservation(
                        obstacle.track_id, obstacle.pose.pose.position.x,
                        obstacle.pose.pose.position.y, obstacle.is_dynamic,
                    )
                    for obstacle in message.obstacles
                ),
            ))
    return frames, poses, ground_truth, sorted(set(track_topics) - set(present))


def format_value(value, digits=3):
    """Format a number for the text report; None and NaN print as '-'."""
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "-"
    return f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def print_report(results, absent):
    """Print a compact per-topic and per-obstacle text report."""
    for topic in absent:
        print(f"{topic}: ABSENT from bag")
    for topic, result in results.items():
        error = result["position_error_m"]
        motion = result["motion"]
        print(
            f"\n{topic}: frames {result['frames']} scored {result['frames_scored']} "
            f"(no GT {result['frames_without_ground_truth']}, no TF {result['frames_without_pose']})"
        )
        print(
            f"  recall {format_value(result['recall'])} ({result['expected_matched']}/{result['expected']})  "
            f"matches {result['matches']}  ID switches {result['id_switches']}  "
            f"fragmentations {result['fragmentations']}  unmatched tracks "
            f"{result['unmatched_track_observations']}/{result['track_observations']}  "
            f"unscored-GT matches {result['matches_to_unscored_ground_truth']}"
        )
        print(
            f"  error m: median {format_value(error.get('median'))} p95 {format_value(error.get('p95'))} "
            f"max {format_value(error.get('max'))} rmse {format_value(error.get('rmse'))}  "
            f"motion accuracy {format_value(motion['accuracy'])} "
            f"(moving->dyn {motion['moving_dynamic']}, moving->static {motion['moving_static']}, "
            f"static->dyn {motion['static_dynamic']}, static->static {motion['static_static']})"
        )
        print(f"  {'obstacle':<20} {'recall':>7} {'match':>6} {'idsw':>5} {'frag':>5} {'ids':>4} "
              f"{'med_m':>6} {'p95_m':>6} {'motion':>7}")
        for obstacle_id, item in result["obstacles"].items():
            print(
                f"  {obstacle_id:<20} {format_value(item['recall']):>7} {item['matches']:>6} "
                f"{item['id_switches']:>5} {item['fragmentations']:>5} {item['distinct_track_ids']:>4} "
                f"{format_value(item['position_error_m'].get('median')):>6} "
                f"{format_value(item['position_error_m'].get('p95')):>6} "
                f"{format_value(item['motion']['accuracy']):>7}"
            )


def parse_arguments(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path, help="bag directory recorded on the Jetson")
    parser.add_argument("--ground-truth-csv", type=Path,
                        help=f"Isaac ground_truth.csv; default reads {GROUND_TRUTH_TOPIC} from the bag")
    parser.add_argument("--gt-fields", default="",
                        help="override ground-truth names, e.g. time=sim_time_s,id=name "
                             f"(keys: {', '.join(CSV_GT_FIELDS)}; time defaults to stamp_sim, x/y to CSV "
                             "columns x,y or JSON position.0,position.1; JSON names may be dotted paths)")
    parser.add_argument("--topics", nargs="+", default=list(DEFAULT_TRACK_TOPICS))
    parser.add_argument("--ids", help="comma-separated ground-truth IDs to score (default: all)")
    parser.add_argument("--gate-m", type=float, default=ScoringConfig.gate_m)
    parser.add_argument("--moving-speed-mps", type=float, default=ScoringConfig.moving_speed_mps,
                        help="ground-truth speed at or above which an obstacle counts as moving")
    parser.add_argument("--max-range-m", type=float, default=math.inf,
                        help="obstacles farther from base_link are not expected")
    parser.add_argument("--fov-deg", type=float, default=360.0,
                        help="horizontal field centred on base_link +x outside which obstacles are not expected")
    parser.add_argument("--max-gt-gap-s", type=float, default=0.25)
    parser.add_argument("--max-tf-gap-s", type=float, default=0.2)
    parser.add_argument("--odom-frame", default=ScoringConfig.odom_frame)
    parser.add_argument("--base-frame", default=ScoringConfig.base_frame)
    parser.add_argument("--json", type=Path, help="also write the full results as JSON")
    return parser.parse_args(argv)


def main(argv=None):
    """Score every requested topic; exit 1 if a topic is absent or has no scored frame."""
    args = parse_arguments(argv)
    gt_fields = parse_field_overrides(
        args.gt_fields, JSON_GT_FIELDS if args.ground_truth_csv is None else CSV_GT_FIELDS,
    )
    frames, pose_samples, gt_samples, absent = read_bag(
        args.bag, args.topics, args.odom_frame, args.base_frame, gt_fields, args.ground_truth_csv is None,
    )
    if args.ground_truth_csv is not None:
        gt_samples = ground_truth_from_csv(args.ground_truth_csv, gt_fields)
    ground_truth = GroundTruth(gt_samples, args.max_gt_gap_s)
    poses = PoseTimeline(pose_samples, args.max_tf_gap_s)
    scored_ids = frozenset(filter(None, args.ids.split(","))) if args.ids else None
    if scored_ids is not None and not scored_ids <= set(ground_truth.ids):
        raise SystemExit(f"--ids {sorted(scored_ids - set(ground_truth.ids))} not in ground truth {ground_truth.ids}")
    config = ScoringConfig(
        gate_m=args.gate_m, moving_speed_mps=args.moving_speed_mps, scored_ids=scored_ids,
        max_range_m=args.max_range_m, fov_deg=args.fov_deg,
        odom_frame=args.odom_frame, base_frame=args.base_frame,
    )
    results = {topic: score_frames(topic_frames, ground_truth, poses, config)
               for topic, topic_frames in frames.items()}

    first, last = ground_truth.span_s
    print(f"ground truth: {len(ground_truth.ids)} obstacles {ground_truth.ids}, t {first:.3f}..{last:.3f} s")
    print_report(results, absent)
    if args.json:
        args.json.write_text(json.dumps(
            {"ground_truth_span_s": [first, last], "absent_topics": absent, "topics": results}, indent=2,
        ) + "\n")
    unscored = [topic for topic, result in results.items() if result["frames_scored"] == 0]
    if unscored:
        print(f"\nno frame scored for {unscored}: check the ground-truth time base against track stamps",
              file=sys.stderr)
    return 1 if absent or unscored else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ValueError as error:
        sys.exit(f"error: {error}")
