#!/usr/bin/env python3
"""Replay raw RGB-D/scan/TF from a bag through the real perception chain.

Starts detector, depth projection, LiDAR clustering, both trackers and fusion
with the checked-in configs (optionally overriding parameters), plays only the
raw sensor topics including the recorded sim /clock, and records every perception output to a new
bag. Perception topics already in the source bag are never replayed.

Usage: replay_perception_chain.py SOURCE_BAG OUTPUT_BAG [--set NODE.PARAM=VALUE ...]
           [--no-detector] [--rate R]
Requires ROS and the workspace (perception packages) to be sourced.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CFG = REPO / "configs" / "perception"
RAW_TOPICS = [
    "/clock", "/tf", "/tf_static", "/scan",
    "/camera/rgb/image_rect_color", "/camera/aligned_depth_to_rgb/image_raw",
    "/camera/rgb/camera_info",
]
OUT_TOPICS = [
    "/perception/detections_2d", "/perception/detections_3d",
    "/perception/lidar/clusters", "/perception/lidar/tracks",
    "/perception/camera/tracks", "/perception/fused_obstacles",
]


def node_cmds(overrides, with_detector):
    """Return (name, argv) for every perception node, with -p overrides applied."""
    specs = [
        ("perception_lidar_clustering", "perception_lidar_clustering", "lidar_clustering_node", "lidar_clustering.yaml"),
        ("perception_camera_tracker", "perception_tracking", "tracking_node", "tracking.yaml"),
        ("perception_lidar_tracker", "perception_tracking", "tracking_node", "tracking.yaml"),
        ("perception_fusion", "perception_fusion", "fusion_node", "fusion.yaml"),
    ]
    if with_detector:
        specs += [
            ("perception_detector", "perception_detector", "detector_node", "detector.yaml"),
            ("detection_depth_projection", "perception_detection_depth_projection",
             "detection_depth_projection_node", "detection_depth_projection.yaml"),
        ]
    cmds = []
    for name, pkg, exe, yaml_file in specs:
        argv = ["ros2", "run", pkg, exe, "--ros-args", "-r", f"__node:={name}",
                "--params-file", str(CFG / yaml_file), "-p", "use_sim_time:=true"]
        for item in overrides:
            key, value = item.split("=", 1)
            node, param = key.split(".", 1)
            if node == name:
                argv += ["-p", f"{param}:={value}"]
        cmds.append((name, argv))
    return cmds


def main():
    """Run the replay; exits nonzero if the output bag is missing."""
    ap = argparse.ArgumentParser()
    ap.add_argument("source_bag")
    ap.add_argument("output_bag")
    ap.add_argument("--set", action="append", default=[], metavar="NODE.PARAM=VALUE")
    ap.add_argument("--no-detector", action="store_true",
                    help="skip detector/projection; replay recorded detections_3d instead")
    ap.add_argument("--rate", default="1.0")
    args = ap.parse_args()
    if Path(args.output_bag).exists():
        sys.exit(f"refusing to overwrite {args.output_bag}")

    topics = list(RAW_TOPICS)
    if args.no_detector:
        topics.append("/perception/detections_3d")
    procs = []
    try:
        for name, argv in node_cmds(args.set, not args.no_detector):
            procs.append(subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                          start_new_session=True))
        time.sleep(6)  # TensorRT engine load
        rec = subprocess.Popen(["ros2", "bag", "record", "-o", args.output_bag, "--use-sim-time"]
                               + OUT_TOPICS, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               start_new_session=True)
        procs.append(rec)
        time.sleep(3)
        subprocess.run(["ros2", "bag", "play", args.source_bag, "--rate", args.rate,
                        "--topics"] + topics, check=True, stdout=subprocess.DEVNULL)
        time.sleep(3)
    finally:
        for p in reversed(procs):
            os.killpg(p.pid, signal.SIGINT)  # ros2 run does not forward SIGINT
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
    if not Path(args.output_bag).exists():
        sys.exit("no output bag produced")


if __name__ == "__main__":
    main()
