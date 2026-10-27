#!/usr/bin/env python3
"""Write reconstructed obstacle centers from the 2026-09-30 Jetson Isaac scene snapshot.

This models the default validation scene's static objects and crossing person.
It is not exported Isaac ground truth and does not model occlusion or walls.
"""

import argparse
import csv
import sys


def crossing_person_y(time_s):
    """Return the default scene's eight-second triangle-wave person Y position."""
    phase = time_s % 8.0
    return -1.2 + 0.6 * phase if phase < 4.0 else 1.2 - 0.6 * (phase - 4.0)


def main():
    """Write reconstructed CSV rows for the requested inclusive simulation-time span."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--start-s", type=float, required=True)
    parser.add_argument("--stop-s", type=float, required=True)
    parser.add_argument("--step-s", type=float, default=0.05)
    args = parser.parse_args()
    if args.step_s <= 0 or args.stop_s < args.start_s:
        parser.error("require positive --step-s and --stop-s >= --start-s")

    writer = csv.writer(sys.stdout)
    writer.writerow(("stamp_sim", "id", "x", "y"))
    sample_count = int((args.stop_s - args.start_s) / args.step_s + 1e-9) + 1
    for index in range(sample_count):
        stamp = args.start_s + index * args.step_s
        for obstacle_id, x, y in (
            ("crossing_person", 2.8, crossing_person_y(stamp)),
            ("box_near", 2.0, -2.2),
            ("box_far", 4.2, 1.7),
            ("chair", 3.5, -0.5),
        ):
            writer.writerow((f"{stamp:.2f}", obstacle_id, x, f"{y:.8f}"))


if __name__ == "__main__":
    main()
