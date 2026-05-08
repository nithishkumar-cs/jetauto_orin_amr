#!/usr/bin/env python3
"""Summarize CSV latency exports produced during bag replay experiments."""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--column", default="latency_ms")
    args = parser.parse_args()

    values: list[float] = []
    with args.csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get(args.column):
                values.append(float(row[args.column]))

    if not values:
        raise SystemExit(f"No numeric values found in column {args.column!r}")

    values.sort()
    p95 = values[int(0.95 * (len(values) - 1))]
    print(f"count={len(values)}")
    print(f"mean_ms={statistics.fmean(values):.3f}")
    print(f"median_ms={statistics.median(values):.3f}")
    print(f"p95_ms={p95:.3f}")
    print(f"max_ms={max(values):.3f}")


if __name__ == "__main__":
    main()

