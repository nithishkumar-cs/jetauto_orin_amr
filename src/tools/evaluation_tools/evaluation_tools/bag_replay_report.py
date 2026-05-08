from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path


def run(command: list[str]) -> str:
    return subprocess.check_output(command, text=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a small markdown report for a ROS bag.")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=Path("bag_report.md"))
    parser.add_argument("--notes", default="")
    args = parser.parse_args()

    info_text = run(["ros2", "bag", "info", str(args.bag)])

    report = [
        "# Bag Replay Report",
        "",
        f"- Bag: `{args.bag}`",
        f"- Notes: {args.notes or 'none'}",
        "",
        "## ros2 bag info",
        "",
        "```text",
        info_text.strip(),
        "```",
        "",
        "## Replay Checklist",
        "",
        "- Confirm `/diagnostics/system/health` stays OK or has explained warnings.",
        "- Record detector FPS and end-to-end obstacle latency.",
        "- Save RF-DETR versus baseline detector metrics in `docs/benchmarks/benchmarking.md`.",
    ]

    args.output.write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "bag": str(args.bag)}))


if __name__ == "__main__":
    main()
