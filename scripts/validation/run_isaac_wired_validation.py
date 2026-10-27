#!/usr/bin/env python3
"""Run the Jetson side of the Isaac Ethernet acceptance checks, simulation only.

The Windows Isaac command-drive instance must already be running. This runner
never starts a physical driver or sends a Nav2 goal. It owns and cleans up only
the safety-only bringup process that it starts.
"""

import argparse
import ipaddress
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "tests" / "integration_tests"
SAFETY_LAUNCH = (
    "ros2", "launch", "robot_bringup", "robot_stack.launch.py",
    "instrumentation_mode:=profile", "backend:=isaac", "slam_mode:=off",
    "navigation_mode:=off", "enable_drivers:=false",
    "enable_detector:=false", "enable_geometry:=false",
    "enable_lidar_clustering:=false", "enable_tracking:=false",
    "enable_fusion:=false", "enable_safety:=true",
    "enable_localization:=false", "enable_navigation:=false",
    "enable_teleop:=false", "enable_diagnostics:=false",
    "enable_benchmarks:=false", "enable_calibration_validator:=false",
)


def link_counters(interface):
    root = Path("/sys/class/net") / interface
    names = ("carrier_changes", "carrier_down_count", "carrier_up_count")
    counters = {name: int((root / name).read_text().strip()) for name in names}
    for name in (
        "rx_bytes", "tx_bytes", "rx_errors", "tx_errors", "rx_dropped", "tx_dropped"
    ):
        counters[name] = int((root / "statistics" / name).read_text().strip())
    return counters


def ethernet_link(interface, peer):
    details = json.loads(subprocess.check_output(
        ["ip", "-j", "address", "show", "dev", interface], text=True
    ))
    if len(details) != 1:
        raise RuntimeError(f"expected exactly one interface named {interface}")
    carrier = (Path("/sys/class/net") / interface / "carrier").read_text().strip()
    speed = (Path("/sys/class/net") / interface / "speed").read_text().strip()
    if carrier != "1":
        raise RuntimeError(f"{interface} has no physical carrier")
    if int(speed) < 1000:
        raise RuntimeError(f"{interface} negotiated only {speed} Mb/s, expected 1 Gb/s")
    peer_address = ipaddress.ip_address(peer)
    addresses = [
        entry for entry in details[0].get("addr_info", [])
        if entry.get("family") == "inet" and
        peer_address in ipaddress.ip_network(
            f"{entry['local']}/{entry['prefixlen']}", strict=False
        )
    ]
    if len(addresses) != 1:
        raise RuntimeError(f"{interface} has no unique IPv4 address on the {peer} subnet")
    route = subprocess.check_output(["ip", "route", "get", peer], text=True).strip()
    if f"dev {interface}" not in route or f"src {addresses[0]['local']}" not in route:
        raise RuntimeError(f"peer route does not use {interface}: {route}")
    return {"interface": interface, "local_ip": addresses[0]["local"],
            "peer_ip": peer, "speed_mbps": int(speed), "route": route,
            "counters_before": link_counters(interface)}


def write_fastdds_profile(path, local_ip):
    """Write a DDS profile that reaches Isaac and local ROS safety nodes."""
    path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n'
        '  <transport_descriptors>\n'
        '    <transport_descriptor>\n'
        '      <transport_id>JetautoEthernetOnly</transport_id>\n'
        '      <type>UDPv4</type>\n'
        '      <interfaceWhiteList>\n'
        f'        <address>{local_ip}</address>\n'
        '        <address>127.0.0.1</address>\n'
        '      </interfaceWhiteList>\n'
        '    </transport_descriptor>\n'
        '  </transport_descriptors>\n'
        '  <participant profile_name="jetauto_ethernet_only" is_default_profile="true">\n'
        '    <rtps>\n'
        '      <userTransports>\n'
        '        <transport_id>JetautoEthernetOnly</transport_id>\n'
        '      </userTransports>\n'
        '      <useBuiltinTransports>false</useBuiltinTransports>\n'
        '    </rtps>\n'
        '  </participant>\n'
        '</profiles>\n'
    )


def ros_command(arguments):
    setup = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROOT / 'install' / 'setup.bash'))} && "
    )
    return ["/bin/bash", "-c", setup + "exec " + shlex.join(arguments)]


def run_logged(name, arguments, env, artifacts, timeout):
    print(f"RUN {name}", flush=True)
    try:
        result = subprocess.run(
            ros_command(arguments), cwd=ROOT, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout,
            check=False,
        )
        output = result.stdout
        code = result.returncode
    except subprocess.TimeoutExpired as error:
        output = (error.stdout or b"").decode(errors="replace") if isinstance(error.stdout, bytes) else (error.stdout or "")
        output += "\nTIMEOUT\n"
        code = 124
    (artifacts / f"{name}.log").write_text(output)
    print(output[-3000:], flush=True)
    print(f"RESULT {name}: exit {code}", flush=True)
    return code, output


def wait_for_safety_active(process, env, artifacts):
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("safety-only bringup exited before collision monitor activated")
        result = subprocess.run(
            ros_command(["ros2", "lifecycle", "get", "/collision_monitor"]),
            cwd=ROOT, env=env, text=True, capture_output=True, timeout=8, check=False,
        )
        if result.returncode == 0 and "active [3]" in result.stdout:
            (artifacts / "collision_monitor_lifecycle.log").write_text(result.stdout)
            return
        time.sleep(0.5)
    raise RuntimeError("collision monitor did not reach active [3] within 25 seconds")


def stop_owned_launch(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    parser.add_argument("--interface", default="eno1")
    parser.add_argument("--peer-ip", default="169.254.71.211")
    parser.add_argument("--domain-id", type=int, default=42)
    parser.add_argument("--stream-seconds", type=float, default=60.0)
    parser.add_argument("--artifacts-dir", type=Path)
    args = parser.parse_args()
    if not args.isaac_simulation_only:
        parser.error("explicit simulation-only flag required")
    if args.stream_seconds < 10.0:
        parser.error("--stream-seconds must be at least 10")
    if not (0 <= args.domain_id <= 232):
        parser.error("invalid ROS domain ID")
    ipaddress.ip_address(args.peer_ip)

    artifacts = args.artifacts_dir or Path(tempfile.mkdtemp(prefix="jetauto_wired_validation_"))
    if args.artifacts_dir:
        artifacts.mkdir(parents=True, exist_ok=False)
    report = {"artifacts_dir": str(artifacts), "results": {}}
    safety_process = None
    safety_log = None
    env = None
    exit_code = 1
    try:
        report["link"] = ethernet_link(args.interface, args.peer_ip)
        print(f"PASS 1 Gb/s direct link: {report['link']}", flush=True)
        profile = artifacts / "fastdds_ethernet_only.xml"
        write_fastdds_profile(profile, report["link"]["local_ip"])
        env = os.environ.copy()
        env.update({
            "ROS_DOMAIN_ID": str(args.domain_id),
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
            "ROS_LOCALHOST_ONLY": "0",
            "ROS2CLI_DISABLE_DAEMON": "1",
            "PYTHONUNBUFFERED": "1",
        })

        stream_code, stream_output = run_logged(
            "streams", ["python3", str(TESTS / "probe_isaac_wired_streams.py"),
                        "--duration", str(args.stream_seconds), "--require-camera"],
            env, artifacts, args.stream_seconds + 30,
        )
        report["results"]["streams"] = stream_code
        if stream_code in (0, 2):
            report["streams"] = json.loads(stream_output)
        if stream_code not in (0, 2):
            raise RuntimeError("core Isaac clock/scan/odom/TF stream gate failed")
        if stream_code == 2:
            print("INCOMPLETE: camera topics are missing or unhealthy; continuing motion gates", flush=True)

        direct_code, direct_output = run_logged(
            "direct_command", ["python3", str(TESTS / "probe_isaac_command_drive.py"),
                               "--direct-isaac-simulation-only"],
            env, artifacts, 45,
        )
        report["results"]["direct_command"] = direct_code
        report["direct_command_observations"] = direct_output.strip().splitlines()
        if direct_code != 0:
            raise RuntimeError("direct Isaac command/watchdog gate failed")

        print("RUN safety-only bringup", flush=True)
        safety_log = (artifacts / "safety_bringup.log").open("w")
        safety_process = subprocess.Popen(
            ros_command(list(SAFETY_LAUNCH)), cwd=ROOT, env=env,
            stdout=safety_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        wait_for_safety_active(safety_process, env, artifacts)
        safety_code, safety_output = run_logged(
            "safety_motion", ["python3", str(TESTS / "probe_isaac_safety_motion.py"),
                              "--isaac-simulation-only"],
            env, artifacts, 45,
        )
        report["results"]["safety_motion"] = safety_code
        report["safety_motion_observations"] = safety_output.strip().splitlines()
        if safety_code != 0:
            raise RuntimeError("safety-chain Isaac motion gate failed")

        exit_code = 0 if stream_code == 0 else 2
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
        print(f"FAIL: {error}", flush=True)
    finally:
        if safety_process is not None:
            stop_owned_launch(safety_process)
            report["safety_bringup_exit"] = safety_process.returncode
            if safety_process.returncode != 0:
                report["error"] = "safety-only bringup did not exit cleanly"
                exit_code = 1
        if safety_log is not None:
            safety_log.close()
        if safety_process is not None and env is not None:
            try:
                graph_code, graph_output = run_logged(
                    "post_cleanup_graph",
                    ["ros2", "topic", "info", "/cmd_vel/safety_limited", "--verbose"],
                    env, artifacts, 20,
                )
                if not (
                    graph_code == 0 and "Publisher count: 0" in graph_output and
                    "Subscription count: 1" in graph_output and "CmdVelSub" in graph_output
                ):
                    report["error"] = "final command publisher remained after cleanup"
                    exit_code = 1
            except (OSError, subprocess.SubprocessError) as error:
                report["post_cleanup_graph_error"] = str(error)
                exit_code = 1
        if "link" in report:
            try:
                after = link_counters(args.interface)
                before = report["link"]["counters_before"]
                deltas = {name: after[name] - before[name] for name in after}
                report["link"]["counters_after"] = after
                report["link"]["counter_deltas"] = deltas
                if any(deltas[name] for name in ("carrier_changes", "rx_errors", "tx_errors")):
                    report["error"] = "Ethernet carrier or interface error counters changed during validation"
                    exit_code = 1
            except OSError as error:
                report["link_counter_error"] = str(error)
                exit_code = 1
        report["exit_code"] = exit_code
        (artifacts / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(f"ARTIFACTS {artifacts}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
