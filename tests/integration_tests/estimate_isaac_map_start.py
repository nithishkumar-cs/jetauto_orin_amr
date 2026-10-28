"""Estimate an Isaac start pose by matching stationary LiDAR returns to a saved map."""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates


def load_map(map_yaml):
    """Return the occupied-cell distance field and map geometry in metres."""
    config = yaml.safe_load(Path(map_yaml).read_text())
    if len(config["origin"]) != 3 or abs(config["origin"][2]) > 1e-9:
        raise ValueError("only zero-yaw occupancy maps are supported")
    image = np.asarray(Image.open(Path(map_yaml).parent / config["image"]))
    if image.ndim != 2:
        raise ValueError("expected a grayscale occupancy map")
    occupied = image < 100
    if not occupied.any():
        raise ValueError("the occupancy map has no occupied cells")
    field = distance_transform_edt(~occupied) * float(config["resolution"])
    return field, config


def scan_points(scan_yaml, lidar_xy):
    """Load one ROS CLI LaserScan document and return valid XY endpoints in base_link."""
    message = next(yaml.safe_load_all(Path(scan_yaml).read_text()))
    ranges = np.asarray(message["ranges"], dtype=float)
    angles = message["angle_min"] + np.arange(len(ranges)) * message["angle_increment"]
    valid = (
        np.isfinite(ranges)
        & (ranges >= message["range_min"])
        & (ranges < message["range_max"])
    )
    if not valid.any():
        raise ValueError(f"{scan_yaml} has no valid returns")
    return np.column_stack(
        (ranges[valid] * np.cos(angles[valid]) + lidar_xy[0],
         ranges[valid] * np.sin(angles[valid]) + lidar_xy[1])
    )


def score_pose(field, config, points, x, y, yaw):
    """Score a candidate base pose with capped endpoint distance to occupied cells."""
    cosine, sine = np.cos(yaw), np.sin(yaw)
    px = x + cosine * points[:, 0] - sine * points[:, 1]
    py = y + sine * points[:, 0] + cosine * points[:, 1]
    resolution = float(config["resolution"])
    origin_x, origin_y, _ = config["origin"]
    columns = (px - origin_x) / resolution - 0.5
    rows = field.shape[0] - (py - origin_y) / resolution - 0.5
    distances = map_coordinates(field, [rows, columns], order=1, mode="constant", cval=1.0)
    return float(np.mean(np.minimum(distances, 0.4))), float(np.mean(distances < 0.1))


def search_grid(field, config, points, xs, ys, yaws):
    """Return the lowest-cost pose from a deterministic bounded grid."""
    best = None
    for yaw in yaws:
        for x in xs:
            for y in ys:
                cost, nearby_fraction = score_pose(field, config, points, x, y, yaw)
                candidate = (cost, -nearby_fraction, float(x), float(y), float(yaw))
                if best is None or candidate < best:
                    best = candidate
    return best


def estimate_pose(field, config, points, center_xy=(0.0, 0.0), radius=0.5, center_yaw=0.0):
    """Search start translation and small heading error at coarse and fine resolutions."""
    coarse = search_grid(
        field, config, points,
        np.arange(center_xy[0] - radius, center_xy[0] + radius + 0.0125, 0.025),
        np.arange(center_xy[1] - radius, center_xy[1] + radius + 0.0125, 0.025),
        np.arange(center_yaw - 0.12, center_yaw + 0.121, 0.02),
    )
    _, _, x0, y0, yaw0 = coarse
    return search_grid(
        field, config, points,
        np.arange(x0 - 0.025, x0 + 0.026, 0.005),
        np.arange(y0 - 0.025, y0 + 0.026, 0.005),
        np.arange(yaw0 - 0.02, yaw0 + 0.021, 0.005),
    )


def main():
    """Print the independent start estimate and fit for each stationary scan."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--scan", type=Path, action="append", required=True)
    parser.add_argument("--lidar-x", type=float, required=True)
    parser.add_argument("--lidar-y", type=float, required=True)
    parser.add_argument("--center-x", type=float, default=0.0)
    parser.add_argument("--center-y", type=float, default=0.0)
    parser.add_argument("--center-yaw", type=float, default=0.0)
    parser.add_argument("--radius", type=float, default=0.5)
    args = parser.parse_args()
    if args.radius <= 0 or not np.isfinite(args.radius):
        parser.error("--radius must be positive and finite")
    field, config = load_map(args.map_yaml)
    scans = [scan_points(path, (args.lidar_x, args.lidar_y)) for path in args.scan]
    all_points = np.concatenate(scans)
    cost, negative_nearby, x, y, yaw = estimate_pose(
        field, config, all_points, (args.center_x, args.center_y), args.radius,
        args.center_yaw,
    )
    print(json.dumps({
        "map_start_xy_yaw": [x, y, yaw],
        "capped_mean_endpoint_distance_m": cost,
        "fraction_within_0_10_m": -negative_nearby,
        "scans": [
            {"path": str(path), "valid_returns": len(points),
             "fit": score_pose(field, config, points, x, y, yaw)}
            for path, points in zip(args.scan, scans)
        ],
    }, indent=2))


if __name__ == "__main__":
    main()
