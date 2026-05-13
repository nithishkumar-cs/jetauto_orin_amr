from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def share_path(package_name: str) -> Path:
    return Path(get_package_share_directory(package_name))


def package_file(package_name: str, *parts: str) -> Path:
    return share_path(package_name).joinpath(*parts)


def read_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def detector_config_for_backend(backend: str) -> Path:
    config_dir = package_file("perception_inference", "config")
    config_map = {
        "debug": config_dir / "debug_detector.yaml",
        "yolo": config_dir / "yolo_detector.yaml",
        "rf_detr": config_dir / "rf_detr_detector.yaml",
    }
    if backend not in config_map:
        raise RuntimeError(f"Unsupported detector backend '{backend}'.")
    return config_map[backend]
