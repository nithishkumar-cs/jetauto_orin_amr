from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def share_path(package_name: str) -> Path:
    return Path(get_package_share_directory(package_name))


def package_file(package_name: str, *parts: str) -> Path:
    return share_path(package_name).joinpath(*parts)


def package_available(package_name: str) -> bool:
    """True if the package is built and discoverable.

    The stack is being rebuilt package by package, so bringup must tolerate a
    node whose package does not exist yet: it reports the gap and carries on
    rather than throwing from get_package_share_directory().
    """
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError:
        return False
    return True


def project_config(*parts: str) -> Path:
    """A file under the installed copy of the top-level configs/ directory.

    configs/ is installed wholesale as share/robot_bringup/project_configs, so
    with --symlink-install these resolve back to the source tree and edits are
    live without a rebuild.
    """
    return package_file("robot_bringup", "project_configs", *parts)


def read_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}
