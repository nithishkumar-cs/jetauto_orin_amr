import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction

# launch_lib/ is installed as a sibling of launch/, so the importable root is
# this file's PARENT directory (share/robot_bringup), not its own (…/launch).
PACKAGE_SHARE = Path(__file__).resolve().parent.parent
if str(PACKAGE_SHARE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SHARE))

from launch_lib.actions import build_actions
from launch_lib.arguments import declared_arguments
from launch_lib.config import resolve_config


def build_launch(context):
    return build_actions(resolve_config(context))


def generate_launch_description():
    return LaunchDescription(declared_arguments() + [OpaqueFunction(function=build_launch)])
