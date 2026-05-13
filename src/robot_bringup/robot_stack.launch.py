import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction

LAUNCH_ROOT = Path(__file__).resolve().parent
if str(LAUNCH_ROOT) not in sys.path:
    sys.path.insert(0, str(LAUNCH_ROOT))

from launch_lib.actions import build_actions
from launch_lib.arguments import declared_arguments
from launch_lib.config import resolve_config


def build_launch(context):
    return build_actions(resolve_config(context))


def generate_launch_description():
    return LaunchDescription(declared_arguments() + [OpaqueFunction(function=build_launch)])
