import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_base_description():
    base_launch_path = os.path.join(
        get_package_share_directory("continuum_desktop"),
        "launch",
        "launch_desktop.py",
    )

    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(base_launch_path),
    )


def generate_reachy_description():
    reachy_launch_path = os.path.join(
        get_package_share_directory("continuum_desktop"),
        "launch",
        "launch_reachy.py",
    )

    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(reachy_launch_path),
    )


def generate_websocket_description():
    bridge_launch_path = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )

    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(bridge_launch_path),
    )


def generate_launch_description():
    """Launch all package nodes for the Docker environment."""

    return LaunchDescription(
        [
            generate_base_description(),
            generate_reachy_description(),
            generate_websocket_description(),
        ]
    )
