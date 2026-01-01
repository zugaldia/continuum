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


def generate_foxglove_description():
    bridge_launch_path = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )

    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(bridge_launch_path),
    )


def generate_launch_description():
    """Launch all core package nodes for the desktop application."""

    return LaunchDescription(
        [
            generate_base_description(),
            generate_foxglove_description(),
        ]
    )
