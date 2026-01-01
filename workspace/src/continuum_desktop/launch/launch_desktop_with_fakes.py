import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from continuum.constants import CONTINUUM_NAMESPACE, NODE_ASR_FAKE, NODE_LLM_FAKE, PATH_ASR, PATH_LLM


def generate_base_description():
    base_launch_path = os.path.join(
        get_package_share_directory("continuum_desktop"),
        "launch",
        "launch_desktop.py",
    )

    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(base_launch_path),
    )


def generate_launch_description():
    """Launch all core package nodes, including fake nodes for testing."""

    return LaunchDescription(
        [
            generate_base_description(),
            Node(
                package="continuum_core",
                executable="fake_asr_node",
                name="fake_asr_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_ASR}/{NODE_ASR_FAKE}",
            ),
            # Fake LLM node
            Node(
                package="continuum_core",
                executable="fake_llm_node",
                name="fake_llm_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_LLM}/{NODE_LLM_FAKE}",
            ),
        ]
    )
