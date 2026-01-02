from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from continuum.constants import (
    CONTINUUM_NAMESPACE,
    NODE_AGENT_PYDANTIC,
    PATH_AGENT,
)


def get_config_file_argument():
    """Create the config file launch argument, and return it with the parameters list as a tuple."""

    # The default is ./workspace/src/continuum_desktop/config/continuum.yaml
    default_config_path = PathJoinSubstitution([FindPackageShare("continuum_desktop"), "config", "continuum.yaml"])

    # Declare launch argument for custom config file path
    config_file_arg = DeclareLaunchArgument(
        "continuum_config",
        default_value=default_config_path,
        description="Path to the Continuum configuration YAML file",
    )

    # Get the config file path from launch configuration
    config_file = LaunchConfiguration("continuum_config")
    parameters = [config_file]
    return config_file_arg, parameters


def generate_launch_description():
    """Launch agent nodes."""

    config_file_arg, parameters = get_config_file_argument()
    return LaunchDescription(
        [
            # Config launch argument
            config_file_arg,
            # Agent nodes
            Node(
                package="continuum_core",
                executable="pydantic_agent_node",
                name="pydantic_agent",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_AGENT}/{NODE_AGENT_PYDANTIC}",
                parameters=parameters,
            ),
        ]
    )
