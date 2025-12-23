from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from continuum.constants import (
    CONTINUUM_NAMESPACE,
    NODE_APP_DICTATION,
    NODE_ASR_FASTER_WHISPER,
    NODE_LLM_OLLAMA,
    NODE_LLM_OPENAI,
    NODE_LLM_GOOGLE,
    PATH_APP,
    PATH_ASR,
    PATH_LLM,
    PROFILE_LOCAL,
    PROFILE_CLOUD,
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
    """Launch all core package nodes for the desktop application."""

    config_file_arg, parameters = get_config_file_argument()

    return LaunchDescription(
        [
            # Config launch argument
            config_file_arg,
            # Health nodes
            Node(
                package="continuum_core",
                executable="heartbeat_node",
                name="heartbeat_node",
                namespace=CONTINUUM_NAMESPACE,
                parameters=parameters,
            ),
            Node(
                package="continuum_core",
                executable="echo_node",
                name="echo_node",
                namespace=CONTINUUM_NAMESPACE,
                parameters=parameters,
            ),
            # ASR nodes
            Node(
                package="continuum_core",
                executable="faster_whisper_asr_node",
                name="faster_whisper_asr_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_ASR}/{NODE_ASR_FASTER_WHISPER}",
                parameters=parameters,
            ),
            # LLM nodes
            Node(
                package="continuum_core",
                executable="ollama_llm_node",
                name="ollama_llm_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_LLM}/{NODE_LLM_OLLAMA}",
                parameters=parameters,
            ),
            Node(
                package="continuum_core",
                executable="openai_llm_node",
                name="openai_llm_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_LLM}/{NODE_LLM_OPENAI}",
                parameters=parameters,
            ),
            Node(
                package="continuum_core",
                executable="google_llm_node",
                name="google_llm_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_LLM}/{NODE_LLM_GOOGLE}",
                parameters=parameters,
            ),
            # App nodes
            Node(
                package="continuum_core",
                executable="dictation_app_node",
                name="dictation_app_node_local",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_APP}/{NODE_APP_DICTATION}/{PROFILE_LOCAL}",
                parameters=parameters,
            ),
            Node(
                package="continuum_core",
                executable="dictation_app_node",
                name="dictation_app_node_cloud",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_APP}/{NODE_APP_DICTATION}/{PROFILE_CLOUD}",
                parameters=parameters,
            ),
        ]
    )
