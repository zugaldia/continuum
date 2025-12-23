from launch import LaunchDescription
from launch_ros.actions import Node
from continuum.constants import (
    CONTINUUM_NAMESPACE,
    NODE_APP_DICTATION,
    NODE_ASR_FASTER_WHISPER,
    NODE_LLM_OLLAMA,
    PATH_APP,
    PATH_ASR,
    PATH_LLM,
)


def generate_launch_description():
    """Launch all core package nodes for the desktop application."""

    return LaunchDescription(
        [
            # Health nodes
            Node(
                package="continuum_core",
                executable="heartbeat_node",
                name="heartbeat_node",
                namespace=CONTINUUM_NAMESPACE,
            ),
            Node(
                package="continuum_core",
                executable="echo_node",
                name="echo_node",
                namespace=CONTINUUM_NAMESPACE,
            ),
            # ASR nodes
            Node(
                package="continuum_core",
                executable="faster_whisper_asr_node",
                name="faster_whisper_asr_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_ASR}/{NODE_ASR_FASTER_WHISPER}",
            ),
            # LLM nodes
            Node(
                package="continuum_core",
                executable="ollama_llm_node",
                name="ollama_llm_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_LLM}/{NODE_LLM_OLLAMA}",
            ),
            # App nodes
            Node(
                package="continuum_core",
                executable="dictation_app_node",
                name="dictation_app_node",
                namespace=f"{CONTINUUM_NAMESPACE}/{PATH_APP}/{NODE_APP_DICTATION}",
            ),
        ]
    )
