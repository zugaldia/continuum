import logging
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict

from continuum.constants import (
    PARAM_DEBUG_MODE,
    PARAM_DEBUG_MODE_DEFAULT,
    PARAM_NODE_DESCRIPTION,
    PARAM_NODE_DESCRIPTION_DEFAULT,
    PARAM_NODE_NAME,
    PARAM_NODE_NAME_DEFAULT,
    PARAM_STORAGE_PATH,
    PARAM_STORAGE_PATH_DEFAULT,
)
from continuum.utils import is_empty, get_data_path
from continuum_core.shared import RosLogHandler


class BaseNode(Node):
    """Base class for all Continuum nodes."""

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Forward Python logging to the ROS logger
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.DEBUG)
        ros_handler = RosLogHandler(self.get_logger())
        ros_handler.setFormatter(logging.Formatter("[%(name)s] %(message)s"))
        root_logger.addHandler(ros_handler)

        self.register_parameters()
        self.register_publishers()
        self.register_subscribers()

    def register_parameters(self) -> None:
        """Override to register other node parameters."""
        self.declare_parameter(
            PARAM_DEBUG_MODE, PARAM_DEBUG_MODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        )
        self.declare_parameter(
            PARAM_NODE_NAME, PARAM_NODE_NAME_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            PARAM_NODE_DESCRIPTION,
            PARAM_NODE_DESCRIPTION_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_STORAGE_PATH,
            PARAM_STORAGE_PATH_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

    def register_publishers(self) -> None:
        """Override to register node publishers."""
        pass

    def register_subscribers(self) -> None:
        """Override to register node subscribers."""
        pass

    def set_node_info(self, name: str, description: str) -> None:
        """Set the user-facing node name and description."""
        self.set_parameters(
            [
                Parameter(PARAM_NODE_NAME, Parameter.Type.STRING, name),
                Parameter(PARAM_NODE_DESCRIPTION, Parameter.Type.STRING, description),
            ]
        )

    def _get_bool_param(self, name: str) -> bool:
        """Get a boolean parameter value."""
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_int_param(self, name: str) -> int:
        """Get a integer parameter value."""
        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_double_param(self, name: str) -> float:
        """Get a double parameter value."""
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_str_param(self, name: str) -> str:
        """Get a string parameter value."""
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_bytes_array_param(self, name: str) -> Sequence[bytes]:
        """Get a bytes array parameter value."""
        return self.get_parameter(name).get_parameter_value().byte_array_value

    def _get_bool_array_param(self, name: str) -> Sequence[bool]:
        """Get a boolean array parameter value."""
        return self.get_parameter(name).get_parameter_value().bool_array_value

    def _get_int_array_param(self, name: str) -> Sequence[int]:
        """Get a integer array parameter value."""
        return self.get_parameter(name).get_parameter_value().integer_array_value

    def _get_double_array_param(self, name: str) -> Sequence[float]:
        """Get a double array parameter value."""
        return self.get_parameter(name).get_parameter_value().double_array_value

    def _get_str_array_param(self, name: str) -> Sequence[str]:
        """Get a string array parameter value."""
        return self.get_parameter(name).get_parameter_value().string_array_value

    @property
    def debug_mode(self) -> bool:
        """Get the debug mode setting."""
        return self._get_bool_param(PARAM_DEBUG_MODE)

    @property
    def storage_path(self) -> Path:
        """Get the storage path setting with a fallback."""
        param_value = self._get_str_param(PARAM_STORAGE_PATH)
        storage_path = get_data_path() if is_empty(param_value) else Path(param_value)
        node_path = storage_path / self.get_name()
        node_path.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Using storage path: {node_path}")
        return node_path

    def _log_message_redacted(self, message: Any) -> None:
        """Log a ROS message, redacting sensitive data (audio_data arrays and API keys)."""
        try:
            msg_dict = message_to_ordereddict(message)
            if "audio_data" in msg_dict:
                msg_dict["audio_data"] = f"<{len(msg_dict['audio_data'])} bytes>"
            if "api_key" in msg_dict:
                api_key = str(msg_dict["api_key"])
                msg_dict["api_key"] = f"{api_key[:5]}..." if len(api_key) > 5 else api_key
            self.get_logger().info(f"Message (redacted): {msg_dict}")
        except Exception as e:
            self.get_logger().error(f"Failed to log message with redaction: {e}")

    def destroy_node(self) -> None:
        """Clean up node resources."""
        self.on_shutdown()
        super().destroy_node()

    def on_shutdown(self) -> None:
        """Override to perform cleanup before node destruction."""
        if self.debug_mode:
            self.get_logger().debug(f"Shutting down {self.get_name()}.")
