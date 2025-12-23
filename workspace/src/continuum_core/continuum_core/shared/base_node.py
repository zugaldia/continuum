from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter

from continuum.constants import PARAM_DEBUG_MODE, PARAM_NODE_DESCRIPTION, PARAM_NODE_NAME


class BaseNode(Node):
    """Base class for all Continuum nodes."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.register_parameters()
        self.register_publishers()
        self.register_subscribers()

    def register_parameters(self) -> None:
        """Override to register node parameters."""
        self.declare_parameter(PARAM_NODE_NAME, "", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(PARAM_NODE_DESCRIPTION, "", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(PARAM_DEBUG_MODE, False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

    def register_publishers(self) -> None:
        """Override to register node publishers."""
        pass

    def register_subscribers(self) -> None:
        """Override to register node subscribers."""
        pass

    def destroy_node(self) -> None:
        """Clean up node resources."""
        self.on_shutdown()
        super().destroy_node()

    def on_shutdown(self) -> None:
        """Override to perform cleanup before node destruction."""
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

    def _get_str_param(self, name: str) -> str:
        """Get a string parameter value."""
        return self.get_parameter(name).get_parameter_value().string_value

    @property
    def debug_mode(self) -> bool:
        """Get the debug mode setting."""
        return self._get_bool_param(PARAM_DEBUG_MODE)
