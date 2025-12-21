from rclpy.node import Node
from rclpy.parameter import Parameter

from continuum.constants import PARAM_NODE_DESCRIPTION, PARAM_NODE_NAME


class BaseNode(Node):
    """Base class for all Continuum nodes."""

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Declare parameters for user-facing node information
        self.declare_parameter(PARAM_NODE_NAME, "")
        self.declare_parameter(PARAM_NODE_DESCRIPTION, "")

        # Register publishers and subscribers
        self.register_publishers()
        self.register_subscribers()

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
