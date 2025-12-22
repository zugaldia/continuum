"""Base class for app nodes."""

from abc import ABC

from continuum_core.shared.queue_node import QueueNode


class BaseAppNode(QueueNode, ABC):
    """Base class for all app nodes in Continuum."""

    def __init__(self, node_name: str):
        super().__init__(node_name)

    def on_shutdown(self) -> None:
        """Clean up app node resources."""
        self.get_logger().info("App node shutting down.")
        super().on_shutdown()
