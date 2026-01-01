"""Base class for app nodes."""

from abc import ABC

from continuum_core.shared.queue_node import QueueNode


class BaseAppNode(QueueNode, ABC):
    """Base class for all app nodes in Continuum."""

    _active_sessions: set[str]

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._active_sessions = set()

    def add_active_session(self, session_id: str) -> None:
        """Add a session ID to track."""
        self._active_sessions.add(session_id)

    def has_active_session(self, session_id: str) -> bool:
        """Check if a session ID is being tracked."""
        return session_id in self._active_sessions

    def remove_active_session(self, session_id: str) -> None:
        """Remove a session ID from tracking."""
        self._active_sessions.discard(session_id)

    def on_shutdown(self) -> None:
        """Clean up app node resources."""
        self.get_logger().info("App node shutting down.")
        super().on_shutdown()
