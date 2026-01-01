import logging

from rclpy.impl.rcutils_logger import RcutilsLogger


class RosLogHandler(logging.Handler):
    """Forward Python logging to ROS logger.

    This handler bridges Python's standard logging module with ROS 2's logging system,
    allowing libraries that use standard Python logging to have their logs appear in ROS logs.
    """

    def __init__(self, ros_logger: RcutilsLogger) -> None:
        """Initialize the handler with a ROS logger."""
        super().__init__()
        self._ros_logger = ros_logger

    def emit(self, record: logging.LogRecord) -> None:
        """Emit a log record by forwarding it to the ROS logger."""
        try:
            log_message = self.format(record)
            if record.levelno >= logging.ERROR:
                self._ros_logger.error(log_message)
            elif record.levelno >= logging.WARNING:
                self._ros_logger.warning(log_message)
            elif record.levelno >= logging.INFO:
                self._ros_logger.info(log_message)
            else:
                self._ros_logger.debug(log_message)
        except Exception:
            self.handleError(record)
