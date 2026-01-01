import asyncio
from pathlib import Path
from typing import Callable
from typing import Optional

import cv2
import numpy as np
import numpy.typing as npt
from pydantic import BaseModel
from rclpy.node import Node
from reachy_mini import ReachyMini

from continuum.utils import create_timestamped_filename


class ReachySeesState(BaseModel):
    """Tracks the state for Reachy camera operations."""

    pass


class ReachySees:
    """Handles camera operations for Reachy Mini."""

    def __init__(
        self,
        node: Node,
        core_loop: asyncio.AbstractEventLoop,
        get_mini: Callable[[], Optional[ReachyMini]],
    ):
        self._node = node
        self._core_loop = core_loop
        self._get_mini = get_mini
        self._logger = node.get_logger()
        self._state = ReachySeesState()
        self._logger.info("Reachy Sees initialized.")

    @property
    def state(self) -> ReachySeesState:
        return self._state

    def take_photo(self) -> Optional[Path]:
        """Take a photo with Reachy's camera."""
        mini = self._get_mini()
        if mini is None:
            self._logger.warning("Reachy Mini not connected.")
            return None

        frame: Optional[npt.NDArray[np.uint8]] = mini.media.get_frame()
        if frame is None:
            self._logger.warning("Failed to capture frame: camera not available.")
            return None

        try:
            filepath = create_timestamped_filename("reachy_photo", "png")
            cv2.imwrite(str(filepath), frame)
            self._logger.info(f"Photo saved to {filepath}")
            return filepath
        except Exception as e:
            self._logger.error(f"Failed to save photo: {e}")
            return None
