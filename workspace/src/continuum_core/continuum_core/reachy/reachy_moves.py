import asyncio
import random
from concurrent.futures import Future
from typing import Callable
from typing import Optional

from pydantic import BaseModel
from rclpy.timer import Timer
from reachy_mini import ReachyMini
from reachy_mini.motion.recorded_move import RecordedMove, RecordedMoves

from continuum_core.shared.base_node import BaseNode

# Predefined emotions and dances
REACHY_EMOTIONS_REPO_ID = "pollen-robotics/reachy-mini-emotions-library"
REACHY_DANCES_REPO_ID = "pollen-robotics/reachy-mini-dances-library"

# Handpicked
IDLE_TIMER_SECONDS = 15.0  # Trigger 4 times per minute
IDLE_TIMER_PROBABILITY = 0.25  # Trigger 25% of the time
IDLE_MOVES = [
    "attentive1",
    "attentive2",
    "boredom1",
    "boredom2",
    "calming1",
    "curious1",
    "helpful1",
    "helpful2",
    "indifferent1",
    "inquiring1",
    "inquiring2",
    "inquiring3",
    "lonely1",
    "serenity1",
    "shy1",
    "sleep1",
    "thoughtful1",
    "thoughtful2",
    "welcoming1",
    "welcoming2",
]


class ReachyMovesState(BaseModel):
    """Tracks the state for Reachy movement operations."""

    is_moving: bool = False
    is_timer_active: bool = False


class ReachyMoves:
    """Handles movement operations for Reachy Mini."""

    def __init__(
        self,
        node: BaseNode,
        core_loop: asyncio.AbstractEventLoop,
        get_mini: Callable[[], Optional[ReachyMini]],
    ):
        self._node = node
        self._core_loop = core_loop
        self._get_mini = get_mini
        self._logger = node.get_logger()
        self._state = ReachyMovesState()

        # Pre-recorded emotions
        self._recorded_emotions = RecordedMoves(REACHY_EMOTIONS_REPO_ID)
        emotions_ids = self._recorded_emotions.list_moves()
        self._logger.info(f"Recorded emotions ({len(emotions_ids)}): {emotions_ids}")

        # Pre-recorded dances
        self._recorded_dances = RecordedMoves(REACHY_DANCES_REPO_ID)
        dances_ids = self._recorded_dances.list_moves()
        self._logger.info(f"Recorded dances ({len(dances_ids)}): {dances_ids}")

        self._movement_timer: Optional[Timer] = None
        self._node.get_logger().info("Reachy Moves initialized.")

    @property
    def state(self) -> ReachyMovesState:
        return self._state

    @property
    def recorded_emotions(self) -> RecordedMoves:
        return self._recorded_emotions

    @property
    def recorded_dances(self) -> RecordedMoves:
        return self._recorded_dances

    def start_movement_timer(self) -> None:
        """Start the movement timer."""
        self._logger.info("Starting movement timer.")
        self._movement_timer = self._node.create_timer(IDLE_TIMER_SECONDS, self._movement_check)

    def stop_movement_timer(self) -> None:
        """Stop the movement timer."""
        self._logger.info("Stopping movement timer.")
        if self._movement_timer is not None:
            self._movement_timer.cancel()
            self._movement_timer = None

    def _movement_check(self) -> None:
        """Timer callback to trigger random idle moves."""
        if random.random() > IDLE_TIMER_PROBABILITY:
            return
        random_idle_id = random.choice(IDLE_MOVES)
        random_idle_move: RecordedMove = self._recorded_emotions.get(random_idle_id)
        self._logger.info(f"Playing random idle move ({random_idle_id}): {random_idle_move.description}")
        self.do_move(random_idle_id, random_idle_move)

    def do_random_emotion(self) -> None:
        """Play a random emotion move."""
        random_emotion_id = random.choice(self._recorded_emotions.list_moves())
        random_emotion: RecordedMove = self._recorded_emotions.get(random_emotion_id)
        self._logger.info(f"Playing random emotion ({random_emotion_id}): {random_emotion.description}")
        self.do_move(random_emotion_id, random_emotion)

    def do_random_dance(self) -> None:
        """Play a random dance move."""
        random_dance_id = random.choice(self._recorded_dances.list_moves())
        random_dance: RecordedMove = self._recorded_dances.get(random_dance_id)
        self._logger.info(f"Playing random dance ({random_dance_id}): {random_dance.description}")
        self.do_move(random_dance_id, random_dance)

    def do_move(self, move_id: str, move: RecordedMove) -> None:
        mini = self._get_mini()
        if mini is None:
            self._logger.warning("Reachy Mini not connected.")
            return
        if self._state.is_moving:
            self._logger.warning("Already moving.")
            return
        self._state.is_moving = True
        future = asyncio.run_coroutine_threadsafe(mini.async_play_move(move=move, sound=True), self._core_loop)
        future.add_done_callback(lambda f: self._on_move_played(f, move_id))

    def _on_move_played(self, future: Future, move_id: str) -> None:
        try:
            future.result()  # Raises exception if the coroutine failed
            self._logger.info(f"Move played: {move_id}")
        except Exception as e:
            self._logger.error(f"Move failed ({move_id}): {e}")
        finally:
            self._state.is_moving = False
