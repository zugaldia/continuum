from pathlib import Path
from time import time
from typing import Optional

import rclpy
from pydantic import BaseModel
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher
from reachy_mini import ReachyMini

from continuum.constants import (
    QOS_DEPTH_DEFAULT,
    CONTINUUM_NAMESPACE,
    PATH_INPUT,
    TOPIC_JOYSTICK_BUTTON_EVENT,
    TOPIC_JOYSTICK_AXIS_EVENT,
    PARAM_ASR_NODE,
    PARAM_ASR_NODE_DEFAULT,
    PARAM_TTS_NODE,
    PARAM_TTS_NODE_DEFAULT,
    PARAM_LLM_NODE,
    PARAM_LLM_NODE_DEFAULT,
    PARAM_SYSTEM_PROMPT_PATH,
    PARAM_SYSTEM_PROMPT_PATH_DEFAULT,
    PATH_ASR,
    PATH_TTS,
    PATH_LLM,
    TOPIC_LLM_REQUEST,
    TOPIC_ASR_REQUEST,
    TOPIC_ASR_RESPONSE,
    TOPIC_TTS_REQUEST,
    TOPIC_TTS_RESPONSE,
    TOPIC_LLM_RESPONSE,
    ERROR_CODE_SUCCESS,
)
from continuum.models import JoystickAxis, JoystickButton
from continuum.utils import generate_unique_id, none_if_empty, strip_markdown, is_empty
from continuum_core.reachy.prompts import SYSTEM_PROMPT
from continuum_core.reachy.reachy_listens import ReachyListens
from continuum_core.reachy.reachy_moves import ReachyMoves
from continuum_core.reachy.reachy_sees import ReachySees
from continuum_core.reachy.reachy_speaks import ReachySpeaks
from continuum_core.shared.async_node import AsyncNode
from continuum_interfaces.msg import (
    JoystickButtonEvent,
    JoystickAxisEvent,
    AsrRequest,
    TtsRequest,
    LlmRequest,
    AsrResponse,
    TtsResponse,
    LlmResponse,
)

# Connection configuration
REACHY_CONNECTION_RETRIES = 3


class ReachyState(BaseModel):
    """Tracks the state for Reachy."""

    active_session_id: Optional[str] = None
    state_id: Optional[str] = None  # LLM memory
    recording_stop_time: Optional[float] = None


class ReachyNode(AsyncNode):
    _asr_publisher: Publisher[AsrRequest]
    _tts_publisher: Publisher[TtsRequest]
    _llm_publisher: Publisher[LlmRequest]
    _asr_node: str
    _tts_node: str
    _llm_node: str
    _system_prompt_path: str

    def __init__(self) -> None:
        super().__init__("reachy_node")
        self.set_node_info(name="Reachy Node", description="Support for Pollen Robotics / Hugging Face Reachy Mini")
        self._state = ReachyState()
        self._mini: Optional[ReachyMini] = None

        self._reachy_moves = ReachyMoves(node=self, core_loop=self._core_loop, get_mini=self._get_mini)
        self._reachy_listens = ReachyListens(node=self, core_loop=self._core_loop, get_mini=self._get_mini)
        self._reachy_sees = ReachySees(node=self, core_loop=self._core_loop, get_mini=self._get_mini)
        self._reachy_speaks = ReachySpeaks(
            node=self, core_loop=self._core_loop, get_mini=self._get_mini, on_queue_empty=self._on_tts_queue_empty
        )

        if self.debug_mode:
            for emotion_id in self._reachy_moves.recorded_emotions.list_moves():
                description = self._reachy_moves.recorded_emotions.get(emotion_id).description
                self.get_logger().info(f"- Emotion: {emotion_id}: {description}")
            for dance_id in self._reachy_moves.recorded_dances.list_moves():
                description = self._reachy_moves.recorded_dances.get(dance_id).description
                self.get_logger().info(f"- Dance: {dance_id}: {description}")

        self.get_logger().info("Reachy node initialized.")

    def _get_mini(self) -> Optional[ReachyMini]:
        """Get the Reachy Mini instance."""
        return self._mini

    def _load_system_prompt(self) -> str:
        """Load the system prompt from a file or use default."""
        if is_empty(self._system_prompt_path):
            self.get_logger().info("Using default system prompt")
            return SYSTEM_PROMPT

        try:
            # We need to start caching this content
            prompt_path = Path(self._system_prompt_path)
            self.get_logger().info(f"Loading system prompt from: {prompt_path}")
            with open(prompt_path, "r", encoding="utf-8") as f:
                prompt = f.read().strip()
            return prompt
        except Exception as e:
            self.get_logger().error(f"Error loading system prompt from {self._system_prompt_path}: {e}")
            return SYSTEM_PROMPT

    def register_parameters(self) -> None:
        """Register the dictation app parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_ASR_NODE, PARAM_ASR_NODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            PARAM_TTS_NODE, PARAM_TTS_NODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            PARAM_LLM_NODE, PARAM_LLM_NODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            PARAM_SYSTEM_PROMPT_PATH,
            PARAM_SYSTEM_PROMPT_PATH_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

        # Set node names before registering any publishers/subscribers
        self._asr_node = self._get_str_param(PARAM_ASR_NODE)
        self._tts_node = self._get_str_param(PARAM_TTS_NODE)
        self._llm_node = self._get_str_param(PARAM_LLM_NODE)
        self._system_prompt_path = self._get_str_param(PARAM_SYSTEM_PROMPT_PATH)

    def register_publishers(self) -> None:
        # Ability to issue ASR requests
        topic_asr_request = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{self._asr_node}/{TOPIC_ASR_REQUEST}"
        self._asr_publisher = self.create_publisher(AsrRequest, topic_asr_request, QOS_DEPTH_DEFAULT)

        # Ability to issue TTS requests
        topic_tts_request = f"/{CONTINUUM_NAMESPACE}/{PATH_TTS}/{self._tts_node}/{TOPIC_TTS_REQUEST}"
        self._tts_publisher = self.create_publisher(TtsRequest, topic_tts_request, QOS_DEPTH_DEFAULT)

        # Ability to issue LLM requests
        topic_llm_request = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{self._llm_node}/{TOPIC_LLM_REQUEST}"
        self._llm_publisher = self.create_publisher(LlmRequest, topic_llm_request, QOS_DEPTH_DEFAULT)

    def register_subscribers(self) -> None:
        """Register the reachy request subscriber."""
        self.create_subscription(
            JoystickButtonEvent,
            f"/{CONTINUUM_NAMESPACE}/{PATH_INPUT}/{TOPIC_JOYSTICK_BUTTON_EVENT}",
            self._joystick_button_callback,
            QOS_DEPTH_DEFAULT,
        )
        self.create_subscription(
            JoystickAxisEvent,
            f"/{CONTINUUM_NAMESPACE}/{PATH_INPUT}/{TOPIC_JOYSTICK_AXIS_EVENT}",
            self._joystick_axis_callback,
            QOS_DEPTH_DEFAULT,
        )

        # Listen to ASR responses
        topic_asr_response = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{self._asr_node}/{TOPIC_ASR_RESPONSE}"
        self.create_subscription(AsrResponse, topic_asr_response, self._on_asr_response, QOS_DEPTH_DEFAULT)

        # Listen to TTS responses
        topic_tts_response = f"/{CONTINUUM_NAMESPACE}/{PATH_TTS}/{self._tts_node}/{TOPIC_TTS_RESPONSE}"
        self.create_subscription(TtsResponse, topic_tts_response, self._on_tts_response, QOS_DEPTH_DEFAULT)

        # Listen to LLM responses
        topic_llm_response = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{self._llm_node}/{TOPIC_LLM_RESPONSE}"
        self.create_subscription(LlmResponse, topic_llm_response, self._on_llm_response, QOS_DEPTH_DEFAULT)

    def _connect(self) -> None:
        if self._mini:
            self.get_logger().warning("Reachy Mini already connected.")
            return

        for attempt in range(1, REACHY_CONNECTION_RETRIES + 1):
            try:
                # Allow connecting to Reachy Mini on the local network
                self.get_logger().info(f"Connecting to Reachy Mini (attempt {attempt}/{REACHY_CONNECTION_RETRIES}).")
                self._mini = ReachyMini(localhost_only=False)
                self.get_logger().info("Reachy Mini connected.")
                self._reachy_listens.start_listening_timer()
                self._reachy_moves.start_movement_timer()
                return
            except Exception as e:
                if attempt < REACHY_CONNECTION_RETRIES:
                    self.get_logger().warning(f"Connection attempt {attempt} failed: {e}. Retrying.")
                else:
                    self.get_logger().error(
                        f"Failed to connect to Reachy Mini after {REACHY_CONNECTION_RETRIES} attempts: {e}"
                    )

    def _disconnect(self) -> None:
        # Do these regardless of Reachy connectivity state
        self._reachy_listens.stop_listening_timer()
        self._reachy_moves.stop_movement_timer()
        self._reachy_speaks.clear_tts_queue()

        if not self._mini:
            self.get_logger().warning("Reachy Mini not connected.")
            return
        try:
            self.get_logger().info("Disconnecting from Reachy Mini.")
            self._mini.__exit__(None, None, None)
            self._mini = None
            self.get_logger().info("Reachy Mini disconnected.")
        except Exception as e:
            self.get_logger().error(f"Error disconnecting Reachy Mini: {e}")

    def _joystick_button_callback(self, msg: JoystickButtonEvent) -> None:
        if msg.button == JoystickButton.BUTTON_SELECT.value:
            self._connect()
        elif msg.button == JoystickButton.BUTTON_START.value:
            self._disconnect()
        elif msg.button == JoystickButton.BUTTON_Y.value:
            if self._state.active_session_id:
                self._logger.warning("Active session in progress.")
                return
            self._reachy_listens.start_recording()
        elif msg.button == JoystickButton.BUTTON_X.value:
            output_path = self._reachy_listens.stop_recording()
            if output_path:
                self._state.recording_stop_time = time()
                self._state.active_session_id = generate_unique_id()
                self.get_logger().info(f"Recording stopped, starting ASR for session {self._state.active_session_id}")
                self._asr_publisher.publish(
                    AsrRequest(session_id=self._state.active_session_id, audio_path=str(output_path), language="en")
                )

    def _joystick_axis_callback(self, msg: JoystickAxisEvent) -> None:
        if msg.axis == JoystickAxis.AXIS_LEFT.value:
            self._reachy_moves.do_random_emotion()
        elif msg.axis == JoystickAxis.AXIS_RIGHT.value:
            self._reachy_moves.do_random_dance()
        elif msg.axis == JoystickAxis.AXIS_UP.value:
            self._reachy_sees.take_photo()
        elif msg.axis == JoystickAxis.AXIS_DOWN.value:
            # Project root is 6 levels up from this file
            project_root = Path(__file__).resolve().parents[5]
            jfk_path = project_root / "assets" / "audio" / "jfk.wav"
            self._reachy_speaks.play_audio_queued(jfk_path)

    def _on_asr_response(self, msg: AsrResponse) -> None:
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed = time() - self._state.recording_stop_time if self._state.recording_stop_time else 0
        self.get_logger().info(f"Received ASR response: {msg} (elapsed: {elapsed:.2f}s)")
        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"ASR error: {msg.error_message}")
            return

        state_id = none_if_empty(self._state.state_id) or ""
        system_prompt = self._load_system_prompt()
        self._llm_publisher.publish(
            LlmRequest(
                session_id=self._state.active_session_id,
                system_prompt=system_prompt,
                state_id=state_id,
                content_text=msg.transcription,
            )
        )

    @staticmethod
    def _extract_emotion(text: str) -> tuple[Optional[str], str]:
        """Extract optional emotion keyword from text. Example: [cheerful] Reachy is functioning at 100% efficiency."""
        text = text.strip()
        if not text.startswith("["):
            return None, text

        closing_bracket_index = text.find("]")
        if closing_bracket_index == -1:
            return None, text  # No closing bracket found, treat as normal text

        emotion = text[1:closing_bracket_index].strip()
        cleaned_text = text[closing_bracket_index + 1 :].strip()
        return (emotion, cleaned_text) if emotion else (None, text)

    def _on_llm_response(self, msg: LlmResponse) -> None:
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed = time() - self._state.recording_stop_time if self._state.recording_stop_time else 0
        self.get_logger().info(f"Received LLM response: {msg} (elapsed: {elapsed:.2f}s)")

        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"LLM error: {msg.error_message}")
            return

        # Each LLM response creates a new ID that we need to pass to the next request
        self._state.state_id = msg.state_id

        try:
            # Extract optional emotion keyword from the LLM response
            emotion, content_without_emotion = self._extract_emotion(msg.content_text)
            if emotion:
                emotion_id = f"{emotion}1"
                if emotion_id in self._reachy_moves.recorded_emotions.list_moves():
                    self.get_logger().info(f"Detected emotion: {emotion_id}")
                    self._reachy_moves.do_move(emotion_id, self._reachy_moves.recorded_emotions.get(emotion_id))

            # Instead of sending the raw response from the LLM, which is generally multi-line and Markdown formatted,
            # we create a plain text version with the same original lines and submit smaller requests to the TTS engine
            # to reduce the latency of the (initial) response.
            text = strip_markdown(content_without_emotion)
            lines = text.splitlines()
            for line in lines:
                if not is_empty(line):
                    self._tts_publisher.publish(
                        TtsRequest(session_id=self._state.active_session_id, text=line, language="en")
                    )
        except Exception as e:
            # Send the raw response in case of an error as a fallback.
            self.get_logger().warning(f"Failed to process LLM response: {e}")
            self._tts_publisher.publish(
                TtsRequest(session_id=self._state.active_session_id, text=msg.content_text, language="en")
            )

    def _on_tts_response(self, msg: TtsResponse) -> None:
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed = time() - self._state.recording_stop_time if self._state.recording_stop_time else 0
        self.get_logger().info(f"Received TTS response: {msg} (elapsed: {elapsed:.2f}s)")
        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"TTS error: {msg.error_message}")
            return

        self._reachy_speaks.play_audio_queued(msg.audio_path)

    def _on_tts_queue_empty(self) -> None:
        """Called when the TTS queue becomes empty after finishing playback."""
        self._state.active_session_id = None
        self._state.recording_stop_time = None
        self.get_logger().info("Session complete, active_session_id cleared")

    def on_shutdown(self) -> None:
        """Clean up reachy node resources."""
        self.get_logger().info("Reachy node shutting down.")
        self._disconnect()
        super().on_shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            reachy_node = ReachyNode()
            rclpy.spin(reachy_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
