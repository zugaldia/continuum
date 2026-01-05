from typing import Optional

import rclpy
import soundfile as sf
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
    PARAM_AGENT_NODE,
    PARAM_AGENT_NODE_DEFAULT,
    PATH_ASR,
    PATH_TTS,
    TOPIC_AGENT_REQUEST,
    TOPIC_ASR_REQUEST,
    TOPIC_ASR_RESPONSE,
    TOPIC_TTS_REQUEST,
    TOPIC_TTS_RESPONSE,
    TOPIC_AGENT_RESPONSE,
    TOPIC_AGENT_STREAMING_RESPONSE,
    ERROR_CODE_SUCCESS,
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
    DEFAULT_AUDIO_FORMAT,
    PATH_HARDWARE,
)
from continuum.models import JoystickAxis, JoystickButton
from continuum.utils import (
    generate_unique_id,
    none_if_empty,
    strip_markdown,
    is_empty,
    generate_order_id,
    compute_elapsed_ms,
)
from continuum_core.reachy.models import ReachyAudio, ReachyState
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
    AgentRequest,
    AsrResponse,
    TtsResponse,
    AgentResponse,
    AgentStreamingResponse,
)

# Connection configuration
REACHY_CONNECTION_RETRIES = 3


class ReachyNode(AsyncNode):
    _asr_publisher: Publisher[AsrRequest]
    _tts_publisher: Publisher[TtsRequest]
    _agent_publisher: Publisher[AgentRequest]
    _asr_node: str
    _tts_node: str
    _agent_node: str

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

        self.get_logger().info(f"Reachy node initialized ({self.storage_path}).")

    def _get_mini(self) -> Optional[ReachyMini]:
        """Get the Reachy Mini instance."""
        return self._mini

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
            PARAM_AGENT_NODE, PARAM_AGENT_NODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        # Set node names before registering any publishers/subscribers
        self._asr_node = self._get_str_param(PARAM_ASR_NODE)
        self._tts_node = self._get_str_param(PARAM_TTS_NODE)
        self._agent_node = self._get_str_param(PARAM_AGENT_NODE)

    def register_publishers(self) -> None:
        # Ability to issue ASR requests
        topic_asr_request = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{self._asr_node}/{TOPIC_ASR_REQUEST}"
        self._asr_publisher = self.create_publisher(AsrRequest, topic_asr_request, QOS_DEPTH_DEFAULT)

        # Ability to issue TTS requests
        topic_tts_request = f"/{CONTINUUM_NAMESPACE}/{PATH_TTS}/{self._tts_node}/{TOPIC_TTS_REQUEST}"
        self._tts_publisher = self.create_publisher(TtsRequest, topic_tts_request, QOS_DEPTH_DEFAULT)

        # Ability to issue agent requests
        topic_agent_request = f"/{CONTINUUM_NAMESPACE}/{PATH_HARDWARE}/{self._agent_node}/{TOPIC_AGENT_REQUEST}"
        self._agent_publisher = self.create_publisher(AgentRequest, topic_agent_request, QOS_DEPTH_DEFAULT)

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

        # Listen to agent responses
        topic_agent_response = f"/{CONTINUUM_NAMESPACE}/{PATH_HARDWARE}/{self._agent_node}/{TOPIC_AGENT_RESPONSE}"
        self.create_subscription(AgentResponse, topic_agent_response, self._on_agent_response, QOS_DEPTH_DEFAULT)

        # Listen to agent streaming responses
        topic_agent_streaming_response = (
            f"/{CONTINUUM_NAMESPACE}/{PATH_HARDWARE}/{self._agent_node}/{TOPIC_AGENT_STREAMING_RESPONSE}"
        )
        self.create_subscription(
            AgentStreamingResponse, topic_agent_streaming_response, self._on_agent_streaming_response, QOS_DEPTH_DEFAULT
        )

    def _connect(self) -> None:
        if self._mini:
            self.get_logger().warning("Reachy Mini already connected.")
            return

        for attempt in range(1, REACHY_CONNECTION_RETRIES + 1):
            try:
                # Allow connecting to Reachy Mini on the local network
                self.get_logger().info(f"Connecting to Reachy Mini (attempt {attempt}/{REACHY_CONNECTION_RETRIES}).")
                self._mini = ReachyMini(connection_mode="network")
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
        self._reachy_speaks.clear_audio_queue()

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
            reachy_audio = self._reachy_listens.stop_recording()
            if reachy_audio:
                self._state.active_session_id = generate_unique_id()
                self._reachy_speaks.clear_audio_queue()

                asr_request = AsrRequest(
                    session_id=self._state.active_session_id,
                    audio_data=list(reachy_audio.audio_data),
                    format=DEFAULT_AUDIO_FORMAT,
                    channels=reachy_audio.channels,
                    sample_rate=reachy_audio.sample_rate,
                    sample_width=reachy_audio.sample_width,
                    language="en",
                )
                self._state.request_timestamp = asr_request.timestamp
                self.get_logger().info(f"Recording stopped, starting ASR for session {self._state.active_session_id}")
                self._asr_publisher.publish(asr_request)

    def _joystick_axis_callback(self, msg: JoystickAxisEvent) -> None:
        if msg.axis == JoystickAxis.AXIS_LEFT.value:
            self._reachy_moves.do_random_emotion()
        elif msg.axis == JoystickAxis.AXIS_RIGHT.value:
            self._reachy_moves.do_random_dance()
        elif msg.axis == JoystickAxis.AXIS_UP.value:
            self._reachy_sees.take_photo()
        elif msg.axis == JoystickAxis.AXIS_DOWN.value:
            self._play_test_audio()

    def _on_asr_response(self, msg: AsrResponse) -> None:
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed_ms = compute_elapsed_ms(self._state.request_timestamp, msg.timestamp)
        self.get_logger().info(f"Received ASR response: {msg} (elapsed: {elapsed_ms:.0f}ms)")
        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"ASR error: {msg.error_message}")
            return

        state_id = none_if_empty(self._state.state_id) or ""
        self._agent_publisher.publish(
            AgentRequest(
                session_id=self._state.active_session_id,
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

    def _on_agent_response(self, msg: AgentResponse) -> None:
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed_ms = compute_elapsed_ms(self._state.request_timestamp, msg.timestamp)
        self.get_logger().info(f"Received agent response: {msg} (elapsed: {elapsed_ms:.0f}ms)")

        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"Agent error: {msg.error_message}")
            return

        # Each agent response creates a new ID that we need to pass to the next request
        self._state.state_id = msg.state_id

        if is_empty(msg.content_text):
            # The content is in the streaming responses instead.
            self.get_logger().warning("Agent response is empty.")
            return

        try:
            # Extract optional emotion keyword from the agent response
            emotion, content_without_emotion = self._extract_emotion(msg.content_text)
            if emotion:
                emotion_id = f"{emotion}1"
                if emotion_id in self._reachy_moves.recorded_emotions.list_moves():
                    self.get_logger().info(f"Detected emotion: {emotion_id}")
                    self._reachy_moves.do_move(emotion_id, self._reachy_moves.recorded_emotions.get(emotion_id))

            # Instead of sending the raw response from the agent, which is generally multi-line and Markdown formatted,
            # we create a plain text version with the same original lines and submit smaller requests to the TTS engine
            # to reduce the latency of the (initial) response.
            text = strip_markdown(content_without_emotion)
            lines = [line for line in text.splitlines() if not is_empty(line)]
            for line_index, line in enumerate(lines):
                is_initial = line_index == 0
                is_final = line_index == len(lines) - 1
                order_id = generate_order_id()
                self._tts_publisher.publish(
                    TtsRequest(
                        session_id=self._state.active_session_id,
                        is_initial=is_initial,
                        is_final=is_final,
                        order_id=order_id,
                        text=line,
                        language="en",
                    )
                )
        except Exception as e:
            # Send the raw response in case of an error as a fallback.
            self.get_logger().warning(f"Failed to process agent response: {e}")
            self._tts_publisher.publish(
                TtsRequest(
                    session_id=self._state.active_session_id,
                    is_initial=True,
                    is_final=True,
                    order_id=generate_order_id(),
                    text=msg.content_text,
                    language="en",
                )
            )

    def _on_agent_streaming_response(self, msg: AgentStreamingResponse) -> None:
        """Callback for agent streaming responses."""
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed_ms = compute_elapsed_ms(self._state.request_timestamp, msg.timestamp)
        self.get_logger().info(f"Received agent streaming response: {msg} (elapsed: {elapsed_ms:.0f}ms)")
        self._reachy_moves.do_random_emotion()

    def _on_tts_response(self, msg: TtsResponse) -> None:
        if msg.session_id != self._state.active_session_id:
            return  # Ignore all responses but ours

        elapsed_ms = compute_elapsed_ms(self._state.request_timestamp, msg.timestamp)
        self.get_logger().info(f"Received TTS response (elapsed: {elapsed_ms:.0f}ms)")
        self._log_message_redacted(msg)
        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"TTS error: {msg.error_message}")
            return

        # Pass the TTS response to ReachySpeaks for buffering and ordered playback
        self._reachy_speaks.queue_audio(
            ReachyAudio(
                audio_data=bytes(msg.audio_data),
                sample_rate=msg.sample_rate,
                channels=msg.channels,
                sample_width=msg.sample_width,
                order_id=msg.order_id,
                is_initial=msg.is_initial,
            )
        )

    def _on_tts_queue_empty(self) -> None:
        """Called when the TTS queue becomes empty after finishing playback."""
        self._state.active_session_id = None
        self._state.request_timestamp = None
        self.get_logger().info("Session complete, active_session_id cleared")

    def _play_test_audio(self) -> None:
        try:
            # Load test audio file and extract raw PCM data (see assets/audio for examples)
            # It assumes test.wav is mono, 16 kHz, 16-bit
            test_path = self.storage_path / "test.wav"
            if not test_path.exists():
                self.get_logger().warning(
                    f"Test audio file not found at {test_path}. "
                    "Place a mono, 16kHz, 16-bit WAV file there to enable test audio playback."
                )
                return

            audio_data, _ = sf.read(test_path, dtype="int16")
            test_audio_data = audio_data.tobytes()
            self._reachy_speaks.clear_audio_queue()
            self._reachy_speaks.queue_audio(
                ReachyAudio(
                    audio_data=test_audio_data,
                    sample_rate=DEFAULT_AUDIO_SAMPLE_RATE,
                    channels=DEFAULT_AUDIO_CHANNELS,
                    sample_width=DEFAULT_AUDIO_SAMPLE_WIDTH,
                    is_initial=True,
                )
            )
        except Exception as e:
            self.get_logger().error(f"Failed to play test audio: {e}")

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
