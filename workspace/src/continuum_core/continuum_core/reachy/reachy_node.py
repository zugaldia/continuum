import asyncio
import random
from collections import deque
from concurrent.futures import Future
from typing import Optional, Deque

import cv2
import numpy as np
import numpy.typing as npt
import rclpy
import soundfile as sf
from rclpy.executors import ExternalShutdownException
from rclpy.timer import Timer
from reachy_mini import ReachyMini
from reachy_mini.motion.recorded_move import RecordedMove, RecordedMoves

from continuum.constants import (
    QOS_DEPTH_DEFAULT,
    CONTINUUM_NAMESPACE,
    PATH_INPUT,
    TOPIC_JOYSTICK_BUTTON_EVENT,
    TOPIC_JOYSTICK_AXIS_EVENT,
)
from continuum.models import JoystickAxis, JoystickButton
from continuum_core.shared.async_node import AsyncNode
from continuum_core.shared.utils import create_timestamped_filename
from continuum_interfaces.msg import JoystickButtonEvent, JoystickAxisEvent

# Predefined emotions and dances
REACHY_EMOTIONS_REPO_ID = "pollen-robotics/reachy-mini-emotions-library"
REACHY_DANCES_REPO_ID = "pollen-robotics/reachy-mini-dances-library"

# Connection configuration
REACHY_CONNECTION_RETRIES = 3

# Audio recording configuration (poll every 10ms to capture samples)
# Reachy produces ~20ms audio chunks, so poll at least 2x that rate
AUDIO_POLL_PERIOD_SECONDS = 0.01

# Audio playback configuration
AUDIO_PLAYBACK_CHUNK_SIZE = 1024

# Audio buffer configuration - keep the last minute of audio
# Reachy produces ~20ms chunks, approximately 50 chunks per second
# 60 seconds × 50 chunks/second = 3000 chunks
AUDIO_BUFFER_MAX_SIZE = 3_000


class ReachyNode(AsyncNode):
    def __init__(self) -> None:
        super().__init__("reachy_node")
        self.set_node_info(name="Reachy Node", description="Support for Pollen Robotics / Hugging Face Reachy Mini")

        self._is_moving = False
        self._is_recording = False
        self._is_talking = False
        self._mini: Optional[ReachyMini] = None
        self._listening_timer: Optional[Timer] = None
        self._playback_timer: Optional[Timer] = None
        self._audio_buffer: Deque[npt.NDArray[np.float32]] = deque(maxlen=AUDIO_BUFFER_MAX_SIZE)

        self._recorded_emotions = RecordedMoves(REACHY_EMOTIONS_REPO_ID)
        emotions_ids = self._recorded_emotions.list_moves()
        self.get_logger().info(f"Recorded emotions ({len(emotions_ids)}): {emotions_ids}")

        self._recorded_dances = RecordedMoves(REACHY_DANCES_REPO_ID)
        dances_ids = self._recorded_dances.list_moves()
        self.get_logger().info(f"Recorded dances ({len(dances_ids)}): {dances_ids}")

        self.get_logger().info("Reachy node initialized.")

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
                self._start_listening_timer()
                return
            except Exception as e:
                if attempt < REACHY_CONNECTION_RETRIES:
                    self.get_logger().warning(f"Connection attempt {attempt} failed: {e}. Retrying.")
                else:
                    self.get_logger().error(
                        f"Failed to connect to Reachy Mini after {REACHY_CONNECTION_RETRIES} attempts: {e}"
                    )

    def _disconnect(self) -> None:
        self._stop_listening_timer()  # Stop the listening timer regardless of Mini connectivity state
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

    def _start_listening_timer(self) -> None:
        self.get_logger().info("Starting audio recording timer.")
        self._listening_timer = self.create_timer(AUDIO_POLL_PERIOD_SECONDS, self._poll_audio_sample)

    def _stop_listening_timer(self) -> None:
        self.get_logger().info("Stopping audio recording timer.")
        if self._listening_timer is not None:
            self._listening_timer.cancel()
            self._listening_timer = None

    def _poll_audio_sample(self) -> None:
        if self._mini is None:
            return
        try:
            sample: npt.NDArray[np.float32] = self._mini.media.get_audio_sample()
            if sample is not None:
                self._audio_buffer.append(sample)
        except Exception as e:
            self.get_logger().error(f"Failed to get audio sample: {e}")

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

    def on_shutdown(self) -> None:
        """Clean up reachy node resources."""
        self.get_logger().info("Reachy node shutting down.")
        self._disconnect()
        super().on_shutdown()

    def _joystick_button_callback(self, msg: JoystickButtonEvent) -> None:
        if msg.button == JoystickButton.BUTTON_SELECT.value:
            self._connect()
            return
        elif msg.button == JoystickButton.BUTTON_START.value:
            self._disconnect()
            return
        elif msg.button == JoystickButton.BUTTON_Y.value:
            self._start_recording()
            return
        elif msg.button == JoystickButton.BUTTON_X.value:
            self._stop_recording()
            return

    def _joystick_axis_callback(self, msg: JoystickAxisEvent) -> None:
        if msg.axis == JoystickAxis.AXIS_LEFT.value:
            self._do_random_emotion()
        elif msg.axis == JoystickAxis.AXIS_RIGHT.value:
            self._do_random_dance()
        elif msg.axis == JoystickAxis.AXIS_UP.value:
            self._take_photo()
        elif msg.axis == JoystickAxis.AXIS_DOWN.value:
            self._play_audio()

    def _do_random_emotion(self) -> None:
        random_emotion_id = random.choice(self._recorded_emotions.list_moves())
        random_emotion: RecordedMove = self._recorded_emotions.get(random_emotion_id)
        self.get_logger().info(f"Playing random emotion ({random_emotion_id}): {random_emotion.description}")
        self._do_random_move(random_emotion_id, random_emotion)

    def _do_random_dance(self) -> None:
        random_dance_id = random.choice(self._recorded_dances.list_moves())
        random_dance: RecordedMove = self._recorded_dances.get(random_dance_id)
        self.get_logger().info(f"Playing random dance ({random_dance_id}): {random_dance.description}")
        self._do_random_move(random_dance_id, random_dance)

    def _do_random_move(self, move_id: str, move: RecordedMove) -> None:
        if self._mini is None:
            self.get_logger().warning("Reachy Mini not connected.")
            return
        if self._is_moving:
            self.get_logger().warning("Already moving.")
            return
        self._is_moving = True
        future = asyncio.run_coroutine_threadsafe(self._mini.async_play_move(move=move, sound=True), self._core_loop)
        future.add_done_callback(lambda f: self._on_move_played(f, move_id))

    def _on_move_played(self, future: Future, move_id: str) -> None:
        try:
            future.result()  # Raises exception if the coroutine failed
            self.get_logger().info(f"Move '{move_id}' played.")
        except Exception as e:
            self.get_logger().error(f"Move '{move_id}' failed: {e}")
        finally:
            self._is_moving = False

    def _take_photo(self) -> None:
        if self._mini is None:
            self.get_logger().warning("Reachy Mini not connected.")
            return

        frame: Optional[npt.NDArray[np.uint8]] = self._mini.media.get_frame()
        if frame is None:
            self.get_logger().warning("Failed to capture frame: camera not available.")
            return

        try:
            filepath = create_timestamped_filename("reachy_photo", "png")
            cv2.imwrite(str(filepath), frame)
            self.get_logger().info(f"Photo saved to {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save photo: {e}")

    def _play_audio(self) -> None:
        if self._mini is None:
            self.get_logger().warning("Reachy Mini not connected.")
            return
        if self._is_talking:
            self.get_logger().warning("Already talking.")
            return

        sound_file = "/home/antonio/code/zugaldia/continuum/assets/audio/jfk.wav"
        data, sample_rate_file = sf.read(sound_file, dtype="float32")
        sample_rate_output = self._mini.media.get_output_audio_samplerate()  # 16000
        if sample_rate_file != sample_rate_output:
            self.get_logger().warning(f"Sampling mismatch: {sample_rate_file} != {sample_rate_output}.")
            return
        if data.ndim > 1:
            self.get_logger().warning("Audio is multichannel, mono is expected.")
            return

        duration = len(data) / sample_rate_output
        self.get_logger().info(f"Playing {duration} seconds of audio from: {sound_file}")

        self._is_talking = True
        self._mini.media.start_playing()
        for i in range(0, len(data), AUDIO_PLAYBACK_CHUNK_SIZE):
            chunk = data[i : i + AUDIO_PLAYBACK_CHUNK_SIZE]
            self._mini.media.push_audio_sample(chunk)

        # We need to wait for the duration of the audio before calling stop_playing.
        # Otherwise, playback will be interrupted.
        self._playback_timer = self.create_timer(duration, self._finish_audio_playback)

    def _finish_audio_playback(self) -> None:
        """Finish audio playback and clean up the one-shot timer."""
        if self._playback_timer is not None:
            self._playback_timer.cancel()
            self._playback_timer = None
        if self._mini is not None:
            self._mini.media.stop_playing()
            self.get_logger().info("Audio played.")
        self._is_talking = False

    def _start_recording(self) -> None:
        if self._is_recording:
            self.get_logger().warning("Already recording.")
            return

        # We are always listening. Start recording simply clears the buffer so that we only capture fresh samples
        # for the given start-stop period.
        self.get_logger().info("Starting recording.")
        self._audio_buffer.clear()
        self._is_recording = True

    def _stop_recording(self) -> None:
        if self._mini is None:
            self.get_logger().warning("Reachy Mini not connected.")
            return
        if not self._is_recording:
            self.get_logger().warning("Not currently recording.")
            return

        self.get_logger().info("Stopping recording.")
        self._is_recording = False
        if len(self._audio_buffer) == 0:
            self.get_logger().warning("No audio samples collected.")
            return

        channels_input = self._mini.media.get_input_channels()  # 2
        sample_rate_input = self._mini.media.get_input_audio_samplerate()  # 16000
        self.get_logger().info(f"Audio recording: {channels_input} channels, {sample_rate_input} Hz")

        # See: https://github.com/pollen-robotics/reachy_mini/blob/develop/examples/debug/sound_record.py
        audio_data = np.concatenate(list(self._audio_buffer), axis=0)
        output_path = create_timestamped_filename("reachy_audio", "wav")
        sf.write(str(output_path), audio_data, sample_rate_input)
        self.get_logger().info(f"Audio saved to {output_path}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            reachy_node = ReachyNode()
            rclpy.spin(reachy_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
