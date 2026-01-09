"""Base class for Mic nodes."""

from abc import ABC
from typing import Optional

from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields

from continuum.constants import (
    TOPIC_MIC_RESPONSE,
    TOPIC_MIC_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_MIC_REQUEST,
    ERROR_CODE_SUCCESS,
    ERROR_CODE_UNEXPECTED,
)
from continuum.mic import MicInterface, MicAction, ContinuumMicResponse, ContinuumMicStreamingResponse, MicState
from continuum_core.shared.base_node import BaseNode
from continuum_interfaces.msg import MicResponse, MicRequest, MicStreamingResponse


class BaseMicNode(BaseNode, ABC):
    """Base class for all Mic nodes in Continuum."""

    _mic_interface: MicInterface
    _mic_publisher: Publisher[MicResponse]
    _mic_streaming_publisher: Publisher[MicStreamingResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    #
    # Base node methods
    #

    def register_publishers(self) -> None:
        """Register the microphone response publishers."""
        self._mic_publisher = self.create_publisher(MicResponse, TOPIC_MIC_RESPONSE, QOS_DEPTH_DEFAULT)
        self._mic_streaming_publisher = self.create_publisher(
            MicStreamingResponse, TOPIC_MIC_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

    def register_subscribers(self) -> None:
        """Register the microphone request subscriber."""
        self.create_subscription(MicRequest, TOPIC_MIC_REQUEST, self._on_mic_request, QOS_DEPTH_DEFAULT)

    def _on_mic_request(self, request: MicRequest) -> None:
        """Handle incoming microphone requests."""
        self.get_logger().info(f"Mic request: {request}")
        response: Optional[ContinuumMicResponse] = None
        if request.action == MicAction.START:
            response = self._start(request.session_id)
        elif request.action == MicAction.STOP:
            response = self._stop()
        elif request.action == MicAction.MUTE:
            response = self._mute()
        elif request.action == MicAction.UNMUTE:
            response = self._unmute()
        else:
            self.get_logger().error(f"Unknown mic action: {request.action}")
            return
        if response is not None:
            self.publish_mic_response(response)

    def _start(self, session_id: str) -> ContinuumMicResponse:
        # We can only start from an IDLE state
        if self._mic_interface.state != MicState.IDLE:
            return ContinuumMicResponse(
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="Mic is already started.",
                state=self._mic_interface.state,
            )
        self._mic_interface.start(session_id=session_id)
        self._mic_interface.state = MicState.STARTED
        return ContinuumMicResponse(state=self._mic_interface.state)

    def _stop(self) -> ContinuumMicResponse:
        # We can only stop from a non-IDLE state (i.e., STARTED, STARTED_MUTED)
        if self._mic_interface.state == MicState.IDLE:
            return ContinuumMicResponse(
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="Mic is already stopped.",
                state=self._mic_interface.state,
            )
        audio_component = self._mic_interface.stop()
        self._mic_interface.state = MicState.IDLE
        return ContinuumMicResponse(
            state=self._mic_interface.state,
            audio_data=audio_component.audio_data,
            format=audio_component.format,
            channels=audio_component.channels,
            sample_rate=audio_component.sample_rate,
            sample_width=audio_component.sample_width,
        )

    def _mute(self) -> ContinuumMicResponse:
        # We can only mute from a STARTED state
        if self._mic_interface.state == MicState.STARTED_MUTED:
            return ContinuumMicResponse(
                error_code=ERROR_CODE_UNEXPECTED, error_message="Mic is already muted.", state=self._mic_interface.state
            )
        if self._mic_interface.state == MicState.IDLE:
            return ContinuumMicResponse(
                error_code=ERROR_CODE_UNEXPECTED, error_message="Mic is stopped.", state=self._mic_interface.state
            )
        self._mic_interface.mute()
        self._mic_interface.state = MicState.STARTED_MUTED
        return ContinuumMicResponse(state=self._mic_interface.state)

    def _unmute(self) -> ContinuumMicResponse:
        # We can only unmute from a STARTED_MUTED state
        if self._mic_interface.state == MicState.STARTED:
            return ContinuumMicResponse(
                error_code=ERROR_CODE_UNEXPECTED, error_message="Mic is not muted.", state=self._mic_interface.state
            )
        if self._mic_interface.state == MicState.IDLE:
            return ContinuumMicResponse(
                error_code=ERROR_CODE_UNEXPECTED, error_message="Mic is stopped.", state=self._mic_interface.state
            )
        self._mic_interface.unmute()
        self._mic_interface.state = MicState.STARTED
        return ContinuumMicResponse(state=self._mic_interface.state)

    #
    # Custom publishing methods
    #

    def publish_mic_response(self, sdk_response: ContinuumMicResponse) -> None:
        response = MicResponse()
        set_message_fields(response, sdk_response.model_dump())
        if sdk_response.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"Microphone response error: {sdk_response}")
        elif self.debug_mode:
            self._log_message_redacted(response)
        self._mic_publisher.publish(response)

    def publish_mic_streaming_response(self, sdk_streaming_response: ContinuumMicStreamingResponse) -> None:
        self.get_logger().debug(f"Microphone streaming response: {sdk_streaming_response}")
        response = MicStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self._mic_streaming_publisher.publish(response)
