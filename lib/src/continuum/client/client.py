"""

This is a thin wrapper around roslibpy using asyncio that provides strongly typed Pydantic-based callbacks
for ROS2 topics and messages.

For SOL/SOS Flatpak and/or Snapcraft distribution, we might need a different WebSocket library with fewer dependencies
for easier packaging. Similarly, we might need a lightweight integration for a TypeScript-based GNOME extension.

"""

import asyncio
import base64
import logging
from typing import Any, Callable, Dict, List, Optional, Protocol, Type, TypeVar

import roslibpy

from continuum.agent.models import (
    ContinuumAgentRequest,
    ContinuumAgentResponse,
    ContinuumAgentStreamingResponse,
)
from continuum.apps.models import (
    ContinuumDictationRequest,
    ContinuumDictationResponse,
    ContinuumDictationStreamingResponse,
)
from continuum.asr.models import (
    ContinuumAsrRequest,
    ContinuumAsrResponse,
    ContinuumAsrStreamingResponse,
)
from continuum.constants import (
    CONTINUUM_NAMESPACE,
    NODE_APP_DICTATION,
    PATH_AGENT,
    PATH_APP,
    PATH_ASR,
    PATH_LLM,
    PATH_TTS,
    PROFILE_LOCAL,
    TOPIC_AGENT_REQUEST,
    TOPIC_AGENT_RESPONSE,
    TOPIC_AGENT_STREAMING_RESPONSE,
    TOPIC_ASR_REQUEST,
    TOPIC_ASR_RESPONSE,
    TOPIC_ASR_STREAMING_RESPONSE,
    TOPIC_DICTATION_REQUEST,
    TOPIC_DICTATION_RESPONSE,
    TOPIC_DICTATION_STREAMING_RESPONSE,
    TOPIC_ECHO_REQUEST,
    TOPIC_ECHO_RESPONSE,
    TOPIC_HEARTBEAT,
    TOPIC_LLM_REQUEST,
    TOPIC_LLM_RESPONSE,
    TOPIC_LLM_STREAMING_RESPONSE,
    TOPIC_TTS_REQUEST,
    TOPIC_TTS_RESPONSE,
    TOPIC_TTS_STREAMING_RESPONSE,
)
from continuum.llm.models import (
    ContinuumLlmRequest,
    ContinuumLlmResponse,
    ContinuumLlmStreamingResponse,
)
from continuum.models import EchoRequest, EchoResponse
from continuum.tts.models import (
    ContinuumTtsRequest,
    ContinuumTtsResponse,
    ContinuumTtsStreamingResponse,
)


# Protocol for types that can be constructed from ROS messages
class FromRosProtocol(Protocol):
    @classmethod
    def from_ros(cls: Type["T"], msg: Dict[str, Any]) -> "T": ...


# Type variable for generic subscription callbacks
T = TypeVar("T", bound=FromRosProtocol)


class ContinuumException(Exception):
    pass


class ContinuumClient:
    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        """Initialize ContinuumClient with host and port configuration."""
        self._logger = logging.getLogger(__name__)
        self._host = host
        self._port = port
        self._ros: Optional[roslibpy.Ros] = None
        self._publishers: Dict[str, roslibpy.Topic] = {}
        self._listeners: Dict[str, roslibpy.Topic] = {}
        self._logger.info(f"Initialized with host={host} and port={port}.")

    async def connect(self) -> None:
        """Establish connection to the ROS bridge server."""
        if self._ros is not None and self._ros.is_connected:
            self._logger.warning("Already connected to ROS bridge server.")
            return

        self._ros = roslibpy.Ros(host=self._host, port=self._port)
        future: asyncio.Future[None] = asyncio.Future()
        loop = asyncio.get_running_loop()

        def on_ready():
            # Wrapping in a `call_soon_threadsafe` instead of simply calling
            # `future.set_result()` is needed in order to be able to play nice
            # with the underlying Twisted implementation. Otherwise, the
            # future would hang.
            loop.call_soon_threadsafe(future.set_result, None)

        self._ros.on_ready(on_ready)
        self._ros.run()
        await future

    def disconnect(self) -> None:
        """Disconnect from the ROS bridge server and cleanup resources."""
        if self._ros is None:
            self._logger.warning("Already disconnected.")
            return

        for publisher in self._publishers.values():
            publisher.unadvertise()
        self._publishers.clear()

        for listener in self._listeners.values():
            listener.unsubscribe()
        self._listeners.clear()

        self._ros.close()
        self._ros = None

    @property
    def is_connected(self) -> bool:
        """Check if the client is connected to the ROS bridge server."""
        return self._ros is not None and bool(self._ros.is_connected)

    #
    # General
    #

    async def get_topics(self) -> Optional[List[str]]:
        """Get the list of available ROS topics."""
        if not self.is_connected:
            self._logger.error("Cannot get topics: Disconnected.")
            return None

        assert self._ros is not None
        future: asyncio.Future[Optional[List[str]]] = asyncio.Future()
        loop = asyncio.get_running_loop()

        def on_topics(response: roslibpy.core.ServiceResponse):
            topics = response.get("topics", None)
            loop.call_soon_threadsafe(future.set_result, topics)

        def on_error(error):
            loop.call_soon_threadsafe(future.set_exception, ContinuumException(f"Failed to get topics: {error}"))

        self._ros.get_topics(on_topics, on_error)
        return await future

    async def get_params(self) -> Optional[List[str]]:
        """Get the list of available ROS parameters."""
        if not self.is_connected:
            self._logger.error("Cannot get parameters: Disconnected.")
            return None

        assert self._ros is not None
        future: asyncio.Future[Optional[List[str]]] = asyncio.Future()
        loop = asyncio.get_running_loop()

        def on_params(response: roslibpy.core.ServiceResponse):
            params = response.get("names", None)
            loop.call_soon_threadsafe(future.set_result, params)

        def on_error(error):
            loop.call_soon_threadsafe(future.set_exception, ContinuumException(f"Failed to get parameters: {error}"))

        self._ros.get_params(on_params, on_error)
        return await future

    #
    # Subscribers
    #

    def _subscribe_topic(self, name: str, message_type: str, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Subscribe to a topic with duplicate subscription prevention."""
        if not self.is_connected:
            self._logger.error("Cannot subscribe: Disconnected.")
            return None

        assert self._ros is not None
        if name in self._listeners:
            self._logger.warning(f"Ignoring, already subscribed to topic: {name}")
            return None

        listener = roslibpy.Topic(self._ros, name, message_type)
        listener.subscribe(callback)
        self._listeners[name] = listener
        return None

    def _subscribe_topic_typed(
        self, name: str, message_type: str, callback: Callable[[T], None], model_class: Type[T]
    ) -> None:
        """Subscribe to a topic with automatic Pydantic model conversion."""

        def wrapped_callback(msg: Dict[str, Any]) -> None:
            # rosbridge encodes uint8[] as base64 strings (AudioComponent)
            if "audio_data" in msg and isinstance(msg["audio_data"], str):
                audio_bytes = base64.b64decode(msg["audio_data"])
                msg["audio_data"] = list(audio_bytes)
            parsed = model_class.from_ros(msg)
            callback(parsed)

        self._subscribe_topic(name, message_type, wrapped_callback)

    def subscribe_diagnostics(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Subscribe to the continuum diagnostics topic."""
        name = f"/{CONTINUUM_NAMESPACE}/{TOPIC_HEARTBEAT}"
        message_type = "diagnostic_msgs/DiagnosticArray"
        self._subscribe_topic(name, message_type, callback)

    def subscribe_echo(self, callback: Callable[[EchoResponse], None]) -> None:
        """Subscribe to the continuum echo response topic."""
        name = f"/{CONTINUUM_NAMESPACE}/{TOPIC_ECHO_RESPONSE}"
        message_type = "continuum_interfaces/EchoResponse"
        self._subscribe_topic_typed(name, message_type, callback, EchoResponse)

    def subscribe_agent_response(self, node_name: str, callback: Callable[[ContinuumAgentResponse], None]) -> None:
        """Subscribe to the agent response topic for the specified agent node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_AGENT}/{node_name}/{TOPIC_AGENT_RESPONSE}"
        message_type = "continuum_interfaces/AgentResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumAgentResponse)

    def subscribe_agent_streaming_response(
        self, node_name: str, callback: Callable[[ContinuumAgentStreamingResponse], None]
    ) -> None:
        """Subscribe to the agent streaming response topic for the specified agent node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_AGENT}/{node_name}/{TOPIC_AGENT_STREAMING_RESPONSE}"
        message_type = "continuum_interfaces/AgentStreamingResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumAgentStreamingResponse)

    def subscribe_asr_response(self, node_name: str, callback: Callable[[ContinuumAsrResponse], None]) -> None:
        """Subscribe to the ASR response topic for the specified ASR node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{node_name}/{TOPIC_ASR_RESPONSE}"
        message_type = "continuum_interfaces/AsrResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumAsrResponse)

    def subscribe_asr_streaming_response(
        self, node_name: str, callback: Callable[[ContinuumAsrStreamingResponse], None]
    ) -> None:
        """Subscribe to the ASR streaming response topic for the specified ASR node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{node_name}/{TOPIC_ASR_STREAMING_RESPONSE}"
        message_type = "continuum_interfaces/AsrStreamingResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumAsrStreamingResponse)

    def subscribe_llm_response(self, node_name: str, callback: Callable[[ContinuumLlmResponse], None]) -> None:
        """Subscribe to the LLM response topic for the specified LLM node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{node_name}/{TOPIC_LLM_RESPONSE}"
        message_type = "continuum_interfaces/LlmResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumLlmResponse)

    def subscribe_llm_streaming_response(
        self, node_name: str, callback: Callable[[ContinuumLlmStreamingResponse], None]
    ) -> None:
        """Subscribe to the LLM streaming response topic for the specified LLM node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{node_name}/{TOPIC_LLM_STREAMING_RESPONSE}"
        message_type = "continuum_interfaces/LlmStreamingResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumLlmStreamingResponse)

    def subscribe_dictation_response(
        self, callback: Callable[[ContinuumDictationResponse], None], profile: str = PROFILE_LOCAL
    ) -> None:
        """Subscribe to the dictation response topic for the specified profile."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_APP}/{NODE_APP_DICTATION}/{profile}/{TOPIC_DICTATION_RESPONSE}"
        message_type = "continuum_interfaces/DictationResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumDictationResponse)

    def subscribe_dictation_streaming_response(
        self, callback: Callable[[ContinuumDictationStreamingResponse], None], profile: str = PROFILE_LOCAL
    ) -> None:
        """Subscribe to the dictation streaming response topic for the specified profile."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_APP}/{NODE_APP_DICTATION}/{profile}/{TOPIC_DICTATION_STREAMING_RESPONSE}"
        message_type = "continuum_interfaces/DictationStreamingResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumDictationStreamingResponse)

    def subscribe_tts_response(self, node_name: str, callback: Callable[[ContinuumTtsResponse], None]) -> None:
        """Subscribe to the TTS response topic for the specified TTS node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_TTS}/{node_name}/{TOPIC_TTS_RESPONSE}"
        message_type = "continuum_interfaces/TtsResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumTtsResponse)

    def subscribe_tts_streaming_response(
        self, node_name: str, callback: Callable[[ContinuumTtsStreamingResponse], None]
    ) -> None:
        """Subscribe to the TTS streaming response topic for the specified TTS node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_TTS}/{node_name}/{TOPIC_TTS_STREAMING_RESPONSE}"
        message_type = "continuum_interfaces/TtsStreamingResponse"
        self._subscribe_topic_typed(name, message_type, callback, ContinuumTtsStreamingResponse)

    #
    # Publishers
    #

    def _publish_message(self, name: str, message_type: str, message: roslibpy.Message) -> None:
        """Publish a message to the topic with publisher caching."""
        if not self.is_connected:
            self._logger.error("Cannot publish: Disconnected.")
            return None

        assert self._ros is not None
        if name not in self._publishers:
            self._publishers[name] = roslibpy.Topic(self._ros, name, message_type)
        publisher = self._publishers[name]
        publisher.publish(message)
        return None

    def publish_echo(self, request: EchoRequest) -> None:
        """Publish an echo request to the continuum echo request topic."""
        name = f"/{CONTINUUM_NAMESPACE}/{TOPIC_ECHO_REQUEST}"
        message_type = "continuum_interfaces/EchoRequest"
        message = roslibpy.Message(request.model_dump())
        self._publish_message(name, message_type, message)
        return None

    def publish_agent_request(self, node_name: str, request: ContinuumAgentRequest) -> None:
        """Publish an agent request to the specified agent node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_AGENT}/{node_name}/{TOPIC_AGENT_REQUEST}"
        message_type = "continuum_interfaces/AgentRequest"
        message = roslibpy.Message(request.model_dump())
        self._publish_message(name, message_type, message)
        return None

    def publish_asr_request(self, node_name: str, request: ContinuumAsrRequest) -> None:
        """Publish an ASR request to the specified ASR node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{node_name}/{TOPIC_ASR_REQUEST}"
        message_type = "continuum_interfaces/AsrRequest"
        message = roslibpy.Message(request.model_dump())
        self._publish_message(name, message_type, message)
        return None

    def publish_llm_request(self, node_name: str, request: ContinuumLlmRequest) -> None:
        """Publish an LLM request to the specified LLM node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{node_name}/{TOPIC_LLM_REQUEST}"
        message_type = "continuum_interfaces/LlmRequest"
        message = roslibpy.Message(request.model_dump())
        self._publish_message(name, message_type, message)
        return None

    def publish_dictation_request(self, request: ContinuumDictationRequest, profile: str = PROFILE_LOCAL) -> None:
        """Publish a dictation request to the dictation app for the specified profile."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_APP}/{NODE_APP_DICTATION}/{profile}/{TOPIC_DICTATION_REQUEST}"
        message_type = "continuum_interfaces/DictationRequest"
        message = roslibpy.Message(request.model_dump())
        self._publish_message(name, message_type, message)
        return None

    def publish_tts_request(self, node_name: str, request: ContinuumTtsRequest) -> None:
        """Publish a TTS request to the specified TTS node."""
        name = f"/{CONTINUUM_NAMESPACE}/{PATH_TTS}/{node_name}/{TOPIC_TTS_REQUEST}"
        message_type = "continuum_interfaces/TtsRequest"
        message = roslibpy.Message(request.model_dump())
        self._publish_message(name, message_type, message)
        return None
