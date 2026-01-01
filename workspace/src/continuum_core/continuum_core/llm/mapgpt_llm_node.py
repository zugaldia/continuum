"""

MapGPT is Mapbox's location-intelligent AI assistant for conversational navigation.

IMPORTANT: This implementation is NOT an official Mapbox product and is provided for
demonstration purposes only. It is an independent integration with the MapGPT API
and is not endorsed, supported, or maintained by Mapbox.

MapGPT is currently available in Private Preview. For official access and support,
contact Mapbox at https://www.mapbox.com/mapgpt

"""

import concurrent.futures
import json
from typing import Any, Callable, Dict, Optional

import rclpy
import requests  # type: ignore[import-untyped]
import websocket
from pydantic import BaseModel
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException
from websocket import WebSocket, WebSocketApp

from continuum.constants import PARAM_MAPBOX_ACCESS_TOKEN, PARAM_MAPBOX_ACCESS_TOKEN_DEFAULT, ERROR_CODE_UNEXPECTED
from continuum.llm.models import ContinuumLlmRequest, ContinuumLlmStreamingResponse, ContinuumLlmResponse
from continuum.models import ContinuumExecutor
from continuum.utils import is_empty
from continuum_core.llm.base_llm_node import BaseLlmNode

MAPGPT_ENDPOINT_EVENTS = "wss://mapgpt-production-ws.mapbox.com"
MAPGPT_ENDPOINT_CONVERSATION = "https://mapgpt-production-api.mapbox.com"
MAPGPT_PING_INTERVAL = 60
MAPGPT_PING_PAYLOAD = "mapgpt"

SAMPLE_CONTEXT = {
    "user_context": {"lat": "38.897778", "lon": "-77.036389", "place_name": "The White House, Washington, DC"},
    "app_context": {"locale": "en-US"},
}


class MapGPTState(BaseModel):
    """Tracks the MapGPT session and state IDs."""

    # This is the Continuum session ID - that we use to track requests and responses across multiple components.
    session_id: Optional[str] = None

    # Confusing, but this is the MapGPT session ID - that is used by MapGPT to keep track of previous interactions.
    state_id: Optional[str] = None


class MapGPTLlmNode(BaseLlmNode, ContinuumExecutor):
    def __init__(self) -> None:
        super().__init__("mapgpt_llm_node")
        self.set_node_info(name="MapGPT LLM Node", description="Integration with Mapbox MapGPT")
        self._executor = self
        self._access_token = self._get_str_param(PARAM_MAPBOX_ACCESS_TOKEN)
        self._ws_app: Optional[WebSocketApp] = None
        self._thread_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._state = MapGPTState()
        self._setup_websocket()
        self.get_logger().info("MapGPT LLM node initialized.")

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_MAPBOX_ACCESS_TOKEN,
            PARAM_MAPBOX_ACCESS_TOKEN_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

    def _setup_websocket(self) -> None:
        self.get_logger().info(f"Connecting to {MAPGPT_ENDPOINT_EVENTS}...")
        header = {"Authorization": self._access_token}
        # websocket.enableTrace(traceable=self.debug_mode)
        self._ws_app = websocket.WebSocketApp(
            url=MAPGPT_ENDPOINT_EVENTS,
            header=header,
            on_open=self._on_open,
            on_reconnect=self._on_reconnect,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
            on_ping=self._on_ping,
            on_pong=self._on_pong,
            on_cont_message=self._on_cont_message,
            on_data=self._on_data,
        )

        # Set a ping interval (in seconds) to keep the connection alive
        self._thread_executor.submit(
            self._ws_app.run_forever,
            ping_interval=MAPGPT_PING_INTERVAL,
            ping_payload=MAPGPT_PING_PAYLOAD,
        )

    #
    # WebSocketApp event handlers
    #

    def _on_open(self, ws_app: WebSocket) -> None:
        self.get_logger().info("WebSocket connection opened, requesting session ID.")
        if self._ws_app:
            event = {"action": "get-session-id"}
            self._ws_app.send_text(json.dumps(event))

    def _on_reconnect(self, ws_app: WebSocket) -> None:
        self.get_logger().warning("WebSocket has been reconnected.")

    def _on_message(self, ws_app: WebSocketApp, message: str) -> None:
        try:
            snippet = "(empty message)" if is_empty(message) else f"{message[:50]}..."
            self.get_logger().info(f"Received message: {snippet}")
            parsed = json.loads(message)
            self._handle_message(parsed)
        except Exception as e:
            # Events larger than 4kb will be split into multiple websocket frames automatically. Clients should wait
            # for the FIN bit to be set in the last frame to stitch them all together to form the full event.
            self.get_logger().error(f"Error parsing message: {e}")

    def _on_error(self, ws_app: WebSocket, error: Exception) -> None:
        self.get_logger().error(f"Error: {error}")

    def _on_close(self, ws_app: WebSocket, close_status_code: int, close_msg: str) -> None:
        self.get_logger().warning(f"WebSocket closed ({close_status_code}): {close_msg}")

    def _on_ping(self, ws_app: WebSocket, data: str) -> None:
        self.get_logger().info(f"Ping: {data}")

    def _on_pong(self, ws_app: WebSocket, data: str) -> None:
        self.get_logger().info(f"Pong: {data}")

    def _on_cont_message(self, ws_app: WebSocket, message: str, continue_flag: int) -> None:
        snippet = "(empty message)" if is_empty(message) else f"{message[:50]}..."
        self.get_logger().debug(f"Continuation message ({continue_flag}): {snippet}")

    def _on_data(self, ws_app: WebSocket, data: str, data_type: int, continue_flag: int) -> None:
        snippet = "(empty data)" if is_empty(data) else f"{data[:50]}..."
        self.get_logger().debug(f"Data ({data_type}, {continue_flag}): {snippet}")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> ContinuumLlmResponse:
        payload = {
            "prompt": request.content_text,
            "context": SAMPLE_CONTEXT,
            "capabilities": [],
            "profile_id": "default",
        }

        self._state.session_id = request.session_id
        url = f"{MAPGPT_ENDPOINT_CONVERSATION}/v1/conversation/{self._state.state_id}"
        headers = {"Authorization": self._access_token}
        self._logger.info(f"Posting prompt to {url} with payload: {payload}")

        try:
            response = requests.post(url, json=payload, headers=headers)
            response.raise_for_status()
            response_code = response.status_code  # 204 (response.text is typically empty)
            chunk_prefix = response.headers.get("Chunk-Prefix")  # Used in the WS events (unused in this example)
            self._logger.info(f"Received response ({response_code}): {chunk_prefix}")
            return ContinuumLlmResponse(
                session_id=request.session_id, state_id=self._state.state_id, done_reason="queued"
            )
        except Exception as e:
            return ContinuumLlmResponse(
                session_id=request.session_id, error_code=ERROR_CODE_UNEXPECTED, error_message=str(e)
            )

    def _handle_message(self, message: Dict[str, Any]) -> None:
        action = message.get("action", "")
        if action == "start-session":
            self._state.state_id = message.get("body", {}).get("session_id")
            self.get_logger().info(f"MapGPT session started: {self._state.state_id}")
        elif action == "send-event":
            action_type = message.get("body", {}).get("type", "")
            if action_type == "conversation":
                # Events may arrive out of order i.e. not in the same order sent by the server. Each event has
                # an id field which can be used to establish correct order.
                order_id = message.get("body", {}).get("id", "")
                content = message.get("body", {}).get("data", {}).get("content", "")
                is_initial = message.get("body", {}).get("data", {}).get("initial", False)
                is_final = message.get("body", {}).get("data", {}).get("final", False)
                self.get_logger().info(f"MapGPT response ({order_id}/{is_initial}/{is_final}): {content}")
                self.publish_llm_streaming_response(
                    ContinuumLlmStreamingResponse(
                        session_id=self._state.session_id,
                        is_initial=is_initial,
                        is_final=is_final,
                        order_id=order_id,
                        content_text=content,
                    )
                )
            else:
                self.get_logger().warning(f"Ignoring non-conversation event: {action_type}")
        else:
            self.get_logger().warning(f"Unknown action received: {action}")

    def on_shutdown(self) -> None:
        """Clean up MapGPT node resources."""
        self.get_logger().info("MapGPT LLM node shutting down.")
        if self._ws_app:
            self._ws_app.close()
            self._ws_app = None
        if self._thread_executor:
            self.get_logger().info("Shutting down thread executor.")
            self._thread_executor.shutdown(wait=True)
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            mapgpt_llm_node = MapGPTLlmNode()
            rclpy.spin(mapgpt_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
