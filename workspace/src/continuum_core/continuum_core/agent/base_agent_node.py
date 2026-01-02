"""Base class for agent nodes."""

from abc import ABC
from concurrent.futures import Future

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.agent.models import ContinuumAgentRequest, ContinuumAgentResponse, ContinuumAgentStreamingResponse
from continuum.constants import (
    TOPIC_AGENT_RESPONSE,
    TOPIC_AGENT_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_AGENT_REQUEST,
    ERROR_CODE_UNEXPECTED,
    ERROR_CODE_SUCCESS,
    PARAM_AGENT_PROVIDER_NAME,
    PARAM_AGENT_PROVIDER_NAME_DEFAULT,
    PARAM_AGENT_MODEL_NAME,
    PARAM_AGENT_MODEL_NAME_DEFAULT,
    PARAM_AGENT_API_KEY,
    PARAM_AGENT_API_KEY_DEFAULT,
    PARAM_AGENT_BASE_URL,
    PARAM_AGENT_BASE_URL_DEFAULT,
    PARAM_AGENT_INSTRUCTIONS,
    PARAM_AGENT_INSTRUCTIONS_DEFAULT,
    PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL,
    PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL_DEFAULT,
    PARAM_AGENT_ENABLE_WEB_FETCH_TOOL,
    PARAM_AGENT_ENABLE_WEB_FETCH_TOOL_DEFAULT,
    PARAM_AGENT_ENABLE_MEMORY_TOOL,
    PARAM_AGENT_ENABLE_MEMORY_TOOL_DEFAULT,
    PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL,
    PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL_DEFAULT,
)
from continuum_core.shared.queue_node import QueueNode
from continuum_interfaces.msg import AgentResponse, AgentRequest, AgentStreamingResponse


class BaseAgentNode(QueueNode, ABC):
    """Base class for all agent nodes in Continuum."""

    _agent_publisher: Publisher[AgentResponse]
    _agent_streaming_publisher: Publisher[AgentStreamingResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    #
    # Base node methods
    #

    def register_parameters(self) -> None:
        """Register the agent node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_AGENT_PROVIDER_NAME,
            PARAM_AGENT_PROVIDER_NAME_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_AGENT_MODEL_NAME,
            PARAM_AGENT_MODEL_NAME_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_AGENT_API_KEY,
            PARAM_AGENT_API_KEY_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_AGENT_BASE_URL,
            PARAM_AGENT_BASE_URL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_AGENT_INSTRUCTIONS,
            PARAM_AGENT_INSTRUCTIONS_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL,
            PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )
        self.declare_parameter(
            PARAM_AGENT_ENABLE_WEB_FETCH_TOOL,
            PARAM_AGENT_ENABLE_WEB_FETCH_TOOL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )
        self.declare_parameter(
            PARAM_AGENT_ENABLE_MEMORY_TOOL,
            PARAM_AGENT_ENABLE_MEMORY_TOOL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )
        self.declare_parameter(
            PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL,
            PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )

    def register_publishers(self) -> None:
        """Register the agent response publishers."""
        self._agent_publisher = self.create_publisher(AgentResponse, TOPIC_AGENT_RESPONSE, QOS_DEPTH_DEFAULT)
        self._agent_streaming_publisher = self.create_publisher(
            AgentStreamingResponse, TOPIC_AGENT_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

    def register_subscribers(self) -> None:
        """Register the agent request subscriber."""
        self.create_subscription(AgentRequest, TOPIC_AGENT_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: AgentRequest) -> None:
        """Handle incoming agent requests."""
        self.get_logger().info(f"Agent request received with session_id: {msg.session_id}")
        sdk_request = ContinuumAgentRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    #
    # Queue node methods
    #

    def handle_result(self, future: Future[ContinuumAgentResponse], sdk_request: ContinuumAgentRequest) -> None:
        """Handle the result of an agent request."""
        try:
            sdk_response = future.result()
            self.publish_agent_response(sdk_response)
        except Exception as e:
            self.publish_agent_response(
                ContinuumAgentResponse(
                    session_id=sdk_request.session_id, error_code=ERROR_CODE_UNEXPECTED, error_message=str(e)
                )
            )
        finally:
            self.manage_queue(sdk_request)

    def handle_streaming_result(
        self, streaming_response: ContinuumAgentStreamingResponse, sdk_request: ContinuumAgentRequest
    ) -> None:
        """Handle a streaming result from an agent request."""
        self.publish_agent_streaming_response(streaming_response)

    #
    # Custom publishing methods
    #

    def publish_agent_response(self, sdk_response: ContinuumAgentResponse) -> None:
        if sdk_response.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"Agent response error: {sdk_response}")
        else:
            self.get_logger().info(f"Agent response: {sdk_response}")
        response = AgentResponse()
        set_message_fields(response, sdk_response.model_dump())
        self._agent_publisher.publish(response)

    def publish_agent_streaming_response(self, sdk_streaming_response: ContinuumAgentStreamingResponse) -> None:
        self.get_logger().debug(f"Agent streaming response: {sdk_streaming_response}")
        response = AgentStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self._agent_streaming_publisher.publish(response)

    #
    # Properties
    #

    @property
    def provider_name(self) -> str:
        """Get the provider name parameter."""
        return self._get_str_param(PARAM_AGENT_PROVIDER_NAME)

    @property
    def model_name(self) -> str:
        """Get the model name parameter."""
        return self._get_str_param(PARAM_AGENT_MODEL_NAME)

    @property
    def api_key(self) -> str:
        """Get the API key parameter."""
        return self._get_str_param(PARAM_AGENT_API_KEY)

    @property
    def base_url(self) -> str:
        """Get the base URL parameter."""
        return self._get_str_param(PARAM_AGENT_BASE_URL)

    @property
    def instructions(self) -> str:
        """Get the instructions parameter."""
        return self._get_str_param(PARAM_AGENT_INSTRUCTIONS)

    @property
    def enable_web_search_tool(self) -> bool:
        """Get the enable web search tool parameter."""
        return self._get_bool_param(PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL)

    @property
    def enable_web_fetch_tool(self) -> bool:
        """Get the enable web fetch tool parameter."""
        return self._get_bool_param(PARAM_AGENT_ENABLE_WEB_FETCH_TOOL)

    @property
    def enable_memory_tool(self) -> bool:
        """Get the enable memory tool parameter."""
        return self._get_bool_param(PARAM_AGENT_ENABLE_MEMORY_TOOL)

    @property
    def enable_file_search_tool(self) -> bool:
        """Get the enable file search tool parameter."""
        return self._get_bool_param(PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL)
