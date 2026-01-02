from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.agent import PydanticAgentRunner
from continuum.agent.models import PydanticAgentOptions
from continuum_core.agent.base_agent_node import BaseAgentNode


class PydanticAgentNode(BaseAgentNode):
    def __init__(self) -> None:
        super().__init__("pydantic_agent_node")
        self.set_node_info(name="Pydantic agent Node", description="Pydantic AI agent node.")

        # Read parameters and create options
        options = PydanticAgentOptions(
            provider_name=self.provider_name,
            model_name=self.model_name,
            api_key=self.api_key,
            base_url=self.base_url,
            instructions=self.instructions,
            enable_web_search_tool=self.enable_web_search_tool,
            enable_web_fetch_tool=self.enable_web_fetch_tool,
            enable_memory_tool=self.enable_memory_tool,
            enable_file_search_tool=self.enable_file_search_tool,
        )

        self._executor = PydanticAgentRunner(options=options)
        self.get_logger().info(f"Pydantic agent node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up pydantic agent node resources."""
        self.get_logger().info("Pydantic agent node shutting down.")
        self._executor.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            pydantic_agent_node = PydanticAgentNode()
            rclpy.spin(pydantic_agent_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
