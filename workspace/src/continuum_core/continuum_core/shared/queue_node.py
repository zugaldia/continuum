"""

This implementation could be compared to a simple action server where streaming == feedback, with additional support
for queues + asyncio. The reason we didn't go with the action server APIs under the hood was that (1) besides the
complexities it would add to WebSocket consumers, (2) we eventually want to include the ability for streaming
*input* (e.g. a mic input), which is something action servers do not support, and we would need to implement manually
anyway (using a custom pubsub mechanism, for example).

"""

import asyncio
import collections
import threading
from abc import abstractmethod, ABC
from concurrent.futures import Future

from continuum.models import ContinuumRequest, ContinuumExecutor, ContinuumResponse
from continuum_core.shared.async_node import AsyncNode

# The number of jobs this client can take in parallel. Generally speaking, cloud clients can take a higher number
# and locally running models, the opposite. Default is 1 (serial execution).
MAX_CONCURRENT_REQUESTS = 1

# Implements backpressure by limiting queue size to prevent unbounded growth. The system can track
# MAX_CONCURRENT_REQUESTS (active) + MAX_REQUEST_QUEUE_SIZE (queued) jobs, requests beyond that are rejected.
MAX_REQUEST_QUEUE_SIZE = 10


class QueueNode(AsyncNode, ABC):
    """Base class for all Continuum nodes that need access to the asyncio loop, with a queueing mechanism."""

    _executor: ContinuumExecutor

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._request_queue: collections.deque[ContinuumRequest] = collections.deque()
        self._request_queue_lock: threading.Lock = threading.Lock()
        self._current_requests: list[ContinuumRequest] = []

    @abstractmethod
    def handle_result(self, future: Future[ContinuumResponse], sdk_request: ContinuumRequest) -> None:
        """Implemented by each subsystem to support their own request-response types."""
        pass

    @abstractmethod
    def handle_streaming_result(self, streaming_response: ContinuumResponse, sdk_request: ContinuumRequest) -> None:
        """Implemented by each subsystem to support their own request-response types."""
        pass

    def _queue_request(self, sdk_request: ContinuumRequest) -> None:
        def callback(streaming_response):
            return self.handle_streaming_result(streaming_response, sdk_request)

        future = asyncio.run_coroutine_threadsafe(self._executor.execute_request(sdk_request, callback), self._core_loop)
        future.add_done_callback(lambda f: self.handle_result(f, sdk_request))

    def receive_request(self, sdk_request: ContinuumRequest) -> None:
        with self._request_queue_lock:
            # Check if we can process immediately (have capacity)
            if len(self._current_requests) < MAX_CONCURRENT_REQUESTS:
                self._current_requests.append(sdk_request)
                self.get_logger().info(
                    f"Processing request immediately. Active: {len(self._current_requests)}/{MAX_CONCURRENT_REQUESTS}"
                )
                self._queue_request(sdk_request)
            # Check if we can queue the request
            elif len(self._request_queue) < MAX_REQUEST_QUEUE_SIZE:
                self._request_queue.append(sdk_request)
                self.get_logger().info(
                    f"Request queued. Queue size: {len(self._request_queue)}/{MAX_REQUEST_QUEUE_SIZE}"
                )
            # Queue is full, discard the request
            else:
                self.get_logger().error(
                    f"Request queue full ({MAX_REQUEST_QUEUE_SIZE}). Discarding request: {sdk_request}"
                )

    def manage_queue(self, sdk_request: ContinuumRequest) -> None:
        with self._request_queue_lock:
            # Remove completed request from active list
            self._current_requests.remove(sdk_request)
            self.get_logger().info(
                f"Request completed. Active: {len(self._current_requests)}/{MAX_CONCURRENT_REQUESTS}"
            )

            # Process queued requests if we have capacity
            while len(self._current_requests) < MAX_CONCURRENT_REQUESTS and self._request_queue:
                next_request = self._request_queue.popleft()
                self._current_requests.append(next_request)
                self._queue_request(next_request)
                self.get_logger().info(
                    f"Starting queued request. "
                    f"Active: {len(self._current_requests)}/{MAX_CONCURRENT_REQUESTS}, "
                    f"Queue: {len(self._request_queue)}/{MAX_REQUEST_QUEUE_SIZE}"
                )
