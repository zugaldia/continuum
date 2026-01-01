import asyncio
from threading import Thread

from continuum_core.shared.base_node import BaseNode


class AsyncNode(BaseNode):
    """Base class for all Continuum nodes that need access to the asyncio loop."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._core_loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()
        self._core_thread: Thread = Thread(target=self._run_event_loop, daemon=True)
        self._core_thread.start()

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._core_loop)
        self._core_loop.run_forever()

    def on_shutdown(self) -> None:
        self._core_loop.call_soon_threadsafe(self._core_loop.stop)
        self._core_thread.join()
        super().on_shutdown()
