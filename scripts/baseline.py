#!/usr/bin/env python3
"""Baseline latency performance test for Continuum echo service."""

import asyncio
import logging
import random
import sys
import time
from dataclasses import dataclass
from typing import Dict, List

import numpy as np
import typer

from continuum.client.client import ContinuumClient
from continuum.models import EchoRequest, EchoResponse

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


@dataclass
class LatencyMeasurement:
    """Single latency measurement."""

    request_time: float
    response_time: float
    latency_ms: float
    session_id: str
    message: str


class EchoLatencyTester:
    """Test harness for measuring echo service latency."""

    def __init__(
        self,
        host: str,
        port: int,
        num_requests: int,
        warmup_requests: int,
        min_interval_ms: float,
        max_interval_ms: float,
    ):
        self._client = ContinuumClient(host=host, port=port)
        self._num_requests = num_requests
        self._warmup_requests = warmup_requests
        self._min_interval_ms = min_interval_ms
        self._max_interval_ms = max_interval_ms
        self._measurements: List[LatencyMeasurement] = []
        self._pending_responses: Dict[str, float] = {}

    async def connect(self) -> None:
        """Connect to WebSocket server."""
        logger.info(f"Connecting to {self._client._host}:{self._client._port}")
        await self._client.connect()
        logger.info("Connected")

    def disconnect(self) -> None:
        """Disconnect from the WebSocket server."""
        self._client.disconnect()
        logger.info("Disconnected")

    def _on_echo_response(self, response: EchoResponse) -> None:
        """Handle echo response."""
        response_time = time.time()
        session_id = response.session_id
        if session_id in self._pending_responses:
            request_time = self._pending_responses.pop(session_id)
            latency_ms = (response_time - request_time) * 1000
            measurement = LatencyMeasurement(
                request_time=request_time,
                response_time=response_time,
                latency_ms=latency_ms,
                session_id=session_id,
                message=response.message,
            )

            self._measurements.append(measurement)
            if len(self._measurements) % 10 == 0:
                logger.info(f"Progress: {len(self._measurements)}/{self._num_requests}")
        else:
            logger.warning(f"Unknown session_id: {session_id}")

    async def run_test(self) -> None:
        """Execute latency test."""
        logger.info(f"Starting test: {self._num_requests} requests ({self._warmup_requests} warmup)")
        self._client.subscribe_echo(self._on_echo_response)
        await asyncio.sleep(0.5)

        for i in range(self._num_requests):
            request = EchoRequest(message=f"Echo test message {i}")
            request_time = time.time()
            self._pending_responses[request.session_id] = request_time
            self._client.publish_echo(request)
            if i < self._num_requests - 1:
                interval = random.uniform(self._min_interval_ms, self._max_interval_ms) / 1000
                await asyncio.sleep(interval)

        timeout = 10
        logger.info(f"Waiting for responses (timeout: {timeout}s)")
        start_wait = time.time()
        while len(self._measurements) < self._num_requests:
            await asyncio.sleep(0.1)
            elapsed = time.time() - start_wait
            if elapsed > timeout:
                logger.warning(f"Timeout: {len(self._measurements)}/{self._num_requests} received")
                break

        logger.info(f"Test completed: {len(self._measurements)} responses")

    def compute_statistics(self) -> None:
        """Compute and print latency statistics."""
        if len(self._measurements) <= self._warmup_requests:
            logger.error("Not enough measurements")
            return

        valid_measurements = self._measurements[self._warmup_requests :]
        latencies = np.array([m.latency_ms for m in valid_measurements])
        if len(latencies) == 0:
            logger.error("No valid measurements after warmup")
            return

        stats = {
            "count": len(latencies),
            "min": np.min(latencies),
            "max": np.max(latencies),
            "mean": np.mean(latencies),
            "median": np.median(latencies),
            "std": np.std(latencies),
            "p50": np.percentile(latencies, 50),
            "p90": np.percentile(latencies, 90),
            "p95": np.percentile(latencies, 95),
            "p99": np.percentile(latencies, 99),
        }

        print("\n" + "=" * 70)
        print("LATENCY STATISTICS (excluding warmup)")
        print("=" * 70)
        print(f"Total requests:        {self._num_requests}")
        print(f"Warmup requests:       {self._warmup_requests}")
        print(f"Valid measurements:    {stats['count']}")
        print("-" * 70)
        print(f"Min:                   {stats['min']:.2f} ms")
        print(f"Max:                   {stats['max']:.2f} ms")
        print(f"Mean:                  {stats['mean']:.2f} ms")
        print(f"Median (P50):          {stats['median']:.2f} ms")
        print(f"Std Dev:               {stats['std']:.2f} ms")
        print("-" * 70)
        print(f"P90:                   {stats['p90']:.2f} ms")
        print(f"P95:                   {stats['p95']:.2f} ms")
        print(f"P99:                   {stats['p99']:.2f} ms")
        print("=" * 70 + "\n")


async def run_baseline_test(
    host: str,
    port: int,
    num_requests: int,
    warmup_requests: int,
    min_interval_ms: float,
    max_interval_ms: float,
):
    """Run a baseline test with the given parameters."""
    tester = EchoLatencyTester(
        host=host,
        port=port,
        num_requests=num_requests,
        warmup_requests=warmup_requests,
        min_interval_ms=min_interval_ms,
        max_interval_ms=max_interval_ms,
    )

    try:
        await tester.connect()
        await tester.run_test()
        tester.compute_statistics()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        sys.exit(1)
    finally:
        tester.disconnect()


def main(
    host: str = typer.Option("localhost", help="WebSocket server host"),
    port: int = typer.Option(9090, help="WebSocket server port"),
    num_requests: int = typer.Option(100, help="Total number of requests to send"),
    warmup_requests: int = typer.Option(10, help="Number of warmup requests to discard"),
    min_interval_ms: float = typer.Option(10.0, help="Minimum interval between requests (ms)"),
    max_interval_ms: float = typer.Option(100.0, help="Maximum interval between requests (ms)"),
):
    """Baseline performance test for Continuum echo service."""
    asyncio.run(
        run_baseline_test(
            host=host,
            port=port,
            num_requests=num_requests,
            warmup_requests=warmup_requests,
            min_interval_ms=min_interval_ms,
            max_interval_ms=max_interval_ms,
        )
    )


if __name__ == "__main__":
    typer.run(main)
