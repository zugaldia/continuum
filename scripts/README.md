# Scripts

## baseline.py

(Sample real results in `baseline.txt`.)

Performance testing script that measures round-trip latency for the Continuum echo service through the full
WebSocket/rosbridge stack. Sends configurable echo requests at random intervals, discards warmup requests, and
computes statistical measurements including P50, P90, P95, and P99 percentiles.

**Current baseline results**: Mean latency of **5.73ms** (P95: 11.24ms, P99: 11.54ms) for the complete
request-response cycle through the Python client → WebSocket → rosbridge_server → ROS2 pub/sub → echo node → back.

Most of this latency comes from the WebSocket bridge rather than ROS2 itself (native ROS2 pub/sub is typically <1ms?).
For voice/LLM applications (where processing typically takes ~ 100ms-2000ms+), this 5-12ms overhead represents
~0.3-6% of total latency. Evaluate whether this latency profile is acceptable for your specific use case.

### Future work

- **Foxglove Bridge integration**: Evaluate migrating from rosbridge / also supporting the Foxglove Bridge for
potentially lower latency (~36% fewer dropped messages according to some benchmarks). Note that no official Python
client SDK is currently available, requiring custom implementation of the Foxglove WebSocket protocol.

- **QoS policy optimization**: Explore different [Quality of Service settings](https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
to optimize latency, reliability, and throughput characteristics for voice/LLM workloads.
