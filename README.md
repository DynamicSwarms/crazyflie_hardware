# crazyflie impl for dynamic swarms

colcon build --packages-select crazyradio --cmake-args -DLEGACY_RADIO=OFF
## Radio diagnostics

Each Crazyradio node publishes one `diagnostic_msgs/msg/DiagnosticArray` entry
on `/diagnostics` every second. The same timer continues publishing the existing
`crazyradio/link_qualities` topic; diagnostics contain no per-link entries.

The entry includes channel and transport, unicast/broadcast link counts, average/
worst/best unicast link quality (percent; unweighted across links, N/A with no links), aggregate
outgoing packets and pending responses, accepted packets/s and cumulative accepted
packets, links in backoff, and software send rates. Total counters are since node
startup. Rates use the actual elapsed steady-clock interval. A send attempt is
one transport call, not a hardware retry; payload/polling partition attempts,
while broadcast attempts are a subset of payload attempts. Pending responses may
overlap outgoing packets until those packets have been successfully transmitted.
Accepted packets are counted once at container enqueue, including broadcasts but
excluding automatic polling and retries. This counter survives link removal and
uses uint64_t; rate calculation handles unsigned wrap between samples. An empty
outgoing queue at publication time can therefore still show nonzero input traffic.
No extra radio packets are sent to collect diagnostics.

`CrtpPacket` carries a host-side steady-clock creation timestamp, retained through
retries and never transmitted. Diagnostics report the oldest outgoing packet age
and the maximum completed delay since the last report, measured from creation to
`recordCommunication()` after a successful send and response handling.
Timing calculations and completed-delay peaks live in RadioDiagnostics. Link/container
snapshots contain only counts and the oldest queued creation timestamp; port queues
remain unchanged. Only FIFO queue fronts are inspected; packets awaiting an
application response after a successful send are excluded. Completed delay peaks survive link removal;
unsent packets discarded on removal are not counted as completed deliveries.
For broadcasts, success means transport send success, not confirmed reception.

Idle/low traffic and ordinary failed sends are OK. Outgoing age or completed delay
above `diagnostics_delivery_warn_ms` produces WARN (read-only, default 100 ms; 0 disables).
The warning clears on the next report with both values below the threshold. It
means outgoing delivery is delayed, which can also be caused by an unreachable
Crazyflie; it does not establish radio overload. Transport exceptions publish ERROR
before shutdown. A viewer must detect stale diagnostics if the node stops publishing.

Statistics snapshots are collected by the node; diagnostics receives only data and
the base/clock/parameters/topics node interfaces.
Diagnostics publishing, counters, and link-quality publishing live in
`radio_diagnostics.hpp` / `radio_diagnostics.cpp`, separately from the node.

Inspect with `ros2 topic echo /diagnostics` or `rqt_runtime_monitor` (no aggregator
required).
