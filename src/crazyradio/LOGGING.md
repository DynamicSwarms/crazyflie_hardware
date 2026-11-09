# Radio transaction logging

The USB driver uses **inline mode 2**, requiring Crazyradio 2 firmware **5.3 or
newer**. The extra RSSI byte is parsed separately from the radio payload.

Logging is enabled by the existing `log_enabled` parameter (default `true`).
Start the normal hardware launch. Each Crazyradio process prints its log folder:

```
~/.ros/log/crazyradio/<unix-microseconds>-<pid>/
  crazyradio_log_<channel>.log
  crazyradio_stats_<channel>.csv
```

`ROS_LOG_DIR`, when set, replaces `~/.ros/log`. Separate folders preserve logs
across launches. The existing packet-log format is unchanged.

The CSV records **every completed USB transaction**, including SafeLink
negotiation, failed radio exchanges, and broadcasts. Columns include:

- `timestamp_us`: ROS clock microseconds (Unix time under the normal system clock).
- `steady_us`: monotonic host time, for intervals unaffected by wall-clock changes.
- `radio_channel`, `address_hex`, `drone_id`: full destination address as well as
  its final byte. Use `ack_enabled=0` to distinguish broadcasts; do not exclude a
  drone solely because its ID matches the broadcast address suffix.
- `datarate`: firmware encoding (`0` = 250 kbps, `1` = 1 Mbps, `2` = 2 Mbps).
- `crtp_port`, `crtp_channel`, `tx_hex`: actual transmitted message, including
  SafeLink negotiation. `tx_hex` retains the SafeLink sequence bits.
- `ack_received`, `retries`, `weak_signal`, `invalid_settings`: firmware status.
- `rssi_dbm`: numeric received-ACK RSSI, negative dBm. Blank for no ACK,
  broadcasts, malformed replies, or invalid settings. Never interpret blank as
  zero dBm. `raw_rssi` retains the unsigned wire value for debugging.
- `malformed`, `usb_declared_bytes`, `usb_received_bytes`: USB frame validation.
- `ack_payload_bytes`, `ack_hex`: radio ACK payload, before SafeLink processing.
- `transfer_us`: host duration of the USB OUT and IN operations; this is not a
  firmware RF-only latency measurement.

CSV output is buffered and flushed every second, on a malformed reply, and on
normal shutdown. A hard crash can lose the last buffered records. USB transfer
errors that throw before completion are reported by the driver, not as CSV rows.
The ACK retry count is hardware retries, separate from later host retries.

Protocol reference:
https://www.bitcraze.io/documentation/repository/crazyradio2-firmware/main/development/compatible-usb-protocol/
