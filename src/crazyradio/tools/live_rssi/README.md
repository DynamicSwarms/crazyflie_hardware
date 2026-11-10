# Live RSSI

Run on the ROS PC with Python 3; no extra packages or ROS nodes required:

```bash
python3 src/crazyradio/tools/live_rssi/server.py --log-dir /home/winni/.ros/log
```

Open `http://<PC-LAN-IP>:8765/` on your phone on the same network.

The page shows the 40 fixed Crazyflie positions from the recorded tracking
broadcasts. Each square displays its drone ID and RSSI in dBm. The large number
is the weakest currently measured RSSI. Grey cells and the missing-reply list
mean no received ACK in the last three seconds. Displayed RSSI is the median
over those three seconds; the page refreshes once per second.

The server tails the newest USB `radio.log` and `packets.log` for each channel,
joining transfer IDs to obtain full addresses, and follows new radio processes
automatically. It ignores broadcasts and invalid/malformed
replies. Old captures are not presented as live data. It reads log files only;
it sends no radio commands and starts no ROS nodes.

Use `--bind`, `--port`, or `--positions` to override the defaults. `positions.json`
contains fixed world coordinates; edit it if the drones move to new locations.
The page contains no external libraries or assets and works without internet.
