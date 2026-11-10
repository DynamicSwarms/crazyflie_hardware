#!/usr/bin/env python3
"""Small read-only LAN viewer for the Crazyradio text logs."""

import argparse
import json
import math
import statistics
import threading
import time
from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class LogTail:
    def __init__(self, path):
        self.path = Path(path)
        self.offset = 0
        self.pending = b""

    def read(self):
        with self.path.open("rb") as stream:
            size = self.path.stat().st_size
            if size < self.offset:
                self.offset = 0
                self.pending = b""
            if self.offset == 0 and size > 512 * 1024:
                stream.seek(size - 512 * 1024)
                stream.readline()
                self.offset = stream.tell()
            stream.seek(self.offset)
            chunk = stream.read()
            self.offset = stream.tell()
        data = self.pending + chunk
        if b"\n" not in data:
            self.pending = data
            return []
        complete, _, self.pending = data.rpartition(b"\n")
        return [line.split() for line in complete.decode().splitlines()
                if line.startswith("[")]


class TransferTail:
    def __init__(self, path):
        self.path = Path(path)
        self.packets = LogTail(self.path.with_name("packets.log"))
        self.radio = LogTail(self.path)
        self.addresses = {}
        self.waiting = []

    def read(self):
        for row in self.packets.read():
            if len(row) >= 3:
                self.addresses[row[1]] = row[2]
        self.waiting.extend(self.radio.read())
        result, waiting = [], []
        for row in self.waiting:
            if len(row) != 7:
                continue  # UDP uses different columns.
            address = self.addresses.get(row[1])
            if address is None:
                waiting.append(row)
                continue
            result.append({
                "timestamp": row[0].strip("[]"), "drone_id": int(address, 16) & 0xff,
                "ack_received": row[2], "rssi_dbm": row[4],
                "invalid_settings": row[5], "usb_reply_malformed": row[6],
            })
        self.waiting = waiting[-8192:]
        if len(self.addresses) > 16384:
            self.addresses = dict(list(self.addresses.items())[-8192:])
        return result


class LiveRssi:
    def __init__(self, log_dir, positions, window=3.0):
        self.log_dir = Path(log_dir)
        self.positions = positions
        self.window = window
        self.tails = {}
        self.samples = defaultdict(lambda: deque(maxlen=512))
        self.lock = threading.Lock()

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            newest = {}
            for path in (self.log_dir / "crazyradio").glob("*/radio.log"):
                try:
                    channel = int(path.parent.name.rsplit("-", 1)[1])
                    started = int(path.parent.name.split("-", 1)[0])
                except ValueError:
                    continue
                if channel not in newest or started > newest[channel][0]:
                    newest[channel] = (started, path)
            for channel, (_, path) in newest.items():
                if channel not in self.tails or self.tails[channel].path != path:
                    self.tails[channel] = TransferTail(path)
                    for key in list(self.samples):
                        if key[0] == channel:
                            del self.samples[key]
                try:
                    rows = self.tails[channel].read()
                except (OSError, UnicodeError, ValueError):
                    continue
                for row in rows:
                    try:
                        if row["ack_received"] != "1":
                            continue
                        if row["usb_reply_malformed"] != "0" or row["invalid_settings"] != "0":
                            continue
                        stamp = float(row["timestamp"])
                        rssi = float(row["rssi_dbm"])
                        drone = int(row["drone_id"])
                        if math.isfinite(rssi) and now - self.window <= stamp <= now + 0.5:
                            self.samples[channel, drone].append((stamp, rssi))
                    except (KeyError, TypeError, ValueError):
                        continue
            drones = []
            for position in self.positions:
                queue = self.samples[position["channel"], position["id"]]
                # Use timestamps, rather than a fixed number of packets.
                recent = [(stamp, value) for stamp, value in queue
                          if now - self.window <= stamp <= now + 0.5]
                drones.append({
                    "id": position["id"],
                    "channel": position["channel"],
                    "rssi": round(statistics.median(value for _, value in recent), 1)
                    if recent else None,
                })
            fresh = [drone for drone in drones if drone["rssi"] is not None]
            return {
                "drones": drones,
                "worst": min(fresh, key=lambda drone: drone["rssi"]) if fresh else None,
                "missing": [drone["id"] for drone in drones if drone["rssi"] is None],
                "window_seconds": self.window,
            }


def render_page(positions):
    xs = [p["x"] for p in positions]
    ys = [p["y"] for p in positions]
    scale = 450 / max(max(xs) - min(xs), 0.1)
    height = (max(ys) - min(ys)) * scale + 80
    cells = []
    for p in positions:
        x = 65 + (p["x"] - min(xs)) * scale
        y = 40 + (max(ys) - p["y"]) * scale
        cells.append(
            f'<g id="d{p["id"]}" transform="translate({x:.2f},{y:.2f})">'
            '<rect x="-28" y="-27" width="56" height="54" rx="7" fill="#cbd5e1"/>'
            f'<text y="-7" class="id">{p["id"]}</text>'
            '<text y="15" class="rssi">—</text></g>'
        )
    return (Path(__file__).with_name("index.html").read_text()
            .replace("__HEIGHT__", str(round(height)))
            .replace("__CELLS__", "".join(cells)))


def make_handler(tracker, page):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                body, content_type = page.encode(), "text/html; charset=utf-8"
            elif path == "/api/rssi":
                body = json.dumps(tracker.snapshot(), allow_nan=False).encode()
                content_type = "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--log-dir", default=str(Path.home() / ".ros/log"))
    parser.add_argument("--positions", type=Path,
                        default=Path(__file__).with_name("positions.json"))
    args = parser.parse_args()
    positions = json.loads(args.positions.read_text())
    tracker = LiveRssi(args.log_dir, positions)
    server = ThreadingHTTPServer((args.bind, args.port),
                                 make_handler(tracker, render_page(positions)))
    print(f"Live RSSI: http://{args.bind}:{args.port} (phone: use this PC's LAN IP)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
