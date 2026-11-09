import csv
import tempfile
import unittest
from pathlib import Path

from server import CsvTail, LiveRssi, render_page


HEADER = ["timestamp_us", "drone_id", "ack_enabled", "ack_received",
          "malformed", "invalid_settings", "rssi_dbm"]


class LiveRssiTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.file = self.root / "crazyradio/100000000-1/crazyradio_stats_90.csv"
        self.file.parent.mkdir(parents=True)
        self.positions = [{"id": 210, "channel": 90, "x": 0, "y": 0},
                          {"id": 231, "channel": 90, "x": 1, "y": 0}]

    def tearDown(self):
        self.temp.cleanup()

    def write(self, path, rows):
        with path.open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(HEADER)
            writer.writerows(rows)

    def test_median_filters_and_staleness(self):
        self.write(self.file, [
            [98000000, 210, 1, 1, 0, 0, -60],
            [99000000, 210, 1, 1, 0, 0, -64],
            [99000000, 210, 1, 0, 0, 0, -90],  # No ACK: no RSSI measurement.
            [99000000, 231, 0, 1, 0, 0, -90],  # Broadcast, not drone 231.
            [99000000, 231, 1, 1, 0, 0, -40],  # Legitimate unicast drone 231.
            [99000000, 210, 1, 1, 1, 0, -95],  # Malformed reply.
        ])
        live = LiveRssi(self.root, self.positions)
        result = live.snapshot(now=100)
        self.assertEqual(result["worst"]["id"], 210)
        self.assertEqual(result["worst"]["rssi"], -62)
        self.assertEqual(result["missing"], [])
        result = live.snapshot(now=104)
        self.assertIsNone(result["worst"])
        self.assertEqual(result["missing"], [210, 231])

    def test_new_run_clears_old_measurements(self):
        self.write(self.file, [[99000000, 210, 1, 1, 0, 0, -65]])
        live = LiveRssi(self.root, self.positions)
        self.assertEqual(live.snapshot(now=100)["worst"]["rssi"], -65)
        newer = self.root / "crazyradio/100000001-2/crazyradio_stats_90.csv"
        newer.parent.mkdir()
        self.write(newer, [[100000000, 231, 1, 1, 0, 0, -45]])
        result = live.snapshot(now=101)
        self.assertEqual(result["missing"], [210])
        self.assertEqual(result["worst"]["id"], 231)

    def test_partial_rows_are_waited_for(self):
        self.file.write_text(",".join(HEADER) + "\n99000000,210,1,1,0,0,-6")
        tail = CsvTail(self.file)
        self.assertEqual(tail.read(), [])
        with self.file.open("a") as stream:
            stream.write("3\n")
        self.assertEqual(tail.read()[0]["rssi_dbm"], "-63")
        self.assertEqual(tail.read(), [])

    def test_grid_is_present_without_javascript(self):
        page = render_page(self.positions)
        self.assertIn('id="d210"', page)
        self.assertIn('id="d231"', page)
        self.assertNotIn("__CELLS__", page)
        self.assertNotIn("__HEIGHT__", page)


if __name__ == "__main__":
    unittest.main()
