"""Exercise the public Soapy API against a deterministic localhost receiver.
No radio hardware or RF transmission. Requires SoapySDR's Python bindings/numpy.
"""

import http.server
import json
import socket
import struct
import threading
import subprocess
import sys
import urllib.request
import unittest
import numpy as np
import SoapySDR as soapy


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self):
        super().__init__(("127.0.0.1", 0), Handler)
        self.state = dict(
            transport="idle",
            protocol=2,
            epoch=0,
            frequency=2412000000,
            rate=4000000,
            gain=40,
            gain_max=69,
            bandwidth=0,
            dc_correction=1,
            time_us=1000000,
        )
        self.bad_reply = None
        self.destination = None
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def emit(self, sample=0, epoch=None, malformed=False):
        data = struct.pack(
            "<4sHHIIQQ",
            b"FAIL" if malformed else b"ESR2",
            2,
            8,
            self.state["epoch"] if epoch is None else epoch,
            self.state["rate"],
            sample,
            1000000 + sample * 1000000 // self.state["rate"],
        )
        self.udp.sendto(data + bytes([128, 127]) * 672, self.destination)


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        if request["op"] == "bad_reply":
            self.server.bad_reply = request["operation"]
        if request["op"] in ("start", "configure", "stop"):
            self.server.state["epoch"] += 1
            for key in ("frequency", "rate", "gain", "bandwidth", "dc_correction"):
                if key in request:
                    self.server.state[key] = request[key]
        if request["op"] == "stop":
            self.server.state["transport"] = "idle"
        if request["op"] == "start":
            self.server.state["transport"] = "ethernet"
            self.server.destination = (self.client_address[0], request["port"])
        if request["op"] == "emit":
            self.server.emit(
                request.get("sample", 0),
                request.get("epoch"),
                request.get("malformed", False),
            )
        result = json.dumps(self.server.state).encode()
        if request["op"] == self.server.bad_reply:
            self.server.bad_reply = None
            result = b"{"  # Command applied, but its reply was truncated.
        self.send_response(200)
        self.send_header("Content-Length", str(len(result)))
        self.end_headers()
        self.wfile.write(result)


class Receiver:
    # A separate process is necessary: Soapy's SWIG calls may hold the GIL
    # while waiting for an HTTP control response.
    def __init__(self):
        self.process = subprocess.Popen(
            [sys.executable, __file__, "--server"], stdout=subprocess.PIPE, text=True
        )
        self.server_port = int(self.process.stdout.readline())

    def request(self, data):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.server_port}/api/rx",
            data=json.dumps(data).encode(),
        )
        with urllib.request.urlopen(req, timeout=5) as response:
            return json.load(response)

    @property
    def state(self):
        return self.request(dict(op="status"))

    def emit(self, sample=0, epoch=None, malformed=False):
        self.request(dict(op="emit", sample=sample, epoch=epoch, malformed=malformed))

    def finish(self):
        self.process.terminate()
        self.process.wait(timeout=5)
        self.process.stdout.close()


class Streaming(unittest.TestCase):
    def setUp(self):
        self.receiver = Receiver()
        self.device = soapy.Device(
            f"driver=espsdr,host=127.0.0.1,http_port={self.receiver.server_port}"
        )
        self.device.setDCOffsetMode(soapy.SOAPY_SDR_RX, 0, False)
        self.stream = None

    def tearDown(self):
        if self.stream:
            self.device.deactivateStream(self.stream)
            self.device.closeStream(self.stream)
        self.device = None
        self.receiver.finish()

    def open(self, fmt):
        self.stream = self.device.setupStream(soapy.SOAPY_SDR_RX, fmt)
        self.assertEqual(self.device.activateStream(self.stream), 0)

    def read(self, dtype, n=672):
        values = np.zeros(n * 2, dtype=dtype)
        result = self.device.readStream(self.stream, [values], n, timeoutUs=500000)
        return result, values

    def test_failed_start_reply_stops_receiver_and_allows_retry(self):
        self.stream = self.device.setupStream(soapy.SOAPY_SDR_RX, "CS8")
        self.receiver.request(dict(op="bad_reply", operation="start"))
        self.assertEqual(self.device.activateStream(self.stream), soapy.SOAPY_SDR_STREAM_ERROR)
        self.assertEqual(self.receiver.state["transport"], "idle")
        self.assertEqual(self.device.activateStream(self.stream), 0)
        self.receiver.emit()
        self.assertEqual(self.read(np.int8)[0].ret, 672)

    def test_failed_stop_reply_returns_error_and_allows_retry(self):
        self.open("CS8")
        self.receiver.request(dict(op="bad_reply", operation="stop"))
        self.assertEqual(self.device.deactivateStream(self.stream), soapy.SOAPY_SDR_STREAM_ERROR)
        self.assertEqual(self.receiver.state["transport"], "idle")
        self.assertEqual(self.device.activateStream(self.stream), 0)
        self.receiver.emit()
        self.assertEqual(self.read(np.int8)[0].ret, 672)

    def test_receive_only_and_formats(self):
        self.assertEqual(self.device.getNumChannels(soapy.SOAPY_SDR_RX), 1)
        self.assertEqual(self.device.getNumChannels(soapy.SOAPY_SDR_TX), 0)
        for fmt, dtype, low, high in [
            ("CS8", np.int8, -128, 127),
            ("CS16", np.int16, -32768, 32512),
            ("CF32", np.float32, -1, 127 / 128),
        ]:
            self.open(fmt)
            self.receiver.emit()
            result, values = self.read(dtype, 17)
            self.assertEqual(result.ret, 17)
            self.assertEqual(result.timeNs, 1000000000)
            self.assertTrue(result.flags & soapy.SOAPY_SDR_HAS_TIME)
            np.testing.assert_equal(values[::2], low)
            np.testing.assert_equal(values[1::2], high)
            result, _ = self.read(dtype, 655)
            self.assertEqual(result.ret, 655)
            self.assertEqual(result.timeNs, 1000004250)
            self.device.deactivateStream(self.stream)
            self.device.closeStream(self.stream)
            self.stream = None

    def test_dc_removal_and_bypass(self):
        self.assertTrue(self.device.hasDCOffsetMode(soapy.SOAPY_SDR_RX, 0))
        self.device.setDCOffsetMode(soapy.SOAPY_SDR_RX, 0, True)
        self.assertTrue(self.device.getDCOffsetMode(soapy.SOAPY_SDR_RX, 0))
        self.assertEqual(self.receiver.state["dc_correction"], 1)
        for fmt, dtype in [("CF32", np.float32), ("CS16", np.int16), ("CS8", np.int8)]:
            self.open(fmt)
            self.receiver.emit()
            result, values = self.read(dtype, 17)
            self.assertEqual(result.ret, 17)
            np.testing.assert_equal(values, 0)
            result, values = self.read(dtype, 655)
            self.assertEqual(result.ret, 655)
            np.testing.assert_equal(values, 0)
            self.device.setDCOffsetMode(soapy.SOAPY_SDR_RX, 0, False)
            self.assertEqual(self.receiver.state["dc_correction"], 0)
            self.receiver.emit()  # Hardware-mode change starts a fresh epoch.
            _, values = self.read(dtype)
            self.assertLess(values[0], 0)
            self.assertGreater(values[1], 0)
            self.device.deactivateStream(self.stream)
            self.device.closeStream(self.stream)
            self.stream = None
            self.device.setDCOffsetMode(soapy.SOAPY_SDR_RX, 0, True)

    def test_gap_duplicate_and_retune(self):
        self.open("CS8")
        self.receiver.emit()
        self.assertEqual(self.read(np.int8)[0].ret, 672)
        self.receiver.emit()  # Duplicate must not reach the application.
        self.receiver.emit(1344)  # One missing packet.
        self.assertEqual(self.read(np.int8)[0].ret, soapy.SOAPY_SDR_OVERFLOW)
        self.assertEqual(self.read(np.int8)[0].ret, 672)
        self.assertEqual(self.device.readSensor("rx_missing_samples"), "672")

        old_epoch = self.receiver.state["epoch"]
        self.device.setFrequency(soapy.SOAPY_SDR_RX, 0, 2450000000)
        self.receiver.emit(2016, epoch=old_epoch)  # In-flight old session.
        self.receiver.emit()
        self.assertEqual(self.read(np.int8)[0].ret, 672)
        self.assertEqual(self.device.readSensor("rx_missing_samples"), "672")

    def test_timestamp_across_fractional_microsecond_packet_boundary(self):
        for rate in (20000000, 40000000):
            self.device.setSampleRate(soapy.SOAPY_SDR_RX, 0, rate)
            self.open("CS8")
            self.receiver.emit()
            self.assertEqual(self.read(np.int8, 671)[0].ret, 671)
            last, _ = self.read(np.int8, 1)
            self.assertEqual(last.timeNs, 1000000000 + 671 * 1000000000 // rate)
            self.receiver.emit(672)
            first, _ = self.read(np.int8, 1)
            self.assertEqual(first.ret, 1)
            self.assertEqual(first.timeNs, 1000000000 + 672 * 1000000000 // rate)
            self.assertEqual(first.timeNs - last.timeNs, 1000000000 // rate)
            self.device.deactivateStream(self.stream)
            self.device.closeStream(self.stream)
            self.stream = None

    def test_initial_loss_is_reported(self):
        self.open("CS8")
        self.receiver.emit(1344)
        self.assertEqual(self.read(np.int8)[0].ret, soapy.SOAPY_SDR_OVERFLOW)
        self.assertEqual(self.read(np.int8)[0].ret, 672)
        self.assertEqual(self.device.readSensor("rx_missing_samples"), "1344")

    def test_timeout_and_malformed_packet(self):
        self.open("CS8")
        self.assertEqual(self.read(np.int8)[0].ret, soapy.SOAPY_SDR_TIMEOUT)
        self.receiver.emit(malformed=True)
        self.assertEqual(self.read(np.int8)[0].ret, soapy.SOAPY_SDR_STREAM_ERROR)
        self.assertEqual(self.device.readSensor("rx_invalid_packets"), "1")


if __name__ == "__main__":
    if "--server" in sys.argv:
        server = Server()
        print(server.server_port, flush=True)
        server.serve_forever()
    else:
        unittest.main()
