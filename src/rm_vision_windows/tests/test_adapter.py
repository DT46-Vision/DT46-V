import ctypes as ct
import json
import socket
import struct
import threading
import unittest

import numpy as np

from rm_vision_windows.aim import AimPipeline, Result
from rm_vision_windows.camera import RemoteCamera, aim_feedback
from rm_vision_windows.transport import Connection


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.pipeline = AimPipeline(debug=True)
        self.addCleanup(self.pipeline.close)
        self.image = np.zeros((128, 160, 3), dtype=np.uint8)
        self.k = np.array([[200., 0., 77.], [0., 201., 61.], [0., 0., 1.]])
        self.d = np.array([-.1, .01, .001, -.002, 0.])
        self.rotation = np.eye(3)
        self.offset = np.array([0., .08, .03])

    def process(self, timestamp=0.):
        return self.pipeline.process(self.image, timestamp, self.k, self.d, self.rotation, self.offset, 1)

    def test_current_abi_and_unchanged_inputs(self):
        self.assertEqual(ct.sizeof(Result), self.pipeline.library.dt46_result_size())
        before = self.image.copy()
        result = self.process()
        self.assertEqual(result["state"], "LOST")
        self.assertFalse(result["aim"]["valid"])
        np.testing.assert_array_equal(before, self.image)
        np.testing.assert_array_equal(self.k, result["input"]["K"])
        np.testing.assert_array_equal(self.d, result["input"]["D"])
        np.testing.assert_array_equal(self.rotation, result["input"]["optical_rotation"])
        np.testing.assert_array_equal(self.offset, result["input"]["muzzle_offset_camera_m"])

    def test_failed_configuration_keeps_context_and_revision(self):
        context, revision = self.pipeline.handle, self.pipeline.revision
        for changes in ({"detector": {"roi_scale": 0}}, {"model": "nonexistent-model.onnx"},
                        {"tracker": {"gravity": 8.}}, {"detector": {"unknown": 0}}):
            with self.assertRaises((ValueError, RuntimeError)):
                self.pipeline.configure(**changes)
            self.assertEqual((context, revision), (self.pipeline.handle, self.pipeline.revision))
        self.assertEqual(self.process()["state"], "LOST")

    def test_reset_and_strict_timestamps(self):
        self.process(1.)
        with self.assertRaises(RuntimeError):
            self.process(1.)
        revision = self.pipeline.revision
        self.pipeline.reset()
        self.assertEqual(self.pipeline.revision, revision + 1)
        self.assertEqual(self.process(0.)["simulation_time_s"], 0.)

    def test_manual_decision_color_and_debug_images(self):
        self.pipeline.configure(tracker={"follow_decision": False, "target_color": 0})
        result = self.process()
        self.assertEqual(result["input"]["target_color"], 0)
        binary = self.pipeline.debug_image("bin_img")
        self.assertIsNotNone(binary)
        self.assertEqual(binary.dtype, np.uint8)
        self.pipeline.configure(detector={"display_mode": False})
        self.assertIsNone(self.pipeline.debug_image("result"))


class TransportTests(unittest.TestCase):
    def serve(self, function):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        self.addCleanup(listener.close)
        errors = []

        def worker():
            try:
                stream, _ = listener.accept()
                connection = Connection(stream)
                try:
                    function(connection)
                finally:
                    connection.close()
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=worker)
        thread.start()
        def finish():
            thread.join(3)
            self.assertFalse(thread.is_alive(), "Camera test server did not finish")
            self.assertEqual(errors, [])
        self.addCleanup(finish)
        return listener.getsockname()[1]

    def test_fragmented_camera_packet_and_exact_feedback(self):
        pixels = bytes(range(36))
        result = {"schema": 1, "frame_number": 7, "state_tick": 21, "reset_generation": 2,
                  "simulation_time_s": 0.5, "camera": "onboard", "robot_id": "red3",
                  "state": "LOST", "tracked_id": -1,
                  "aim": {"valid": False, "can_fire": False, "yaw_error_deg": 0., "pitch_error_deg": 0.}}

        def server(connection):
            self.assertEqual(connection.receive()[0], {"key": "test"})
            connection.send({"protocol": 1})
            self.assertEqual(connection.receive()[0]["op"], "describe")
            connection.send({"protocol": 1, "camera": {}, "physics": {}})
            self.assertEqual(connection.receive()[0]["after"], 0)
            frame = {"width": 4, "height": 3, "color_order": "RGB", "frame_number": 7,
                     "K": np.eye(3).tolist(), "D": [0]*5, "optical": np.eye(4).tolist(), "muzzle_offset": [0]*3}
            data = json.dumps({"version": 7, "frame": frame}).encode()
            packet = struct.pack("<8sII", b"DT46V001", len(data), len(pixels)) + data + pixels
            for index in range(0, len(packet), 3):
                connection.stream.sendall(packet[index:index+3])
            request, _ = connection.receive()
            self.assertEqual(request["result"], result)
            self.assertEqual(request["after"], 7)
            connection.send({"version": 7, "frame": None})

        camera = RemoteCamera({"port": self.serve(server), "key": "test"})
        self.addCleanup(camera.close)
        frame = camera.next({}, None)
        self.assertEqual(frame["image"].tobytes(), pixels)
        self.assertIsNone(camera.next({}, aim_feedback(result)))
        with self.assertRaises(ValueError):
            camera.validate(flip_image=False)

    def test_invalid_wire_header(self):
        def server(connection):
            connection.stream.sendall(struct.pack("<8sII", b"BADMAGIC", 1, 0))
        connection = Connection.connect(self.serve(server))
        self.addCleanup(connection.close)
        with self.assertRaises(ValueError):
            connection.receive()


if __name__ == "__main__":
    unittest.main()
