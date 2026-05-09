import unittest
from unittest import mock

import numpy as np

from hand_tracking import (
    OpenCVCamera,
    OpenCVCameraConfig,
    PiCamera2Camera,
    create_default_camera,
)


class FakeVideoCapture:
    def __init__(self, frame=None, opened=True):
        self.frame = frame
        self.opened = opened
        self.released = False
        self.set_calls = []

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.set_calls.append((prop, value))
        return True

    def read(self):
        if self.frame is None:
            return False, None
        return True, self.frame

    def release(self):
        self.released = True


class OpenCVCameraTest(unittest.TestCase):
    def test_read_returns_bgr_frame_from_video_capture(self) -> None:
        frame = np.zeros((24, 32, 3), dtype=np.uint8)
        fake_capture = FakeVideoCapture(frame=frame)

        with mock.patch(
            "hand_tracking.camera.cv2.VideoCapture",
            return_value=fake_capture,
        ) as video_capture:
            camera = OpenCVCamera(
                OpenCVCameraConfig(
                    width=320,
                    height=240,
                    fps=15,
                    camera_index=2,
                )
            )
            camera.start()
            captured = camera.read()
            camera.stop()

        video_capture.assert_called_once_with(2)
        self.assertIs(captured, frame)
        self.assertTrue(fake_capture.released)
        self.assertEqual(len(fake_capture.set_calls), 3)

    def test_open_failure_raises_clear_error(self) -> None:
        fake_capture = FakeVideoCapture(opened=False)

        with mock.patch(
            "hand_tracking.camera.cv2.VideoCapture",
            return_value=fake_capture,
        ):
            camera = OpenCVCamera()
            with self.assertRaisesRegex(RuntimeError, "Failed to open OpenCV camera"):
                camera.start()

        self.assertTrue(fake_capture.released)

    def test_read_failure_raises_clear_error(self) -> None:
        fake_capture = FakeVideoCapture(frame=None)

        with mock.patch(
            "hand_tracking.camera.cv2.VideoCapture",
            return_value=fake_capture,
        ):
            camera = OpenCVCamera()
            camera.start()
            with self.assertRaisesRegex(RuntimeError, "Failed to capture frame"):
                camera.read()
            camera.stop()


class DefaultCameraFactoryTest(unittest.TestCase):
    def test_macos_uses_opencv_camera(self) -> None:
        with mock.patch("hand_tracking.camera.sys.platform", "darwin"):
            camera = create_default_camera(
                width=800,
                height=600,
                fps=24,
                camera_index=1,
            )

        self.assertIsInstance(camera, OpenCVCamera)
        self.assertEqual(camera.config.width, 800)
        self.assertEqual(camera.config.height, 600)
        self.assertEqual(camera.config.fps, 24)
        self.assertEqual(camera.config.camera_index, 1)

    def test_raspberry_pi_linux_uses_picamera2_camera(self) -> None:
        with mock.patch("hand_tracking.camera.sys.platform", "linux"):
            with mock.patch("hand_tracking.camera.platform.machine", return_value="aarch64"):
                camera = create_default_camera(width=320, height=240, fps=10)

        self.assertIsInstance(camera, PiCamera2Camera)
        self.assertEqual(camera.config.width, 320)
        self.assertEqual(camera.config.height, 240)
        self.assertEqual(camera.config.fps, 10)

    def test_unsupported_platform_raises_clear_error(self) -> None:
        with mock.patch("hand_tracking.camera.sys.platform", "linux"):
            with mock.patch("hand_tracking.camera.platform.machine", return_value="x86_64"):
                with self.assertRaisesRegex(RuntimeError, "No default camera"):
                    create_default_camera()


if __name__ == "__main__":
    unittest.main()
