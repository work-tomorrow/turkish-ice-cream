from __future__ import annotations

import platform
import sys
from dataclasses import dataclass
from typing import Optional, Protocol

import cv2
import numpy as np


Frame = np.ndarray


class Camera(Protocol):
    """
    Common camera interface used by hand_tracking.

    Implementations should return OpenCV BGR frames.
    """

    def start(self) -> None:
        ...

    def read(self) -> Frame:
        ...

    def stop(self) -> None:
        ...


@dataclass
class PiCameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 30


@dataclass
class OpenCVCameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 30
    camera_index: int = 0


class OpenCVCamera:
    """
    Camera wrapper for local OpenCV-compatible webcams.

    OpenCV VideoCapture returns BGR frames, which matches HandDetector.
    """

    def __init__(self, config: Optional[OpenCVCameraConfig] = None):
        self.config = config or OpenCVCameraConfig()
        self._camera = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return

        camera = cv2.VideoCapture(self.config.camera_index)
        if not camera.isOpened():
            camera.release()
            raise RuntimeError(
                f"Failed to open OpenCV camera index {self.config.camera_index}."
            )

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        camera.set(cv2.CAP_PROP_FPS, self.config.fps)

        self._camera = camera
        self._started = True

    def read(self) -> Frame:
        if not self._started or self._camera is None:
            raise RuntimeError("Camera is not started. Call start() before read().")

        ok, frame = self._camera.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to capture frame from OpenCV camera.")

        return frame

    def stop(self) -> None:
        if self._camera is not None:
            self._camera.release()

        self._started = False
        self._camera = None

    def __enter__(self) -> "OpenCVCamera":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()


class PiCamera2Camera:
    """
    Camera wrapper for Raspberry Pi CSI camera using Picamera2.

    Public contract:
        read() -> OpenCV BGR frame

    Picamera2 captures RGB frames with RGB888 format.
    HandDetector expects OpenCV BGR frames, so this wrapper converts RGB to BGR.
    """

    def __init__(self, config: Optional[PiCameraConfig] = None):
        self.config = config or PiCameraConfig()
        self._camera = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return

        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 is not installed. "
                "On Raspberry Pi, install it with: sudo apt install python3-picamera2"
            ) from exc

        self._camera = Picamera2()

        preview_config = self._camera.create_preview_configuration(
            main={
                "size": (self.config.width, self.config.height),
                "format": "RGB888",
            },
            controls={
                "FrameRate": self.config.fps,
            },
        )

        self._camera.configure(preview_config)
        self._camera.start()
        self._started = True

    def read(self) -> Frame:
        if not self._started or self._camera is None:
            raise RuntimeError("Camera is not started. Call start() before read().")

        rgb_frame = self._camera.capture_array()

        if rgb_frame is None:
            raise RuntimeError("Failed to capture frame from Pi camera.")

        return cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    def stop(self) -> None:
        if self._camera is not None and self._started:
            self._camera.stop()

        self._started = False
        self._camera = None

    def __enter__(self) -> "PiCamera2Camera":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()


def create_default_camera(
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    camera_index: int = 0,
) -> Camera:
    """
    Create the default camera for the current platform.
    """

    if sys.platform == "darwin":
        return OpenCVCamera(
            OpenCVCameraConfig(
                width=width,
                height=height,
                fps=fps,
                camera_index=camera_index,
            )
        )

    if sys.platform.startswith("linux") and platform.machine() == "aarch64":
        return PiCamera2Camera(
            PiCameraConfig(
                width=width,
                height=height,
                fps=fps,
            )
        )

    raise RuntimeError(
        "No default camera is configured for this platform. "
        "Use macOS with an OpenCV webcam or Raspberry Pi Linux aarch64."
    )
