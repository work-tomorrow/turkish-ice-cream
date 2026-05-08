from __future__ import annotations

from collections.abc import Generator

import cv2

from .camera import Camera


def encode_frame_to_jpeg(frame, jpeg_quality: int = 80) -> bytes:
    """
    Encode one OpenCV BGR frame into JPEG bytes.
    """

    jpeg_quality = max(1, min(100, jpeg_quality))

    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
    )

    if not ok:
        raise RuntimeError("Failed to encode frame as JPEG.")

    return encoded.tobytes()


def stream_camera_jpeg(
    camera: Camera,
    jpeg_quality: int = 80,
) -> Generator[bytes, None, None]:
    """
    Yield JPEG bytes from camera frames.

    This is suitable for WebSocket or any upper layer that wants raw JPEG bytes.
    """

    camera.start()

    try:
        while True:
            frame = camera.read()
            yield encode_frame_to_jpeg(frame, jpeg_quality=jpeg_quality)

    finally:
        camera.stop()


def stream_camera_mjpeg(
    camera: Camera,
    jpeg_quality: int = 80,
) -> Generator[bytes, None, None]:
    """
    Yield MJPEG chunks from camera frames.

    Upper layer should use this content type:

        multipart/x-mixed-replace; boundary=frame

    This function intentionally does not import FastAPI.
    """

    for jpeg in stream_camera_jpeg(camera, jpeg_quality=jpeg_quality):
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg
            + b"\r\n"
        )