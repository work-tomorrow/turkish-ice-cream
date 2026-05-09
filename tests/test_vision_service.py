import asyncio
import unittest
from types import SimpleNamespace

import numpy as np

from vision_service.app import main as vision_main
from vision_service.app import router as vision_router
from vision_service.app.main import app
from vision_service.app.models import HandDetectionResult


class VisionServiceTest(unittest.TestCase):
    def test_openapi_exposes_desktop_vision_routes(self) -> None:
        schema = app.openapi()
        paths = schema["paths"]

        self.assertIn("/ping", paths)
        self.assertIn("/hand", paths)
        self.assertIn("/vision/frame", paths)
        self.assertIn("/vision/stream", paths)

    def test_get_hand_returns_latest_detection_result(self) -> None:
        result = HandDetectionResult(detected=True, confidence=0.8)
        request = SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace(latest_hand_result=result)))

        response = asyncio.run(vision_router.get_hand(request))

        self.assertIs(response, result)

    def test_preview_frame_falls_back_to_placeholder(self) -> None:
        request = SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace(latest_frame_jpeg=None)))

        response = asyncio.run(vision_router.get_vision_frame(request))

        self.assertEqual(response.media_type, "image/jpeg")
        self.assertTrue(response.body.startswith(bytes([255, 216])))

    def test_detection_result_scales_from_detector_frame_to_preview_frame(self) -> None:
        detection = SimpleNamespace(
            detected=True,
            box=SimpleNamespace(x1=5, y1=10, x2=25, y2=30),
            confidence=0.6,
        )

        result = vision_main._scale_detection(detection, frame_width=640, frame_height=480)

        self.assertTrue(result.detected)
        self.assertIsNotNone(result.box)
        self.assertEqual(result.box.x1, 10.0)
        self.assertEqual(result.box.y1, 20.0)
        self.assertEqual(result.box.x2, 50.0)
        self.assertEqual(result.box.y2, 60.0)
        self.assertEqual(result.confidence, 0.6)

    def test_latest_result_queue_keeps_only_newest_payload(self) -> None:
        queue = asyncio.Queue(maxsize=1)
        fake_app = SimpleNamespace(state=SimpleNamespace(hand_result_queue=queue))
        older = HandDetectionResult(detected=False)
        newer = HandDetectionResult(detected=True, confidence=0.9)

        vision_main._queue_latest_result(fake_app, older)
        vision_main._queue_latest_result(fake_app, newer)

        self.assertIs(queue.get_nowait(), newer)
        self.assertTrue(queue.empty())

    def test_overlay_does_not_require_real_camera(self) -> None:
        frame = np.zeros((80, 80, 3), dtype=np.uint8)
        detection = SimpleNamespace(
            detected=True,
            box=SimpleNamespace(x1=10, y1=10, x2=40, y2=40),
            confidence=0.7,
        )
        result = vision_main._scale_detection(detection, frame_width=80, frame_height=80)

        annotated = vision_main._draw_detection_overlay(frame, result)

        self.assertEqual(annotated.shape, frame.shape)


if __name__ == "__main__":
    unittest.main()
