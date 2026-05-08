from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


@dataclass
class Box:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass
class HandDetectionResult:
    detected: bool
    box: Optional[Box] = None
    confidence: float = 0.0


class HandDetector:
    def __init__(
        self,
        model_path: Optional[str] = None,
        num_hands: int = 1,
        min_hand_detection_confidence: float = 0.5,
        min_hand_presence_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):
        if model_path is None:
            model_path = (
                Path(__file__).resolve().parent
                / "models"
                / "hand_landmarker.task"
            )
        else:
            model_path = Path(model_path)

        if not model_path.exists():
            raise FileNotFoundError(f"MediaPipe model not found: {model_path}")

        base_options = python.BaseOptions(
            model_asset_path=str(model_path)
        )

        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.IMAGE,
            num_hands=num_hands,
            min_hand_detection_confidence=min_hand_detection_confidence,
            min_hand_presence_confidence=min_hand_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

        self.detector = vision.HandLandmarker.create_from_options(options)

    def detect(self, frame) -> HandDetectionResult:
        if frame is None:
            return HandDetectionResult(detected=False)

        height, width, _ = frame.shape

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_frame,
        )

        result = self.detector.detect(mp_image)

        if not result.hand_landmarks:
            return HandDetectionResult(detected=False)

        landmarks = result.hand_landmarks[0]

        xs = [lm.x * width for lm in landmarks]
        ys = [lm.y * height for lm in landmarks]

        x1 = max(0.0, min(xs))
        y1 = max(0.0, min(ys))
        x2 = min(float(width), max(xs))
        y2 = min(float(height), max(ys))

        confidence = 1.0
        if result.handedness and len(result.handedness) > 0:
            confidence = float(result.handedness[0][0].score)

        return HandDetectionResult(
            detected=True,
            box=Box(x1=x1, y1=y1, x2=x2, y2=y2),
            confidence=confidence,
        )