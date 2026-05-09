from typing import Optional

from pydantic import BaseModel


class PingResponse(BaseModel):
    pong: bool


class HandDetectionBox(BaseModel):
    x1: float
    y1: float
    x2: float
    y2: float


class HandDetectionResult(BaseModel):
    detected: bool
    box: Optional[HandDetectionBox] = None
    confidence: float = 0.0
