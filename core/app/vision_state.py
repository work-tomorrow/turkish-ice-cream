import asyncio
import time

from core.app.models import Hand, HandDetectionResult


def hand_from_detection(result: HandDetectionResult) -> Hand:
    if not result.detected or result.box is None:
        return Hand()

    width = max(0.0, float(result.box.x2) - float(result.box.x1))
    height = max(0.0, float(result.box.y2) - float(result.box.y1))

    return Hand(
        detected=True,
        x=(result.box.x1 + result.box.x2) / 2,
        y=(result.box.y1 + result.box.y2) / 2,
        area=width * height,
        confidence=float(result.confidence),
    )


def apply_hand_detection(
    state, result: HandDetectionResult, now: float | None = None
) -> None:
    state.hand = hand_from_detection(result)
    state.vision_ready = True
    state.last_hand_result_at = time.monotonic() if now is None else now


def mark_vision_disconnected(state) -> None:
    state.hand = Hand()
    state.vision_ready = False
    state.last_hand_result_at = None


def vision_is_stale(state, timeout_seconds: float, now: float | None = None) -> bool:
    last_seen = getattr(state, "last_hand_result_at", None)
    if last_seen is None:
        return True

    current = time.monotonic() if now is None else now
    return current - float(last_seen) > timeout_seconds


async def vision_stale_monitor(
    app, timeout_seconds: float, interval_seconds: float = 0.2
) -> None:
    while True:
        if getattr(app.state, "vision_ready", False) and vision_is_stale(
            app.state,
            timeout_seconds,
        ):
            mark_vision_disconnected(app.state)

        await asyncio.sleep(interval_seconds)
