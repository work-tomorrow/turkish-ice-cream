import asyncio
import logging
from contextlib import asynccontextmanager, suppress

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from core.app.config import settings
from core.app.models import Hand, RobotState
from core.app.router import api_router
from motor_control import MotorManager
from motor_control.mock_motors import build_mock_motors

logger = logging.getLogger(__name__)


def _create_motor_manager() -> MotorManager:
    try:
        return MotorManager()
    except RuntimeError:
        return MotorManager(motors=build_mock_motors())


def _hand_from_detection(detection) -> Hand:
    if not detection.detected or detection.box is None:
        return Hand()

    return Hand(
        detected=True,
        x=float(detection.box.cx),
        y=float(detection.box.cy),
        confidence=float(detection.confidence),
    )


def _draw_detection_overlay(frame, detection):
    import cv2

    annotated = frame.copy()

    if not detection.detected or detection.box is None:
        return annotated

    x1 = int(detection.box.x1)
    y1 = int(detection.box.y1)
    x2 = int(detection.box.x2)
    y2 = int(detection.box.y2)

    cv2.rectangle(annotated, (x1, y1), (x2, y2), (56, 189, 248), 2)
    cv2.putText(
        annotated,
        f"hand {detection.confidence:.2f}",
        (x1, max(24, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (56, 189, 248),
        2,
        cv2.LINE_AA,
    )
    return annotated


async def _vision_loop(app: FastAPI) -> None:
    from hand_tracking import HandDetector, PiCamera2Camera, encode_frame_to_jpeg

    camera = PiCamera2Camera()
    detector = None

    try:
        detector = HandDetector()
        await asyncio.to_thread(camera.start)
        app.state.vision_ready = True
        app.state.vision_error = None

        while True:
            frame = await asyncio.to_thread(camera.read)
            detection = await asyncio.to_thread(detector.detect, frame)

            app.state.hand = _hand_from_detection(detection)

            annotated = _draw_detection_overlay(frame, detection)
            app.state.latest_frame_jpeg = await asyncio.to_thread(
                encode_frame_to_jpeg,
                annotated,
            )
            await asyncio.sleep(0.01)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        app.state.vision_ready = False
        app.state.vision_error = str(exc)
        app.state.hand = Hand()
        logger.exception("Vision loop stopped")
    finally:
        with suppress(Exception):
            await asyncio.to_thread(camera.stop)


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.running = False
    app.state.robot_state = RobotState.idle
    app.state.tease_count = 0
    app.state.max_tease = 10
    app.state.hand = Hand()
    app.state.latest_frame_jpeg = None
    app.state.vision_ready = False
    app.state.vision_error = None
    app.state.motor_manager = _create_motor_manager()
    app.state.motor_manager_ready = False
    app.state.vision_task = asyncio.create_task(_vision_loop(app))
    try:
        yield
    finally:
        app.state.vision_task.cancel()
        with suppress(asyncio.CancelledError):
            await app.state.vision_task
        app.state.motor_manager.cleanup()


app = FastAPI(
    title=settings.project_name,
    description=settings.description,
    version=settings.version,
    validate_response=True,
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=settings.cors_allow_origin_regex,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router)
