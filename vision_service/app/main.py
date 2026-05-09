import asyncio
import logging
import time
from contextlib import asynccontextmanager, suppress

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from vision_service.app.config import settings
from vision_service.app.models import HandDetectionBox, HandDetectionResult
from vision_service.app.router import api_router

logger = logging.getLogger(__name__)

VISION_WIDTH = 640
VISION_HEIGHT = 480
VISION_FPS = 30
VISION_JPEG_QUALITY = 65
VISION_DETECTION_HZ = 20
VISION_DETECTION_WIDTH = 320
VISION_DETECTION_HEIGHT = 240
VISION_TIMING_LOG_INTERVAL = 2.0


def _empty_result() -> HandDetectionResult:
    return HandDetectionResult(detected=False)


def _draw_detection_overlay(frame, result: HandDetectionResult):
    import cv2

    annotated = frame.copy()

    if not result.detected or result.box is None:
        return annotated

    x1 = int(result.box.x1)
    y1 = int(result.box.y1)
    x2 = int(result.box.x2)
    y2 = int(result.box.y2)

    cv2.rectangle(annotated, (x1, y1), (x2, y2), (56, 189, 248), 2)
    cv2.putText(
        annotated,
        f"hand {result.confidence:.2f}",
        (x1, max(24, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (56, 189, 248),
        2,
        cv2.LINE_AA,
    )
    return annotated


def _prepare_detection_frame(frame):
    import cv2

    return cv2.resize(
        frame,
        (VISION_DETECTION_WIDTH, VISION_DETECTION_HEIGHT),
        interpolation=cv2.INTER_LINEAR,
    )


def _scale_detection(detection, frame_width: int, frame_height: int) -> HandDetectionResult:
    if not detection.detected or detection.box is None:
        return _empty_result()

    scale_x = frame_width / VISION_DETECTION_WIDTH
    scale_y = frame_height / VISION_DETECTION_HEIGHT

    return HandDetectionResult(
        detected=True,
        box=HandDetectionBox(
            x1=float(detection.box.x1 * scale_x),
            y1=float(detection.box.y1 * scale_y),
            x2=float(detection.box.x2 * scale_x),
            y2=float(detection.box.y2 * scale_y),
        ),
        confidence=float(detection.confidence),
    )


def _queue_latest_result(app: FastAPI, result: HandDetectionResult) -> None:
    queue: asyncio.Queue = app.state.hand_result_queue
    if queue.full():
        with suppress(asyncio.QueueEmpty):
            queue.get_nowait()

    queue.put_nowait(result)


async def _vision_loop(app: FastAPI) -> None:
    from hand_tracking import (
        HandDetector,
        create_default_camera,
        encode_frame_to_jpeg,
    )

    camera = create_default_camera(
        width=VISION_WIDTH,
        height=VISION_HEIGHT,
        fps=VISION_FPS,
    )
    result = _empty_result()
    next_detection_at = 0.0
    stats_started_at = time.monotonic()
    stats_last_logged_at = stats_started_at
    stats_frames = 0
    stats_reads = 0.0
    stats_detects = 0.0
    stats_encodes = 0.0
    stats_loops = 0.0

    try:
        detector = HandDetector()
        await asyncio.to_thread(camera.start)
        app.state.vision_ready = True
        app.state.vision_error = None

        while True:
            loop_started_at = time.monotonic()

            read_started_at = time.monotonic()
            frame = await asyncio.to_thread(camera.read)
            stats_reads += time.monotonic() - read_started_at

            now = time.monotonic()
            if now >= next_detection_at:
                detect_started_at = time.monotonic()
                detect_frame = await asyncio.to_thread(_prepare_detection_frame, frame)
                raw_detection = await asyncio.to_thread(detector.detect, detect_frame)
                result = _scale_detection(
                    raw_detection,
                    frame.shape[1],
                    frame.shape[0],
                )
                app.state.latest_hand_result = result
                _queue_latest_result(app, result)
                stats_detects += time.monotonic() - detect_started_at
                next_detection_at = now + (1.0 / VISION_DETECTION_HZ)

            annotated = _draw_detection_overlay(frame, result)
            encode_started_at = time.monotonic()
            app.state.latest_frame_jpeg = await asyncio.to_thread(
                encode_frame_to_jpeg,
                annotated,
                VISION_JPEG_QUALITY,
            )
            stats_encodes += time.monotonic() - encode_started_at
            stats_frames += 1
            stats_loops += time.monotonic() - loop_started_at

            if now - stats_last_logged_at >= VISION_TIMING_LOG_INTERVAL:
                elapsed = now - stats_last_logged_at
                total_elapsed = now - stats_started_at
                if elapsed > 0 and total_elapsed > 0 and stats_frames > 0:
                    logger.info(
                        (
                            "vision timing: "
                            "read=%.1fms detect=%.1fms encode=%.1fms loop=%.1fms "
                            "fps=%.1f total_fps=%.1f"
                        ),
                        (stats_reads / stats_frames) * 1000.0,
                        (stats_detects / stats_frames) * 1000.0,
                        (stats_encodes / stats_frames) * 1000.0,
                        (stats_loops / stats_frames) * 1000.0,
                        stats_frames / elapsed,
                        stats_frames / total_elapsed,
                    )
                stats_last_logged_at = now
                stats_frames = 0
                stats_reads = 0.0
                stats_detects = 0.0
                stats_encodes = 0.0
                stats_loops = 0.0
                stats_started_at = now
            await asyncio.sleep(0.001)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        app.state.vision_ready = False
        app.state.vision_error = str(exc)
        app.state.latest_hand_result = _empty_result()
        logger.exception("Vision loop stopped")
    finally:
        with suppress(Exception):
            await asyncio.to_thread(camera.stop)


async def _rpi_push_loop(app: FastAPI) -> None:
    import websockets

    while True:
        try:
            async with websockets.connect(settings.rpi_hand_ingest_ws_url) as ws:
                app.state.rpi_connected = True
                await ws.send(app.state.latest_hand_result.model_dump_json())

                while True:
                    result = await app.state.hand_result_queue.get()
                    await ws.send(result.model_dump_json())
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            app.state.rpi_connected = False
            logger.warning("RPI hand ingest connection failed: %s", exc)
            await asyncio.sleep(settings.rpi_reconnect_seconds)


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.latest_frame_jpeg = None
    app.state.latest_hand_result = _empty_result()
    app.state.hand_result_queue = asyncio.Queue(maxsize=1)
    app.state.vision_ready = False
    app.state.vision_error = None
    app.state.rpi_connected = False
    app.state.vision_task = asyncio.create_task(_vision_loop(app))
    app.state.rpi_push_task = asyncio.create_task(_rpi_push_loop(app))
    try:
        yield
    finally:
        app.state.vision_task.cancel()
        app.state.rpi_push_task.cancel()
        with suppress(asyncio.CancelledError):
            await app.state.vision_task
        with suppress(asyncio.CancelledError):
            await app.state.rpi_push_task


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
