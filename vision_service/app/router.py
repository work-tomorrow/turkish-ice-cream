import asyncio
import base64

from fastapi import APIRouter, Request, Response
from fastapi.responses import StreamingResponse

from vision_service.app.models import HandDetectionResult, PingResponse

api_router = APIRouter()

PLACEHOLDER_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////"
    "2wBDAf//////////////////////////////////////////////////////////////////////////////////////"
    "wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAX/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/"
    "9oADAMBAAIQAxAAAAH/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAEFAqf/xAAUEQEAAAAAAAAAAAAA"
    "AAAAAAAA/9oACAEDAQE/ASP/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAECAQE/ASP/xAAUEAEAAAAAAAAA"
    "AAAAAAAAAAAA/9oACAEBAAY/Al//xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAE/IV//2gAMAwEAAgAD"
    "AAAAEP/EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQMBAT8QH//EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAI"
    "AQIBAT8QH//EABQQAQAAAAAAAAAAAAAAAAAAABD/2gAIAQEAAT8QH//Z"
)


async def _vision_mjpeg_stream(request: Request):
    while True:
        frame = getattr(request.app.state, "latest_frame_jpeg", None) or PLACEHOLDER_JPEG
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame
            + b"\r\n"
        )
        await asyncio.sleep(1 / 30)


@api_router.get("/ping", response_model=PingResponse, tags=["health"])
async def ping() -> PingResponse:
    return PingResponse(pong=True)


@api_router.get("/hand", response_model=HandDetectionResult, tags=["vision"])
async def get_hand(request: Request) -> HandDetectionResult:
    return getattr(
        request.app.state,
        "latest_hand_result",
        HandDetectionResult(detected=False),
    )


@api_router.get("/vision/frame", tags=["vision"])
async def get_vision_frame(request: Request) -> Response:
    frame = getattr(request.app.state, "latest_frame_jpeg", None) or PLACEHOLDER_JPEG
    return Response(content=frame, media_type="image/jpeg")


@api_router.get("/vision/stream", tags=["vision"])
async def get_vision_stream(request: Request) -> StreamingResponse:
    return StreamingResponse(
        _vision_mjpeg_stream(request),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
