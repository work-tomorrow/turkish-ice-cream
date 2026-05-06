import asyncio
import base64

from fastapi import APIRouter, HTTPException, Request, Response, WebSocket
from fastapi import WebSocketDisconnect

from core.app.models import (
    ActionCommandResponse,
    ActionName,
    ActionRequest,
    ActionResult,
    ActionsResponse,
    EmergencyStopClearResponse,
    EmergencyStopSetResponse,
    MessageResponse,
    PingResponse,
    SessionCreateRequest,
    SessionCreateResponse,
    SessionActiveResponse,
    SessionDeleteResponse,
    SessionPatchRequest,
    SessionPatchResponse,
    SessionResponse,
    SessionSummary,
    StatusResponse,
)

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


def _status_payload(request: Request | WebSocket) -> StatusResponse:
    hand = request.app.state.hand
    return StatusResponse(
        running=request.app.state.running,
        emergency_stopped=request.app.state.emergency_stopped,
        current_action=request.app.state.current_action,
        tease_count=request.app.state.tease_count,
        max_tease=request.app.state.max_tease,
        hand_detected=hand.detected,
        hand=hand,
    )


def _session_payload(request: Request) -> SessionResponse:
    return SessionResponse(
        active=request.app.state.running,
        current_action=request.app.state.current_action,
        tease_count=request.app.state.tease_count,
        max_tease=request.app.state.max_tease,
    )


def _session_summary(request: Request) -> SessionSummary:
    return SessionSummary(
        active=request.app.state.running,
        max_tease=request.app.state.max_tease,
        tease_count=request.app.state.tease_count,
    )


def _ensure_not_emergency_stopped(request: Request) -> None:
    if request.app.state.emergency_stopped:
        raise HTTPException(status_code=423, detail="Emergency stop is active")


@api_router.get("/", response_model=MessageResponse, tags=["health"])
async def root() -> MessageResponse:
    """Check whether the core API is reachable."""
    return MessageResponse(ok=True, message="Core API is reachable.")


@api_router.get("/ping", response_model=PingResponse, tags=["health"])
async def ping() -> PingResponse:
    """Simple ping probe for frontend and deployment health checks."""
    return PingResponse(pong=True)


@api_router.get("/status", response_model=StatusResponse, tags=["status"])
async def get_status(request: Request) -> StatusResponse:
    """Return the current demo, robot action, and hand detection status."""
    return _status_payload(request)


@api_router.get("/actions", response_model=ActionsResponse, tags=["actions"])
async def get_actions() -> ActionsResponse:
    """List every robot action that can be triggered manually."""
    return ActionsResponse(actions=list(ActionName))


@api_router.post("/actions", response_model=ActionCommandResponse, tags=["actions"])
async def run_action(
    req: ActionRequest,
    request: Request,
) -> ActionCommandResponse:
    """Execute one manual robot action command as a stub state update."""
    _ensure_not_emergency_stopped(request)
    try:
        action_name = ActionName(req.name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="Unknown action") from exc

    request.app.state.current_action = action_name
    return ActionCommandResponse(
        ok=True,
        action=ActionResult(name=action_name, status="executed"),
        current_action=action_name,
    )


@api_router.get("/session", response_model=SessionResponse, tags=["session"])
async def get_session(request: Request) -> SessionResponse:
    """Return the current single demo session state."""
    return _session_payload(request)


@api_router.post(
    "/session",
    response_model=SessionCreateResponse,
    response_model_exclude_none=True,
    tags=["session"],
)
async def create_session(
    req: SessionCreateRequest,
    request: Request,
) -> SessionCreateResponse:
    """Create or return the single active demo session."""
    _ensure_not_emergency_stopped(request)
    if request.app.state.running:
        return SessionCreateResponse(
            ok=True,
            session=_session_summary(request),
            message="session already active",
        )

    request.app.state.running = True
    request.app.state.max_tease = req.max_tease
    request.app.state.tease_count = 0
    request.app.state.current_action = ActionName.present
    return SessionCreateResponse(ok=True, session=_session_summary(request))


@api_router.patch("/session", response_model=SessionPatchResponse, tags=["session"])
async def update_session(
    req: SessionPatchRequest,
    request: Request,
) -> SessionPatchResponse:
    """Pause, resume, or reset the current demo session."""
    if req.active is True:
        _ensure_not_emergency_stopped(request)

    if req.reset:
        request.app.state.tease_count = 0
    if req.active is not None:
        request.app.state.running = req.active
        if req.active:
            request.app.state.current_action = ActionName.present

    return SessionPatchResponse(ok=True, session=_session_summary(request))


@api_router.delete("/session", response_model=SessionDeleteResponse, tags=["session"])
async def delete_session(request: Request) -> SessionDeleteResponse:
    """Stop the current demo session and return the robot to home."""
    request.app.state.running = False
    request.app.state.current_action = ActionName.home
    return SessionDeleteResponse(
        ok=True,
        session=SessionActiveResponse(active=False),
        current_action=request.app.state.current_action,
    )


@api_router.post(
    "/emergency-stop",
    response_model=EmergencyStopSetResponse,
    tags=["emergency-stop"],
)
async def set_emergency_stop(request: Request) -> EmergencyStopSetResponse:
    """Activate emergency stop and halt the current demo session."""
    request.app.state.running = False
    request.app.state.emergency_stopped = True
    request.app.state.current_action = ActionName.stop
    return EmergencyStopSetResponse(
        ok=True,
        emergency_stopped=True,
        running=False,
        current_action=request.app.state.current_action,
    )


@api_router.delete(
    "/emergency-stop",
    response_model=EmergencyStopClearResponse,
    tags=["emergency-stop"],
)
async def clear_emergency_stop(request: Request) -> EmergencyStopClearResponse:
    """Clear emergency stop and return the robot action state to home."""
    request.app.state.emergency_stopped = False
    request.app.state.current_action = ActionName.home
    return EmergencyStopClearResponse(
        ok=True,
        emergency_stopped=False,
        current_action=request.app.state.current_action,
    )


@api_router.get("/vision/frame", tags=["vision"])
async def get_vision_frame() -> Response:
    """Return one placeholder JPEG frame for the hand detection overlay."""
    return Response(content=PLACEHOLDER_JPEG, media_type="image/jpeg")


@api_router.websocket("/ws/vision")
async def ws_vision(ws: WebSocket):
    """Stream placeholder JPEG frames for the hand detection overlay."""
    await ws.accept()
    try:
        while True:
            await ws.send_bytes(PLACEHOLDER_JPEG)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass


@api_router.websocket("/ws/status")
async def ws_status(ws: WebSocket):
    """Stream the current system status as JSON messages."""
    await ws.accept()
    try:
        while True:
            await ws.send_json(_status_payload(ws).model_dump(mode="json"))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass
