import asyncio
import base64

from fastapi import APIRouter, HTTPException, Request, Response, WebSocket
from fastapi import WebSocketDisconnect

from core.app.models import (
    EmergencyStopClearResponse,
    EmergencyStopSetResponse,
    MessageResponse,
    MotorDirection,
    MotorJogRequest,
    MotorJogResponse,
    MotorJogResult,
    MotorStatus,
    MotorTargetRequest,
    MotorTargetResponse,
    MotorZeroResponse,
    MotorsResponse,
    PingResponse,
    RobotState,
    SessionActiveResponse,
    SessionCreateRequest,
    SessionCreateResponse,
    SessionDeleteResponse,
    SessionPatchRequest,
    SessionPatchResponse,
    SessionResponse,
    SessionSummary,
    StatusResponse,
    StatusRobot,
    StatusSystem,
    StatusVision,
)

api_router = APIRouter()

DEFAULT_STEP_DEGREES = 3.0

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
        system=StatusSystem(
            ready=bool(getattr(request.app.state, "vision_ready", False)),
            emergency_stopped=False,
        ),
        robot=StatusRobot(state=request.app.state.robot_state),
        vision=StatusVision(hand_detected=hand.detected, hand=hand),
    )


def _session_payload(request: Request) -> SessionResponse:
    return SessionResponse(
        active=request.app.state.running,
        robot_state=request.app.state.robot_state,
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
    return


def _ensure_motor_id(request: Request, motor_id: str) -> None:
    if motor_id not in request.app.state.motor_manager.motor_ids:
        raise HTTPException(status_code=400, detail=f"Unknown motor id: {motor_id}")


def _ensure_motor_manager_ready(request: Request) -> None:
    if request.app.state.motor_manager_ready:
        return

    try:
        request.app.state.motor_manager.home_all()
    except Exception as exc:
        request.app.state.motor_manager_ready = False
        raise HTTPException(
            status_code=500,
            detail=f"Motor manager failed to initialize: {exc}",
        ) from exc

    request.app.state.motor_manager_ready = True


def _motor_payload(request: Request) -> list[MotorStatus]:
    state = request.app.state.motor_manager.get_state()
    motors = state["motors"]
    return [
        MotorStatus(
            id=motor_id,
            current_angle=float(motor_state["current_angle"]),
            target_angle=float(motor_state["target_angle"]),
            min_angle=float(motor_state["min_angle"]),
            max_angle=float(motor_state["max_angle"]),
            step_degrees=DEFAULT_STEP_DEGREES,
        )
        for motor_id, motor_state in motors.items()
    ]


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
    """Return structured system, robot lifecycle, and vision status."""
    return _status_payload(request)


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
    request.app.state.robot_state = RobotState.started
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
        request.app.state.robot_state = (
            RobotState.started if req.active else RobotState.idle
        )

    return SessionPatchResponse(ok=True, session=_session_summary(request))


@api_router.delete("/session", response_model=SessionDeleteResponse, tags=["session"])
async def delete_session(request: Request) -> SessionDeleteResponse:
    """Stop the current demo session and return the robot to home."""
    request.app.state.robot_state = RobotState.stopping
    request.app.state.running = False
    request.app.state.robot_state = RobotState.idle
    return SessionDeleteResponse(
        ok=True,
        session=SessionActiveResponse(active=False),
        robot_state=request.app.state.robot_state,
    )


@api_router.get("/motors", response_model=MotorsResponse, tags=["motors"])
async def get_motors(request: Request) -> MotorsResponse:
    """List all motor states and jog step sizes."""
    return MotorsResponse(
        motors=_motor_payload(request),
    )


@api_router.post(
    "/motors/{motor_id}/jog",
    response_model=MotorJogResponse,
    tags=["motors"],
)
async def jog_motor(
    motor_id: str,
    req: MotorJogRequest,
    request: Request,
) -> MotorJogResponse:
    """Move one motor by a small relative angle for initial calibration."""
    _ensure_not_emergency_stopped(request)
    _ensure_motor_id(request, motor_id)
    _ensure_motor_manager_ready(request)

    delta_degrees = req.step_degrees
    if req.direction is MotorDirection.negative:
        delta_degrees *= -1

    manager_state = request.app.state.motor_manager.get_state()
    motor_state = manager_state["motors"][motor_id]
    next_angle = float(motor_state["current_angle"]) + delta_degrees
    if next_angle < float(motor_state["min_angle"]) or next_angle > float(
        motor_state["max_angle"]
    ):
        raise HTTPException(
            status_code=400,
            detail=(
                f"Motor {motor_id} angle {next_angle} outside "
                f"[{motor_state['min_angle']}, {motor_state['max_angle']}]"
            ),
        )

    targets = {
        current_motor_id: float(current_motor_state["target_angle"])
        for current_motor_id, current_motor_state in manager_state["motors"].items()
    }
    targets[motor_id] = next_angle
    try:
        request.app.state.motor_manager.set_target(targets)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc

    return MotorJogResponse(
        ok=True,
        action=MotorJogResult(
            motor_id=motor_id,
            delta_degrees=delta_degrees,
            current_angle=next_angle,
            status="moved",
        ),
        robot_state=request.app.state.robot_state,
    )


@api_router.post("/motors/targets", response_model=MotorTargetResponse, tags=["motors"])
async def set_motor_targets(
    req: MotorTargetRequest,
    request: Request,
) -> MotorTargetResponse:
    """Update logical target angles for one or more motors."""
    _ensure_not_emergency_stopped(request)
    _ensure_motor_manager_ready(request)

    manager_state = request.app.state.motor_manager.get_state()
    targets = {
        current_motor_id: float(current_motor_state["target_angle"])
        for current_motor_id, current_motor_state in manager_state["motors"].items()
    }

    for motor_id, angle in req.targets.items():
        _ensure_motor_id(request, motor_id)
        motor_state = manager_state["motors"][motor_id]
        min_angle = float(motor_state["min_angle"])
        max_angle = float(motor_state["max_angle"])
        next_angle = float(angle)
        if next_angle < min_angle or next_angle > max_angle:
            raise HTTPException(
                status_code=400,
                detail=(
                    f"Motor {motor_id} angle {next_angle} outside "
                    f"[{min_angle}, {max_angle}]"
                ),
            )
        targets[motor_id] = next_angle

    try:
        request.app.state.motor_manager.set_target(targets)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc

    return MotorTargetResponse(
        ok=True,
        target_motor_ids=sorted(req.targets.keys()),
        motors=_motor_payload(request),
        robot_state=request.app.state.robot_state,
    )


@api_router.post("/motors/zero", response_model=MotorZeroResponse, tags=["motors"])
async def zero_all_motors(request: Request) -> MotorZeroResponse:
    """Save the current motor positions as the absolute zero calibration."""
    _ensure_not_emergency_stopped(request)
    _ensure_motor_manager_ready(request)

    motor_ids = list(request.app.state.motor_manager.motors)
    try:
        request.app.state.motor_manager.set_current_angle_zero()
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    return MotorZeroResponse(
        ok=True,
        zeroed_motor_ids=motor_ids,
        motors=_motor_payload(request),
        robot_state=request.app.state.robot_state,
    )


@api_router.post(
    "/motors/{motor_id}/zero",
    response_model=MotorZeroResponse,
    tags=["motors"],
)
async def zero_motor(
    motor_id: str,
    request: Request,
) -> MotorZeroResponse:
    """Save one motor's current position as the absolute zero calibration."""
    _ensure_not_emergency_stopped(request)
    _ensure_motor_id(request, motor_id)
    _ensure_motor_manager_ready(request)

    try:
        request.app.state.motor_manager.set_current_angle_zero(motor_id)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    return MotorZeroResponse(
        ok=True,
        zeroed_motor_ids=[motor_id],
        motors=_motor_payload(request),
        robot_state=request.app.state.robot_state,
    )


@api_router.get("/session", response_model=SessionResponse, tags=["session"])
async def get_session(request: Request) -> SessionResponse:
    """Return the current single demo session state."""
    return _session_payload(request)


@api_router.post(
    "/emergency-stop",
    response_model=EmergencyStopSetResponse,
    tags=["emergency-stop"],
)
async def set_emergency_stop(request: Request) -> EmergencyStopSetResponse:
    """Immediately stop all motors without creating a blocked state."""
    request.app.state.running = False
    request.app.state.motor_manager.emergency_stop()
    request.app.state.robot_state = RobotState.idle
    return EmergencyStopSetResponse(
        ok=True,
        emergency_stopped=False,
        running=False,
        robot_state=request.app.state.robot_state,
    )


@api_router.delete(
    "/emergency-stop",
    response_model=EmergencyStopClearResponse,
    tags=["emergency-stop"],
)
async def clear_emergency_stop(request: Request) -> EmergencyStopClearResponse:
    """Compatibility no-op for the old clear endpoint."""
    request.app.state.motor_manager.clear_emergency_stop()
    return EmergencyStopClearResponse(
        ok=True,
        emergency_stopped=False,
        robot_state=request.app.state.robot_state,
    )


@api_router.get("/vision/frame", tags=["vision"])
async def get_vision_frame(request: Request) -> Response:
    """Return the latest vision frame with the current detection overlay."""
    frame = request.app.state.latest_frame_jpeg or PLACEHOLDER_JPEG
    return Response(content=frame, media_type="image/jpeg")


@api_router.websocket("/ws/vision")
async def ws_vision(ws: WebSocket):
    """Stream the latest vision frame with the current detection overlay."""
    await ws.accept()
    try:
        while True:
            frame = ws.app.state.latest_frame_jpeg or PLACEHOLDER_JPEG
            await ws.send_bytes(frame)
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
