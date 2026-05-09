import asyncio
import logging
import math
from contextlib import suppress

from fastapi import APIRouter, FastAPI, HTTPException, Query, Request, WebSocket
from fastapi import WebSocketDisconnect
from pydantic import ValidationError

from core.app.controller import REACHABLE_ERROR_THRESHOLD_CM
from core.app.models import (
    CircleDemoResponse,
    HandDetectionResult,
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
    SessionCreateResponse,
    SessionDeleteResponse,
    SessionResponse,
    SessionSummary,
    StatusResponse,
    StatusRobot,
    StatusSystem,
    StatusVision,
)
from core.app.vision_state import apply_hand_detection, mark_vision_disconnected

api_router = APIRouter()

DEFAULT_STEP_DEGREES = 3.0

logger = logging.getLogger(__name__)

CIRCLE_DEMO_TICK_S = 0.1


async def _circle_demo_loop(
    app: FastAPI,
    *,
    radius_cm: float,
    period_s: float,
) -> None:
    """
    在 robot frame 的 Y–Z 平面上，讓末端以 (0,0) 為圓心、半徑 radius_cm
    畫圓；X 由目前機構的 Y–Z-only IK 決定，與主互動控制邏輯相同。
    """
    loop = asyncio.get_running_loop()
    t0 = loop.time()
    controller = app.state.controller
    motor_manager = app.state.motor_manager

    try:
        while app.state.circle_demo_running:
            motor_state = motor_manager.get_state()
            controller._sync_ik_limits(motor_state)
            current_targets = controller._current_motor_targets(motor_state)

            elapsed = loop.time() - t0
            theta = (2.0 * math.pi * elapsed) / period_s
            y_cm = radius_cm * math.cos(theta)
            z_cm = radius_cm * math.sin(theta)

            try:
                ik_result = controller.ik.solve_xyz(
                    target_x_cm=0.0,
                    target_y_cm=y_cm,
                    target_z_cm=z_cm,
                    reachable_error_threshold_cm=REACHABLE_ERROR_THRESHOLD_CM,
                )
            except Exception:
                logger.warning("Circle demo IK failed; holding targets", exc_info=True)
                await asyncio.sleep(CIRCLE_DEMO_TICK_S)
                continue

            if not ik_result.reachable:
                logger.warning(
                    "Circle demo target not reachable: y=%.2f z=%.2f err=%.2f",
                    y_cm,
                    z_cm,
                    ik_result.error_cm,
                )
                await asyncio.sleep(CIRCLE_DEMO_TICK_S)
                continue

            targets = controller._joint_targets_to_motor_targets(
                ik_result.motor_targets(),
                base_targets=current_targets,
            )
            motor_manager.set_target(targets)
            await asyncio.sleep(CIRCLE_DEMO_TICK_S)
    except asyncio.CancelledError:
        raise
    finally:
        app.state.circle_demo_running = False
        app.state.circle_demo_task = None


def _status_payload(request: Request | WebSocket) -> StatusResponse:
    hand = request.app.state.hand
    return StatusResponse(
        system=StatusSystem(
            ready=bool(getattr(request.app.state, "vision_ready", False)),
        ),
        robot=StatusRobot(state=request.app.state.robot_state),
        vision=StatusVision(hand_detected=hand.detected, hand=hand),
    )


def _session_payload(request: Request) -> SessionResponse:
    return SessionResponse(
        active=request.app.state.running,
        robot_state=request.app.state.robot_state,
    )


def _session_summary(request: Request) -> SessionSummary:
    return SessionSummary(
        active=request.app.state.running,
    )


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
    request: Request,
) -> SessionCreateResponse:
    """Create or return the single active demo session."""
    if getattr(request.app.state, "circle_demo_running", False):
        raise HTTPException(
            status_code=409,
            detail="Stop the circle demo before starting a session.",
        )
    if request.app.state.running:
        return SessionCreateResponse(
            ok=True,
            session=_session_summary(request),
            message="session already active",
        )

    _ensure_motor_manager_ready(request)

    request.app.state.running = True
    request.app.state.robot_state = RobotState.started

    return SessionCreateResponse(
        ok=True,
        session=_session_summary(request),
    )


@api_router.delete("/session", response_model=SessionDeleteResponse, tags=["session"])
async def delete_session(request: Request) -> SessionDeleteResponse:
    """Stop the current demo session and all motors."""
    request.app.state.robot_state = RobotState.stopping
    request.app.state.running = False
    request.app.state.motor_manager.stop()
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
    "/debug/circle-demo/start",
    response_model=CircleDemoResponse,
    tags=["debug"],
)
async def start_circle_demo(
    request: Request,
    radius_cm: float = Query(
        25.0,
        gt=0,
        le=50,
        description="Y–Z plane circle radius in cm, centered at robot-frame origin.",
    ),
    period_s: float = Query(
        12.0,
        gt=0.5,
        le=120.0,
        description="Seconds per full revolution.",
    ),
) -> CircleDemoResponse:
    """Test-only: move the end-effector on a circle (radius_cm) in the Y–Z plane."""
    if request.app.state.robot_state is RobotState.started:
        raise HTTPException(
            status_code=409,
            detail="Stop the interactive session before starting the circle demo.",
        )
    if getattr(request.app.state, "circle_demo_running", False):
        raise HTTPException(
            status_code=409,
            detail="Circle demo is already running.",
        )

    _ensure_motor_manager_ready(request)

    request.app.state.circle_demo_running = True
    request.app.state.circle_demo_task = asyncio.create_task(
        _circle_demo_loop(
            request.app,
            radius_cm=radius_cm,
            period_s=period_s,
        ),
    )
    return CircleDemoResponse(
        ok=True,
        active=True,
        message="Circle demo started.",
        radius_cm=radius_cm,
        period_s=period_s,
    )


@api_router.delete(
    "/debug/circle-demo/stop",
    response_model=CircleDemoResponse,
    tags=["debug"],
)
async def stop_circle_demo(request: Request) -> CircleDemoResponse:
    """Stop the circle demo background loop."""
    request.app.state.circle_demo_running = False
    task = getattr(request.app.state, "circle_demo_task", None)
    if task is not None and not task.done():
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
    return CircleDemoResponse(
        ok=True,
        active=False,
        message="Circle demo stopped.",
    )


@api_router.websocket("/ws/hand-ingest")
async def ws_hand_ingest(ws: WebSocket):
    """Receive desktop hand detection results from the vision service."""
    await ws.accept()
    try:
        while True:
            payload = await ws.receive_json()
            try:
                result = HandDetectionResult.model_validate(payload)
            except (TypeError, ValueError, ValidationError):
                continue

            apply_hand_detection(ws.app.state, result)
    except WebSocketDisconnect:
        mark_vision_disconnected(ws.app.state)


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
