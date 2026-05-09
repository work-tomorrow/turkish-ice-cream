from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class RobotState(str, Enum):
    idle = "idle"
    started = "started"
    debug = "debug"
    stopping = "stopping"
    error = "error"


class MotorDirection(str, Enum):
    positive = "positive"
    negative = "negative"


class MessageResponse(BaseModel):
    ok: bool = Field(..., description="Indicates whether the call succeeded.")
    message: str = Field(..., description="Human-readable message.")


class PingResponse(BaseModel):
    pong: bool


class Hand(BaseModel):
    detected: bool = False
    x: Optional[float] = None
    y: Optional[float] = None
    area: Optional[float] = None
    confidence: Optional[float] = None


class HandDetectionBox(BaseModel):
    x1: float
    y1: float
    x2: float
    y2: float


class HandDetectionResult(BaseModel):
    detected: bool
    box: Optional[HandDetectionBox] = None
    confidence: float = 0.0


class StatusSystem(BaseModel):
    ready: bool


class StatusRobot(BaseModel):
    state: RobotState


class StatusVision(BaseModel):
    hand_detected: bool
    hand: Hand


class StatusResponse(BaseModel):
    system: StatusSystem
    robot: StatusRobot
    vision: StatusVision


class MotorsResponse(BaseModel):
    motors: list["MotorStatus"]


class MotorJogRequest(BaseModel):
    direction: MotorDirection
    step_degrees: float = Field(
        3.0,
        gt=0,
        le=10,
        description="Relative move size in degrees.",
    )


class MotorJogResult(BaseModel):
    motor_id: str
    delta_degrees: float
    current_angle: float
    status: str


class MotorJogResponse(BaseModel):
    ok: bool
    action: MotorJogResult
    robot_state: RobotState


class MotorStatus(BaseModel):
    id: str
    current_angle: float
    target_angle: float
    min_angle: float
    max_angle: float
    step_degrees: float = 3.0


class MotorTargetRequest(BaseModel):
    targets: dict[str, float] = Field(
        ...,
        min_length=1,
        description="Requested logical target angle for one or more motors.",
    )


class MotorTargetResponse(BaseModel):
    ok: bool
    target_motor_ids: list[str]
    motors: list[MotorStatus]
    robot_state: RobotState


class MotorZeroResponse(BaseModel):
    ok: bool
    zeroed_motor_ids: list[str]
    motors: list[MotorStatus]
    robot_state: RobotState


class SessionResponse(BaseModel):
    active: bool
    robot_state: RobotState


class SessionSummary(BaseModel):
    active: bool


class SessionActiveResponse(BaseModel):
    active: bool


class SessionCreateResponse(BaseModel):
    ok: bool
    session: SessionSummary
    message: Optional[str] = None


class SessionDeleteResponse(BaseModel):
    ok: bool
    session: SessionActiveResponse
    robot_state: RobotState


class DebugSessionResponse(BaseModel):
    ok: bool
    active: bool
    moving: bool = False
    message: str = ""
    target_y_cm: float | None = None
    target_z_cm: float | None = None
    duration_s: float | None = None


class DebugMoveRequest(BaseModel):
    y_cm: float = Field(
        ...,
        ge=-25.0,
        le=25.0,
        description="Robot-frame Y target in centimeters.",
    )
    z_cm: float = Field(
        ...,
        ge=-25.0,
        le=25.0,
        description="Robot-frame Z target in centimeters.",
    )
    duration_s: float = Field(
        5.0,
        gt=0.0,
        le=60.0,
        description="Move duration in seconds.",
    )


class DebugMoveResponse(BaseModel):
    ok: bool
    active: bool
    moving: bool
    message: str = ""
    target_y_cm: float
    target_z_cm: float
    duration_s: float
