from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class RobotState(str, Enum):
    idle = "idle"
    started = "started"
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
    confidence: Optional[float] = None


class StatusSystem(BaseModel):
    ready: bool
    emergency_stopped: bool


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
    tease_count: int
    max_tease: int


class SessionSummary(BaseModel):
    active: bool
    max_tease: int
    tease_count: int


class SessionActiveResponse(BaseModel):
    active: bool


class SessionCreateRequest(BaseModel):
    max_tease: int = 3


class SessionPatchRequest(BaseModel):
    active: Optional[bool] = None
    reset: bool = False


class SessionCreateResponse(BaseModel):
    ok: bool
    session: SessionSummary
    message: Optional[str] = None


class SessionPatchResponse(BaseModel):
    ok: bool
    session: SessionSummary


class SessionDeleteResponse(BaseModel):
    ok: bool
    session: SessionActiveResponse
    robot_state: RobotState


class EmergencyStopSetResponse(BaseModel):
    ok: bool
    emergency_stopped: bool
    running: bool
    robot_state: RobotState


class EmergencyStopClearResponse(BaseModel):
    ok: bool
    emergency_stopped: bool
    robot_state: RobotState
