from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class ActionName(str, Enum):
    home = "home"
    present = "present"
    pull_back = "pull_back"
    dodge_left = "dodge_left"
    dodge_right = "dodge_right"
    dodge_up = "dodge_up"
    spin = "spin"
    give = "give"
    stop = "stop"


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


class StatusResponse(BaseModel):
    running: bool
    emergency_stopped: bool
    current_action: ActionName
    tease_count: int
    max_tease: int
    hand_detected: bool
    hand: Hand


class ActionsResponse(BaseModel):
    actions: list[ActionName]


class ActionRequest(BaseModel):
    name: str


class ActionResult(BaseModel):
    name: ActionName
    status: str


class ActionCommandResponse(BaseModel):
    ok: bool
    action: ActionResult
    current_action: ActionName


class SessionResponse(BaseModel):
    active: bool
    current_action: ActionName
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
    current_action: ActionName


class EmergencyStopSetResponse(BaseModel):
    ok: bool
    emergency_stopped: bool
    running: bool
    current_action: ActionName


class EmergencyStopClearResponse(BaseModel):
    ok: bool
    emergency_stopped: bool
    current_action: ActionName
