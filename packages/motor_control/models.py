"""Minimal shared models for motor_control."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MotorKind(str, Enum):
    STEPPER = "stepper"
    SERVO = "servo"


@dataclass
class MotorState:
    id: str
    kind: MotorKind
    current_angle: float
    target_angle: float
    moving: bool = False
    ready: bool = True
    last_error: str | None = None
