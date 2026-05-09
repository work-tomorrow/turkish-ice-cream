"""Shared data models for motor control."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class MotorKind(str, Enum):
    """Supported motor categories."""

    STEPPER = "stepper"
    SERVO = "servo"


@dataclass(frozen=True)
class MotorConfig:
    """Static configuration for one motor."""

    id: str
    kind: MotorKind
    min_angle: float
    max_angle: float
    home_angle: float = 0.0
    zero_offset: float = 0.0
    driver_type: str = "hardware"
    driver_options: dict[str, Any] = field(default_factory=dict)
    requires_homing: bool = False
    steps_per_rev: int | None = None
    microsteps: int = 1
    gear_ratio: float = 1.0


@dataclass
class MotorState:
    """Runtime state for one motor."""

    id: str
    kind: MotorKind
    current_angle: float
    target_angle: float
    hw_angle: float | None = None
    moving: bool = False
    homed: bool = False
    ready: bool = False
    last_command: str | None = None
    last_error: str | None = None


@dataclass
class MotorManagerState:
    """Snapshot of the manager and all motors."""

    running: bool
    emergency_stopped: bool
    motors: dict[str, MotorState]
