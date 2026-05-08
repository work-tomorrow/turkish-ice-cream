"""Shared data models for mixed motor control."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class MotorKind(str, Enum):
    """Supported motor categories."""

    STEPPER = "stepper"
    SERVO = "servo"
    PM30225_SERIAL = "pm30225_serial"


@dataclass(frozen=True)
class MotorConfig:
    """Static configuration for one motor."""

    id: str
    kind: MotorKind
    min_angle: float
    max_angle: float
    home_angle: float = 0.0
    zero_offset: float = 0.0
    driver_type: str = "mock"
    driver_options: dict[str, object] = field(default_factory=dict)
    requires_homing: bool = False
    steps_per_rev: int | None = None
    microsteps: int = 1
    gear_ratio: float = 1.0


@dataclass
class MotorState:
    """Runtime state for one motor."""

    id: str
    kind: MotorKind
    # Trajectory setpoints (same interpolation clock as MotorManager ticks).
    current_angle: float
    target_angle: float
    # Optional: hardware-estimated angle (e.g. step count); None if unknown.
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
    motors: dict[str, MotorState]
