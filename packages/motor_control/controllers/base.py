"""Shared controller protocol and state handling."""

from __future__ import annotations

from threading import RLock
from typing import Protocol

from motor_control.models import MotorConfig, MotorState


class MotorController(Protocol):
    """Controller interface used by MotorManager."""

    config: MotorConfig

    def setup(self) -> None: ...

    def home(self) -> None: ...

    def move_to(self, angle: float, duration_ms: int) -> None: ...

    def update(
        self,
        target_angle: float,
        duration_ms: int,
        command_id: int,
        dt_ms: float,
    ) -> None: ...

    def stop(self) -> None: ...

    def get_state(self) -> MotorState: ...


class BaseController:
    """Shared state handling for all motor types."""

    def __init__(self, config: MotorConfig, driver: object) -> None:
        self.config = config
        self.driver = driver
        self._lock = RLock()
        ready = not config.requires_homing
        self.state = MotorState(
            id=config.id,
            kind=config.kind,
            current_angle=config.home_angle,
            target_angle=config.home_angle,
            ready=ready,
            homed=ready,
            approximate=config.position_control_mode == "approximate",
        )
        self._active_command_id = 0
        self._move_start_angle = config.home_angle
        self._move_elapsed_ms = 0.0

    def setup(self) -> None:
        self.driver.setup()

    def home(self) -> None:
        with self._lock:
            self.state.current_angle = self.config.home_angle
            self.state.target_angle = self.config.home_angle
            self.state.ready = True
            self.state.homed = True
            self.state.moving = False

    def stop(self) -> None:
        with self._lock:
            self.state.moving = False
        self.driver.stop()

    def get_state(self) -> MotorState:
        with self._lock:
            return MotorState(**self.state.__dict__)

    def _ensure_ready(self) -> None:
        if not self.state.ready:
            raise RuntimeError(f"Motor {self.config.id!r} is not ready")

    def _begin_command_if_needed(self, target_angle: float, command_id: int) -> None:
        if command_id == self._active_command_id:
            return
        self._active_command_id = command_id
        self._move_start_angle = self.state.current_angle
        self._move_elapsed_ms = 0.0
        self.state.target_angle = target_angle
        self.state.moving = self.state.current_angle != target_angle

    def _next_angle(self, target_angle: float, duration_ms: int, dt_ms: float) -> float:
        if duration_ms <= 0:
            return target_angle

        self._move_elapsed_ms = min(duration_ms, self._move_elapsed_ms + dt_ms)
        progress = self._move_elapsed_ms / duration_ms
        return self._move_start_angle + (
            target_angle - self._move_start_angle
        ) * progress
