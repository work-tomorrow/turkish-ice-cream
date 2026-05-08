"""Stepper angle controller."""

from __future__ import annotations

from motor_control.controllers.base import BaseController
from motor_control.models import MotorConfig


class StepperController(BaseController):
    """Translate angle targets into stepper movement commands."""

    def __init__(self, config: MotorConfig, driver: object) -> None:
        if config.steps_per_rev is None:
            raise ValueError(f"Stepper {config.id!r} needs steps_per_rev")
        super().__init__(config, driver)
        self.current_steps = self._angle_to_steps(config.home_angle)

    def home(self) -> None:
        self.driver.home()
        super().home()
        with self._lock:
            self.current_steps = self._angle_to_steps(self.config.home_angle)

    def set_current_angle_zero(self) -> None:
        super().set_current_angle_zero()
        with self._lock:
            self.current_steps = self._angle_to_steps(0.0)

    def move_to(self, angle: float, duration_ms: int) -> None:
        self._ensure_ready()
        target_steps = self._angle_to_steps(angle)
        self.driver.move_steps(target_steps - self.current_steps, duration_ms)
        self.current_steps = target_steps
        self.state.current_angle = angle
        self.state.target_angle = angle
        self.state.last_command = (
            f"MOVE_STEPPER {self.config.id} {angle:.3f} {duration_ms}"
        )

    def update(
        self,
        target_angle: float,
        duration_ms: int,
        command_id: int,
        dt_ms: float,
    ) -> None:
        with self._lock:
            self._ensure_ready()
            self._begin_command_if_needed(target_angle, command_id)
            next_angle = self._next_angle(target_angle, duration_ms, dt_ms)
            next_steps = self._angle_to_steps(next_angle)
            delta_steps = next_steps - self.current_steps

        if delta_steps:
            self.driver.move_steps(delta_steps, round(dt_ms))

        with self._lock:
            if delta_steps:
                self.current_steps = next_steps
            self.state.current_angle = next_angle
            self.state.target_angle = target_angle
            self.state.moving = next_angle != target_angle
            self.state.last_command = (
                f"UPDATE_STEPPER {self.config.id} {target_angle:.3f} {duration_ms}"
            )

    def _angle_to_steps(self, angle: float) -> int:
        steps_per_degree = (
            (self.config.steps_per_rev or 1)
            * self.config.microsteps
            * self.config.gear_ratio
            / 360.0
        )
        return round(angle * steps_per_degree)
