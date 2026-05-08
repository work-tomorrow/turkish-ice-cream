"""MG995 servo angle controller."""

from __future__ import annotations

from motor_control.controllers.base import BaseController


class MG995ServoController(BaseController):
    """Translate angle targets into MG995 PWM angle commands."""

    def move_to(self, angle: float, duration_ms: int) -> None:
        self._ensure_ready()
        self.driver.set_angle(angle)
        with self._lock:
            self.state.current_angle = angle
            self.state.target_angle = angle
            self.state.moving = False
            self.state.last_command = (
                f"MOVE_SERVO {self.config.id} {angle:.3f} {duration_ms}"
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

        self.driver.set_angle(next_angle)

        with self._lock:
            self.state.current_angle = next_angle
            self.state.target_angle = target_angle
            self.state.moving = next_angle != target_angle
            self.state.last_command = (
                f"UPDATE_SERVO {self.config.id} {target_angle:.3f} {duration_ms}"
            )
