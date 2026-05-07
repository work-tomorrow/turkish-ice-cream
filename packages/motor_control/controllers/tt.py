"""TT motor angle controller."""

from __future__ import annotations

from motor_control.controllers.base import BaseController


class TTController(BaseController):
    """Translate angle targets into TT motor commands.

    This is approximate until encoder feedback is decided and implemented.
    """

    def move_to(self, angle: float, duration_ms: int) -> None:
        self._ensure_ready()
        self.driver.move_to_angle_approx(angle, duration_ms)
        self.state.current_angle = angle
        self.state.target_angle = angle
        self.state.approximate = True
        self.state.last_command = f"MOVE_TT {self.config.id} {angle:.3f} {duration_ms}"

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

        self.driver.move_to_angle_approx(next_angle, round(dt_ms))

        with self._lock:
            self.state.current_angle = next_angle
            self.state.target_angle = target_angle
            self.state.moving = next_angle != target_angle
            self.state.approximate = True
            self.state.last_command = (
                f"UPDATE_TT {self.config.id} {target_angle:.3f} {duration_ms}"
            )
