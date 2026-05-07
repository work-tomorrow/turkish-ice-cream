"""PM30225 coreless motor controller."""

from __future__ import annotations

from motor_control.controllers.base import BaseController


class PM30225Controller(BaseController):
    """Translate angle targets into PM30225 serial commands."""

    def home(self) -> None:
        self.driver.send_command(f"HOME {self.config.id}")
        super().home()

    def move_to(self, angle: float, duration_ms: int) -> None:
        self._ensure_ready()
        command = f"SET_TARGET {self.config.id} {angle:.3f} {duration_ms}"
        self.driver.send_command(command)
        self.state.current_angle = angle
        self.state.target_angle = angle
        self.state.last_command = command

    def update(
        self,
        target_angle: float,
        duration_ms: int,
        command_id: int,
        dt_ms: float,
    ) -> None:
        with self._lock:
            self._ensure_ready()
            is_new_command = command_id != self._active_command_id
            self._begin_command_if_needed(target_angle, command_id)

        if is_new_command:
            command = f"SET_TARGET {self.config.id} {target_angle:.3f} {duration_ms}"
            self.driver.send_command(command)
        else:
            command = None

        with self._lock:
            self.state.current_angle = self._next_angle(
                target_angle,
                duration_ms,
                dt_ms,
            )
            self.state.target_angle = target_angle
            self.state.moving = self.state.current_angle != target_angle
            if command is not None:
                self.state.last_command = command
