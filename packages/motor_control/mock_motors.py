"""Mock motor implementations for local development/tests.

This file is intentionally separate from the real GPIO driver path.
Production code should import from motor_control.motors and motor_control.config.
"""

from __future__ import annotations

from motor_control.models import MotorConfig, MotorKind, MotorState


class MockMotor:
    def __init__(self, config: MotorConfig) -> None:
        self.id = config.id
        self.kind = config.kind
        self.min_angle = float(config.min_angle)
        self.max_angle = float(config.max_angle)
        self.home_angle = float(config.home_angle)
        self.requires_homing = bool(config.requires_homing)
        self.current_angle = float(config.home_angle)
        self.target_angle = float(config.home_angle)
        self.homed = not self.requires_homing
        self.ready = True
        self.moving = False
        self.last_command: str | None = None
        self.last_error: str | None = None

    def snapshot(self) -> MotorState:
        return MotorState(
            id=self.id,
            kind=self.kind,
            current_angle=self.current_angle,
            target_angle=self.target_angle,
            hw_angle=self.current_angle,
            moving=self.moving,
            homed=self.homed,
            ready=self.ready,
            last_command=self.last_command,
            last_error=self.last_error,
        )

    def home(self) -> None:
        self.current_angle = self.home_angle
        self.target_angle = self.home_angle
        self.homed = True
        self.moving = False
        self.last_command = "MOCK_HOME"

    def move_to(self, target_angle: float, *, duration_ms: int = 0) -> None:
        if self.requires_homing and not self.homed:
            self.last_error = "Motor must be homed before movement"
            return
        self.target_angle = max(self.min_angle, min(self.max_angle, float(target_angle)))
        self.current_angle = self.target_angle
        self.moving = False
        self.last_command = f"MOCK_MOVE angle={self.target_angle:.3f} duration_ms={int(duration_ms)}"

    def stop_now(self) -> None:
        self.moving = False
        self.last_command = "MOCK_STOP"

    def clear_stop(self) -> None:
        self.last_error = None

    def set_current_angle_zero(self) -> None:
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.last_command = "MOCK_ZERO"

    def shutdown(self) -> None:
        self.moving = False
        self.last_command = "MOCK_SHUTDOWN"


MockServoMotor = MockMotor
MockStepperMotor = MockMotor


def build_mock_motors() -> dict[str, MockMotor]:
    configs = [
        MotorConfig("m1", MotorKind.STEPPER, -180.0, 180.0, requires_homing=False),
        MotorConfig("m2", MotorKind.STEPPER, -180.0, 180.0, requires_homing=False),
        MotorConfig("m3", MotorKind.SERVO, 0.0, 180.0, home_angle=90.0),
        MotorConfig("m4", MotorKind.SERVO, 0.0, 180.0, home_angle=90.0),
    ]
    return {config.id: MockMotor(config) for config in configs}
