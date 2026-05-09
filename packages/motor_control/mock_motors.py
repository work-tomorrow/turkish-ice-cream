"""Tiny mock motors for local development."""

from __future__ import annotations

from motor_control.models import MotorKind, MotorState


class MockMotor:
    def __init__(self, id: str, kind: MotorKind, min_angle: float, max_angle: float, home_angle: float = 0.0):
        self.id = id
        self.kind = kind
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.home_angle = home_angle
        self.current_angle = home_angle
        self.target_angle = home_angle
        self.moving = False
        self.ready = True
        self.last_error = None

    def move_to(self, angle: float, *, duration_ms: int = 0) -> None:
        angle = max(self.min_angle, min(self.max_angle, float(angle)))
        self.current_angle = angle
        self.target_angle = angle

    def home(self) -> None:
        self.move_to(self.home_angle)

    def stop_now(self) -> None:
        self.moving = False

    def clear_stop(self) -> None:
        pass

    def snapshot(self) -> MotorState:
        return MotorState(self.id, self.kind, self.current_angle, self.target_angle)

    def cleanup(self) -> None:
        pass


def build_mock_motors() -> dict[str, MockMotor]:
    return {
        "m1": MockMotor("m1", MotorKind.STEPPER, -180.0, 180.0),
        "m2": MockMotor("m2", MotorKind.SERVO, 0.0, 180.0, 90.0),
        "m3": MockMotor("m3", MotorKind.SERVO, 0.0, 180.0, 90.0),
        "m4": MockMotor("m4", MotorKind.SERVO, 0.0, 180.0, 90.0),
    }
