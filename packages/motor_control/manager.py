"""Minimal public motor control API (no planner/queue)."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from motor_control.config import build_default_motors
from motor_control.motors import ServoMotor, StepperMotor


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, value))


def _keys_match_exactly(expected: set[str], actual: set[str]) -> None:
    missing = expected - actual
    extra = actual - expected
    if missing or extra:
        if extra:
            unknown = ", ".join(sorted(extra))
            raise ValueError(f"Unknown motor id(s): {unknown}")
        missing_ids = ", ".join(sorted(missing))
        raise ValueError(f"Missing target(s) for motor id(s): {missing_ids}")


@dataclass(frozen=True)
class _LimitedTarget:
    raw_target: float
    safe_target: float


class MotorManager:
    def __init__(
        self,
        motors: dict[str, ServoMotor | StepperMotor] | None = None,
        *,
        dt_s: float = 0.1,
    ) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")

        self.dt_s = float(dt_s)
        self._motors: dict[str, ServoMotor | StepperMotor] = (
            dict(motors) if motors is not None else build_default_motors()
        )
        if not self._motors:
            raise ValueError("MotorManager expects at least one motor")

        self._lock = Lock()
        self._emergency_stopped = False

    @property
    def motor_ids(self) -> tuple[str, ...]:
        return tuple(self._motors.keys())

    def start(self) -> None:
        # Start any background threads if needed (steppers have their own)
        pass

    def stop(self) -> None:
        # Stop any background threads
        pass

    def home_all(self) -> None:
        with self._lock:
            for motor in self._motors.values():
                if isinstance(motor, StepperMotor):
                    motor.commanded_angle = 0.0
                    motor.current_steps = 0
                    motor.target_steps = 0

    def set_target(self, targets: dict[str, float], duration_ms: int) -> None:
        self.set_target_angles(targets)

    def set_current_angle_zero(self) -> None:
        with self._lock:
            for motor in self._motors.values():
                motor.commanded_angle = 0.0
                if isinstance(motor, StepperMotor):
                    motor.current_steps = 0
                    motor.target_steps = 0

    def set_target_angles(self, targets: dict[str, float]) -> None:
        with self._lock:
            if self._emergency_stopped:
                raise RuntimeError("Emergency stop is active")
            expected = set(self._motors.keys())
            _keys_match_exactly(expected, set(targets.keys()))

            # Compute safe targets under the manager lock to keep commanded angles consistent.
            limited: dict[str, _LimitedTarget] = {}
            for motor_id, raw in targets.items():
                motor = self._motors[motor_id]

                raw_target = _clamp(float(raw), motor.min_angle, motor.max_angle)
                max_delta = float(motor.max_velocity_deg_s) * self.dt_s
                delta = raw_target - float(motor.commanded_angle)
                limited_delta = _clamp(delta, -max_delta, max_delta)
                safe_target = float(motor.commanded_angle) + limited_delta
                safe_target = _clamp(safe_target, motor.min_angle, motor.max_angle)
                limited[motor_id] = _LimitedTarget(raw_target=raw_target, safe_target=safe_target)

        # Apply targets outside the manager lock (avoid holding lock across hardware IO).
        for motor_id, item in limited.items():
            self._motors[motor_id].set_target_angle(item.safe_target)

    def get_state(self) -> dict:
        with self._lock:
            motors_payload: dict[str, dict] = {}
            for motor_id, motor in self._motors.items():
                base = {
                    "id": motor_id,
                    "min_angle": motor.min_angle,
                    "max_angle": motor.max_angle,
                    "max_velocity_deg_s": motor.max_velocity_deg_s,
                    "commanded_angle": motor.commanded_angle,
                    "estimated_angle": motor.estimated_angle,
                }
                if isinstance(motor, StepperMotor):
                    base.update(
                        {
                            "kind": "stepper",
                            "current_steps": motor.current_steps,
                            "target_steps": motor.target_steps,
                        }
                    )
                else:
                    base.update({"kind": "servo"})
                motors_payload[motor_id] = base

            return {
                "emergency_stopped": self._emergency_stopped,
                "dt_s": self.dt_s,
                "motors": motors_payload,
            }

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stopped = True
        for motor in self._motors.values():
            motor.stop_now()

    def clear_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stopped = False
        for motor in self._motors.values():
            motor.clear_stop()

    # Alias: some callers may prefer reset_* wording.
    def reset_emergency_stop(self) -> None:
        self.clear_emergency_stop()

    def cleanup(self) -> None:
        # Best-effort shutdown for hardware resources.
        for motor in self._motors.values():
            if isinstance(motor, StepperMotor | ServoMotor):
                motor.shutdown()
