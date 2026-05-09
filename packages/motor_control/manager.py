"""Public motor control API for the robot."""

from __future__ import annotations

from threading import Lock
from typing import Protocol

from motor_control.config import build_default_motors
from motor_control.models import MotorKind, MotorManagerState, MotorState
from motor_control.motors import ServoMotor, StepperMotor


class MotorLike(Protocol):
    id: str
    kind: MotorKind
    min_angle: float
    max_angle: float
    homed: bool
    requires_homing: bool

    def move_to(self, target_angle: float, *, duration_ms: int = 0) -> None: ...
    def home(self) -> None: ...
    def stop_now(self) -> None: ...
    def clear_stop(self) -> None: ...
    def set_current_angle_zero(self) -> None: ...
    def snapshot(self) -> MotorState: ...
    def shutdown(self) -> None: ...


def _keys_match_exactly(expected: set[str], actual: set[str]) -> None:
    missing = expected - actual
    extra = actual - expected
    if extra:
        raise ValueError(f"Unknown motor id(s): {', '.join(sorted(extra))}")
    if missing:
        raise ValueError(f"Missing target(s) for motor id(s): {', '.join(sorted(missing))}")


class MotorManager:
    """Simple synchronous motor manager.

    The upper layer should call set_target_angles() every control tick.
    This manager validates ids/ranges and immediately forwards targets to motors.
    """

    def __init__(
        self,
        motors: dict[str, MotorLike] | None = None,
        *,
        control_hz: float = 10.0,
        max_delta_per_tick: dict[str, float] | None = None,
    ) -> None:
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")

        self.control_hz = float(control_hz)
        self._motors: dict[str, MotorLike] = dict(motors) if motors is not None else build_default_motors()
        if not self._motors:
            raise ValueError("MotorManager expects at least one motor")

        self._lock = Lock()
        self._running = True
        self._emergency_stopped = False
        self._max_delta_per_tick = dict(max_delta_per_tick or {})

    @property
    def motor_ids(self) -> tuple[str, ...]:
        return tuple(self._motors.keys())

    def start(self) -> None:
        """Compatibility no-op: this manager sends commands synchronously."""

        with self._lock:
            self._running = True

    def stop(self) -> None:
        """Mark manager as stopped. It does not disable motors; use cleanup/emergency_stop for that."""

        with self._lock:
            self._running = False

    def home_all(self) -> None:
        for motor in self._motors.values():
            motor.home()

    def set_target_angles(self, targets: dict[str, float]) -> None:
        with self._lock:
            if self._emergency_stopped:
                raise RuntimeError("Emergency stop is active")
            if not self._running:
                raise RuntimeError("MotorManager is stopped")

            expected = set(self._motors.keys())
            _keys_match_exactly(expected, set(targets.keys()))

            safe_targets: dict[str, float] = {}
            for motor_id, raw_target in targets.items():
                motor = self._motors[motor_id]
                target = float(raw_target)
                if target < motor.min_angle or target > motor.max_angle:
                    raise ValueError(
                        f"Motor {motor_id} angle {target} outside [{motor.min_angle}, {motor.max_angle}]"
                    )
                if motor.requires_homing and not motor.homed:
                    raise RuntimeError(f"Motor {motor_id} must be homed before movement")

                max_delta = self._max_delta_per_tick.get(motor_id)
                if max_delta is not None:
                    current = motor.snapshot().target_angle
                    delta = target - current
                    if abs(delta) > max_delta:
                        target = current + max_delta * (1 if delta > 0 else -1)

                safe_targets[motor_id] = target

        for motor_id, target in safe_targets.items():
            self._motors[motor_id].move_to(target, duration_ms=0)

    def set_target(self, targets: dict[str, float], *, duration_ms: int = 0) -> None:
        """Compatibility wrapper for older callers."""

        self.set_target_angles(targets)

    def set_current_angle_zero(self, motor_id: str | None = None) -> None:
        if motor_id is None:
            for motor in self._motors.values():
                motor.set_current_angle_zero()
            return

        motor = self._motors.get(motor_id)
        if motor is None:
            raise ValueError(f"Unknown motor id: {motor_id}")
        motor.set_current_angle_zero()

    def get_state(self) -> MotorManagerState:
        with self._lock:
            running = self._running
            emergency_stopped = self._emergency_stopped

        return MotorManagerState(
            running=running,
            emergency_stopped=emergency_stopped,
            motors={motor_id: motor.snapshot() for motor_id, motor in self._motors.items()},
        )

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stopped = True
            self._running = False
        for motor in self._motors.values():
            motor.stop_now()

    def clear_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stopped = False
            self._running = True
        for motor in self._motors.values():
            motor.clear_stop()

    def reset_emergency_stop(self) -> None:
        self.clear_emergency_stop()

    def cleanup(self) -> None:
        with self._lock:
            self._running = False
        for motor in self._motors.values():
            motor.shutdown()
