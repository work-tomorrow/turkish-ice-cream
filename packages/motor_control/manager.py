"""Simple motor control manager.

The manager is the safety/control layer between upper-level robot logic
and low-level GPIO motors.

Expected usage:
    manager = MotorManager()
    manager.set_target_angles({
        "m1": 0,
        "m2": 0,
        "m3": 90,
        "m4": 90,
    })
"""

from __future__ import annotations

from typing import Any

from motor_control.config import build_default_motors


class MotorManager:
    """Simple synchronous motor manager.

    Upper layer should call set_target_angles() periodically, e.g. every 100ms.

    This manager handles:
    - motor id completeness check
    - angle clamp
    - speed limiting
    - emergency stop
    - state reporting
    """

    def __init__(
        self,
        motors: dict[str, Any] | None = None,
        *,
        dt_s: float = 0.1,
        max_velocity_deg_s: dict[str, float] | None = None,
    ) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")

        self.motors = motors if motors is not None else build_default_motors()
        if not self.motors:
            raise ValueError("MotorManager needs at least one motor")

        self.dt_s = float(dt_s)

        # Default speed limits.
        # You can tune these later based on the real robot.
        self.max_velocity_deg_s = {
            "m1": 45.0,
            "m2": 180.0,
            "m3": 180.0,
            "m4": 180.0,
        }

        if max_velocity_deg_s is not None:
            self.max_velocity_deg_s.update(max_velocity_deg_s)

        # Manager-side target memory.
        # This is the value after clamp + speed limit.
        self.current_targets: dict[str, float] = {}
        self.zero_offsets: dict[str, float] = {}
        for motor_id, motor in self.motors.items():
            current_angle = float(
                getattr(motor, "current_angle", getattr(motor, "home_angle", 0.0))
            )
            self.current_targets[motor_id] = current_angle
            self.zero_offsets[motor_id] = 0.0

    @property
    def motor_ids(self) -> tuple[str, ...]:
        return tuple(self.motors.keys())

    def set_target_angles(self, targets: dict[str, float]) -> None:
        """Set target angles for all motors.

        targets must contain exactly the same motor ids as self.motors.
        Example:
            {
                "m1": 0,
                "m2": 0,
                "m3": 90,
                "m4": 90,
            }
        """

        self._check_motor_ids(targets)

        safe_targets: dict[str, float] = {}

        for motor_id, raw_angle in targets.items():
            motor = self.motors[motor_id]
            offset = self.zero_offsets[motor_id]

            # 1. Clamp angle into motor range.
            target = self._clamp(
                float(raw_angle),
                float(motor.min_angle) - offset,
                float(motor.max_angle) - offset,
            )

            # 2. Speed limit.
            current = self.current_targets[motor_id]
            max_velocity = float(self.max_velocity_deg_s.get(motor_id, 180.0))
            max_delta = max_velocity * self.dt_s

            delta = target - current

            if delta > max_delta:
                target = current + max_delta
            elif delta < -max_delta:
                target = current - max_delta

            safe_targets[motor_id] = target

        # 3. Send safe targets to motors.
        for motor_id, target in safe_targets.items():
            physical_target = target + self.zero_offsets[motor_id]
            self.motors[motor_id].move_to(physical_target)
            self.current_targets[motor_id] = target

    def set_target(self, targets: dict[str, float], duration_ms: int = 0) -> None:
        """Compatibility wrapper.
        """

        self.set_target_angles(targets)

    def start(self) -> None:
        """Compatibility no-op for the old manager lifecycle."""

    def stop(self) -> None:
        """Compatibility alias for cleanup()."""

        self.cleanup()

    def home_all(self) -> None:
        """Move all motors to their home angle."""

        targets = {}
        for motor_id, motor in self.motors.items():
            home_angle = float(getattr(motor, "home_angle", 0.0))
            targets[motor_id] = home_angle - self.zero_offsets[motor_id]

        self.set_target_angles(targets)

    def emergency_stop(self) -> None:
        """Immediately stop all motors and freeze targets at the current pose."""

        for motor_id, motor in self.motors.items():
            if hasattr(motor, "stop_now"):
                motor.stop_now()
            elif hasattr(motor, "stop"):
                motor.stop()
            self.current_targets[motor_id] = self._logical_current_angle(motor_id, motor)

    def clear_emergency_stop(self) -> None:
        """Compatibility no-op for the old clear operation."""

    def reset_emergency_stop(self) -> None:
        """Alias for clear_emergency_stop()."""

        self.clear_emergency_stop()

    def get_state(self) -> dict[str, Any]:
        """Return simple manager + motor state."""

        return {
            "emergency_stopped": False,
            "dt_s": self.dt_s,
            "current_targets": dict(self.current_targets),
            "motors": {
                motor_id: self._get_motor_state(motor_id, motor)
                for motor_id, motor in self.motors.items()
            },
        }

    def set_current_angle_zero(self, motor_id: str | None = None) -> None:
        """Treat the current position as logical zero for one or all motors."""

        motor_ids = [motor_id] if motor_id is not None else list(self.motors)
        for current_motor_id in motor_ids:
            if current_motor_id not in self.motors:
                raise ValueError(f"Unknown motor id: {current_motor_id}")

        for current_motor_id in motor_ids:
            motor = self.motors[current_motor_id]
            if hasattr(motor, "set_current_angle_zero"):
                motor.set_current_angle_zero()
                self.zero_offsets[current_motor_id] = 0.0
            else:
                physical_current = float(getattr(motor, "current_angle", 0.0))
                self.zero_offsets[current_motor_id] += physical_current
            self.current_targets[current_motor_id] = 0.0

    def cleanup(self) -> None:
        """Shutdown all motors and release GPIO resources."""

        for motor in self.motors.values():
            if hasattr(motor, "cleanup"):
                motor.cleanup()
            elif hasattr(motor, "shutdown"):
                motor.shutdown()

    def _check_motor_ids(self, targets: dict[str, float]) -> None:
        expected = set(self.motors.keys())
        actual = set(targets.keys())

        missing = expected - actual
        extra = actual - expected

        if missing or extra:
            raise ValueError(
                f"Motor ids mismatch: missing={sorted(missing)}, extra={sorted(extra)}"
            )

    def _get_motor_state(self, motor_id: str, motor: Any) -> dict[str, Any]:
        if hasattr(motor, "get_state"):
            snapshot = motor.get_state()
            return self._normalize_state(motor_id, snapshot)

        if hasattr(motor, "snapshot"):
            snapshot = motor.snapshot()
            if hasattr(snapshot, "__dict__"):
                snapshot = dict(snapshot.__dict__)
            return self._normalize_state(motor_id, snapshot)

        snapshot = {
            "id": getattr(motor, "id", None),
            "current_angle": getattr(motor, "current_angle", None),
            "target_angle": getattr(motor, "target_angle", None),
            "min_angle": getattr(motor, "min_angle", None),
            "max_angle": getattr(motor, "max_angle", None),
            "ready": getattr(motor, "ready", None),
            "last_error": getattr(motor, "last_error", None),
        }
        return self._normalize_state(motor_id, snapshot)

    def _normalize_state(self, motor_id: str, snapshot: Any) -> dict[str, Any]:
        if not isinstance(snapshot, dict):
            return snapshot

        normalized = dict(snapshot)
        offset = self.zero_offsets[motor_id]

        current_angle = normalized.get("current_angle")
        if current_angle is not None:
            normalized["current_angle"] = float(current_angle) - offset

        target_angle = normalized.get("target_angle")
        if target_angle is not None:
            normalized["target_angle"] = float(target_angle) - offset

        min_angle = normalized.get("min_angle")
        if min_angle is None:
            min_angle = getattr(self.motors[motor_id], "min_angle", None)
        if min_angle is not None:
            normalized["min_angle"] = float(min_angle) - offset

        max_angle = normalized.get("max_angle")
        if max_angle is None:
            max_angle = getattr(self.motors[motor_id], "max_angle", None)
        if max_angle is not None:
            normalized["max_angle"] = float(max_angle) - offset

        return normalized

    def _logical_current_angle(self, motor_id: str, motor: Any) -> float:
        current_angle = float(getattr(motor, "current_angle", 0.0))
        return current_angle - self.zero_offsets[motor_id]

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))
