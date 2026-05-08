"""Public motor control API for the robot."""

from __future__ import annotations

from threading import Condition, Event, Lock, Thread
from time import monotonic
from typing import Callable

from motor_control.config import build_default_motors
from motor_control.models import MotorManagerState
from motor_control.motors import ServoMotor, StepperMotor


def _keys_match_exactly(expected: set[str], actual: set[str]) -> None:
    missing = expected - actual
    extra = actual - expected
    if missing or extra:
        if extra:
            unknown = ", ".join(sorted(extra))
            raise ValueError(f"Unknown motor id(s): {unknown}")
        missing_ids = ", ".join(sorted(missing))
        raise ValueError(f"Missing target(s) for motor id(s): {missing_ids}")


class MotorManager:
    def __init__(
        self,
        motors: dict[str, ServoMotor | StepperMotor] | None = None,
        *,
        control_hz: float = 50.0,
        tick_timing_hook: Callable[[float], None] | None = None,
    ) -> None:
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")

        self.control_hz = float(control_hz)
        self._motors: dict[str, ServoMotor | StepperMotor] = (
            dict(motors) if motors is not None else build_default_motors()
        )
        if not self._motors:
            raise ValueError("MotorManager expects at least one motor")

        self._lock = Lock()
        self._command_condition = Condition(self._lock)
        self._stop_event = Event()
        self._worker: Thread | None = None
        self._running = False
        self._emergency_stopped = False
        self._pending_targets: dict[str, float] | None = None
        self._pending_duration_ms = 0
        self._tick_timing_hook = tick_timing_hook

    @property
    def motor_ids(self) -> tuple[str, ...]:
        return tuple(self._motors.keys())

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._stop_event.clear()
            self._running = True
            self._worker = Thread(
                target=self._run_worker,
                name="motor-manager-worker",
                daemon=True,
            )
            self._worker.start()
            self._command_condition.notify_all()

    def stop(self) -> None:
        worker: Thread | None = None
        with self._lock:
            if not self._running and self._worker is None:
                return
            self._running = False
            self._stop_event.set()
            self._command_condition.notify_all()
            worker = self._worker
            self._worker = None

        if worker is not None:
            worker.join(timeout=1.0)

    def home_all(self) -> None:
        for motor in self._motors.values():
            motor.home()

    def set_target(self, targets: dict[str, float], *, duration_ms: int) -> None:
        with self._command_condition:
            if self._emergency_stopped:
                raise RuntimeError("Emergency stop is active")

            expected = set(self._motors.keys())
            _keys_match_exactly(expected, set(targets.keys()))

            for motor_id, raw_target in targets.items():
                motor = self._motors[motor_id]
                if raw_target < motor.min_angle or raw_target > motor.max_angle:
                    raise ValueError(
                        f"Motor {motor_id} angle {raw_target} outside "
                        f"[{motor.min_angle}, {motor.max_angle}]"
                    )

            for motor_id, motor in self._motors.items():
                if isinstance(motor, StepperMotor) and motor.requires_homing and not motor.homed:
                    raise RuntimeError(f"Motor {motor_id} must be homed before movement")

            self._pending_targets = {motor_id: float(target) for motor_id, target in targets.items()}
            self._pending_duration_ms = int(duration_ms)
            self._command_condition.notify_all()

    def set_target_angles(self, targets: dict[str, float]) -> None:
        self.set_target(targets, duration_ms=0)

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

        return MotorManagerState(
            running=running,
            motors={motor_id: motor.snapshot() for motor_id, motor in self._motors.items()},
        )

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stopped = True
        for motor in self._motors.values():
            motor.stop_now()
        self.stop()

    def clear_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stopped = False
        for motor in self._motors.values():
            motor.clear_stop()

    def reset_emergency_stop(self) -> None:
        self.clear_emergency_stop()

    def cleanup(self) -> None:
        self.stop()
        for motor in self._motors.values():
            motor.shutdown()

    def _run_worker(self) -> None:
        tick_period_s = 1.0 / self.control_hz
        while not self._stop_event.is_set():
            started_at = monotonic()

            with self._command_condition:
                if self._pending_targets is None and not self._stop_event.is_set():
                    self._command_condition.wait(timeout=tick_period_s)

                if self._stop_event.is_set():
                    break

                pending_targets = self._pending_targets
                pending_duration_ms = self._pending_duration_ms
                self._pending_targets = None

            if pending_targets is not None:
                for motor_id, target in pending_targets.items():
                    self._motors[motor_id].move_to(target, duration_ms=pending_duration_ms)

            elapsed_ms = max(0.0, (monotonic() - started_at) * 1000.0)
            if self._tick_timing_hook is not None:
                self._tick_timing_hook(elapsed_ms)
