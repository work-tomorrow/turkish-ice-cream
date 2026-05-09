"""Minimal Raspberry Pi GPIO motor drivers.

Goal: make the motors move with the smallest useful abstraction.
Pin numbers are BCM GPIO numbers, not physical board pin numbers.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from motor_control.models import MotorKind, MotorState

_GPIO_READY = False


def _gpio() -> Any:
    global _GPIO_READY
    try:
        import RPi.GPIO as GPIO  # type: ignore
    except ImportError as exc:
        raise RuntimeError("RPi.GPIO is required on Raspberry Pi") from exc

    if not _GPIO_READY:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        _GPIO_READY = True

    return GPIO


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ServoMotor:
    """Simple positional servo driver for SG995/SG996R/MG995/MG996R."""

    def __init__(
        self,
        id: str,
        pin: int,
        *,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        home_angle: float = 90.0,
        min_pulse_us: float = 500.0,
        max_pulse_us: float = 2500.0,
        frequency_hz: float = 50.0,
    ) -> None:
        self.id = id
        self.kind = MotorKind.SERVO
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.home_angle = home_angle
        self.current_angle = home_angle
        self.target_angle = home_angle
        self.moving = False
        self.ready = True
        self.last_error: str | None = None

        self._min_pulse_us = min_pulse_us
        self._max_pulse_us = max_pulse_us
        self._frequency_hz = frequency_hz

        GPIO = _gpio()
        GPIO.setup(self.pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self.pin, self._frequency_hz)
        self._pwm.start(0)

    def move_to(self, angle: float, *, duration_ms: int = 0) -> None:
        angle = _clamp(float(angle), self.min_angle, self.max_angle)
        duty = 2.5 + (angle / 180.0) * 10.0

        try:
            self._pwm.ChangeDutyCycle(duty)
            self.current_angle = angle
            self.target_angle = angle
            self.moving = False
            self.ready = True
            self.last_error = None
        except Exception as exc:
            self.ready = False
            self.last_error = str(exc)
            raise

    def home(self) -> None:
        self.move_to(self.home_angle)

    def stop_now(self) -> None:
        self.target_angle = self.current_angle
        self.moving = False
        self.release()

    def release(self) -> None:
        self._pwm.ChangeDutyCycle(0)

    def snapshot(self) -> MotorState:
        return MotorState(
            id=self.id,
            kind=self.kind,
            current_angle=self.current_angle,
            target_angle=self.target_angle,
            moving=self.moving,
            ready=self.ready,
            last_error=self.last_error,
        )

    def cleanup(self) -> None:
        GPIO = _gpio()
        self.release()
        self._pwm.stop()
        GPIO.cleanup(self.pin)

    # def _angle_to_duty(self, angle: float) -> float:
    #     progress = (angle - self.min_angle) / (
    #         self.max_angle - self.min_angle
    #     )
    #     pulse_us = self._min_pulse_us + progress * (
    #         self._max_pulse_us - self._min_pulse_us
    #     )
    #     period_us = 1_000_000.0 / self._frequency_hz
    #     return pulse_us / period_us * 100.0


class StepperMotor:
    """Very simple STEP/DIR stepper driver."""

    def __init__(
        self,
        id: str,
        step_pin: int,
        dir_pin: int,
        *,
        enable_pin: int | None = None,
        enable_active_high: bool = True,
        min_angle: float = -180.0,
        max_angle: float = 180.0,
        home_angle: float = 0.0,
        steps_per_rev: int = 200,
        steps_per_second: float = 400.0,
    ) -> None:
        self.id = id
        self.kind = MotorKind.STEPPER
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self._enable_active_high = bool(enable_active_high)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.home_angle = float(home_angle)
        self.current_angle = self.home_angle
        self.target_angle = self.home_angle
        self.moving = False
        self.ready = True
        self.last_error: str | None = None

        self._steps_per_degree = int(steps_per_rev) / 360.0
        if self._steps_per_degree <= 0:
            raise ValueError(
                "steps_per_rev, microsteps, and gear_ratio must be positive"
            )
        self._min_steps = math.ceil(self.min_angle * self._steps_per_degree)
        self._max_steps = math.floor(self.max_angle * self._steps_per_degree)
        if self._min_steps > self._max_steps:
            raise ValueError("Stepper angle range is smaller than one reachable step")

        self._steps_per_second = max(1.0, float(steps_per_second))
        self._half_step_delay_s = 1.0 / (self._steps_per_second * 2.0)
        self._current_steps = self._angle_to_steps(self.home_angle)
        self._target_steps = self._current_steps
        self._sync_angles_from_steps()
        self._closed = False
        self._stop_requested = False
        self._condition = threading.Condition()

        GPIO = _gpio()
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.dir_pin, GPIO.LOW)
        if self.enable_pin is not None:
            GPIO.setup(self.enable_pin, GPIO.OUT)
            self._enable_driver()
            self._disable_driver()

        self._worker = threading.Thread(
            target=self._run_worker,
            name=f"{self.id}-stepper",
            daemon=True,
        )
        self._worker.start()

    def move_to(self, angle: float, *, duration_ms: int = 0) -> None:
        with self._condition:
            if self._closed:
                raise RuntimeError(f"{self.id} has been cleaned up")

            self._target_steps = self._angle_to_steps(angle)
            self.target_angle = self._steps_to_angle(self._target_steps)
            self._stop_requested = False
            self.moving = self._target_steps != self._current_steps
            self.ready = True
            self.last_error = None
            self._condition.notify_all()

    def home(self) -> None:
        self.move_to(self.home_angle)

    def stop_now(self) -> None:
        with self._condition:
            self._stop_requested = True
            self._target_steps = self._current_steps
            self._sync_angles_from_steps()
            self.moving = False
            self._condition.notify_all()

    def snapshot(self) -> MotorState:
        with self._condition:
            return MotorState(
                id=self.id,
                kind=self.kind,
                current_angle=self.current_angle,
                target_angle=self.target_angle,
                moving=self.moving,
                ready=self.ready,
                last_error=self.last_error,
            )

    def cleanup(self) -> None:
        with self._condition:
            self._closed = True
            self._target_steps = self._current_steps
            self.moving = False
            self._condition.notify_all()

        self._worker.join(timeout=1.0)

        GPIO = _gpio()
        GPIO.output(self.step_pin, GPIO.LOW)
        self._disable_driver()
        GPIO.cleanup(self.step_pin)
        GPIO.cleanup(self.dir_pin)
        if self.enable_pin is not None:
            GPIO.cleanup(self.enable_pin)

    def _run_worker(self) -> None:
        while True:
            with self._condition:
                while not self._closed and self._current_steps == self._target_steps:
                    self.moving = False
                    self._condition.wait()

                if self._closed:
                    return

                direction = 1 if self._target_steps > self._current_steps else -1
                self.moving = True

            try:
                self._enable_driver()
                self._pulse(direction)
            except Exception as exc:
                with self._condition:
                    self.ready = False
                    self.last_error = str(exc)
                    self._target_steps = self._current_steps
                    self._sync_angles_from_steps()
                    self.moving = False
                    self._stop_requested = False
                    self._condition.notify_all()
                self._disable_driver()
                continue

            should_disable = False
            with self._condition:
                self._current_steps += direction
                self.current_angle = self._steps_to_angle(self._current_steps)

                if self._stop_requested:
                    self._target_steps = self._current_steps
                    self._sync_angles_from_steps()
                    self._stop_requested = False

                self.moving = self._current_steps != self._target_steps
                if not self.moving:
                    self._sync_angles_from_steps()
                    should_disable = True

                self.ready = True
                self.last_error = None
                self._condition.notify_all()

            if should_disable:
                self._disable_driver()

    def _pulse(self, direction: int) -> None:
        GPIO = _gpio()
        GPIO.output(self.dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(self._half_step_delay_s)
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(self._half_step_delay_s)

    def _angle_to_steps(self, angle: float) -> int:
        angle = _clamp(float(angle), self.min_angle, self.max_angle)
        steps = self._round_steps(angle * self._steps_per_degree)
        return steps

    def _steps_to_angle(self, steps: int) -> float:
        return float(steps) / self._steps_per_degree

    def _sync_angles_from_steps(self) -> None:
        self.current_angle = self._steps_to_angle(self._current_steps)
        self.target_angle = self._steps_to_angle(self._target_steps)

    @staticmethod
    def _round_steps(value: float) -> int:
        if value >= 0:
            return math.floor(value + 0.5)
        return math.ceil(value - 0.5)

    def _enable_driver(self) -> None:
        if self.enable_pin is None:
            return

        GPIO = _gpio()
        GPIO.output(
            self.enable_pin,
            GPIO.HIGH if self._enable_active_high else GPIO.LOW,
        )

    def _disable_driver(self) -> None:
        if self.enable_pin is None:
            return

        GPIO = _gpio()
        GPIO.output(
            self.enable_pin,
            GPIO.LOW if self._enable_active_high else GPIO.HIGH,
        )
