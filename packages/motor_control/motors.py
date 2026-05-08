"""Minimal motor implementations for the robot."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from time import sleep
from typing import Any, Callable


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, value))


@dataclass(frozen=True)
class ServoPwmConfig:
    signal_pin: int | None
    frequency_hz: float = 50.0
    min_pulse_us: float = 500.0
    max_pulse_us: float = 2500.0


class ServoMotor:
    def __init__(
        self,
        *,
        motor_id: str,
        min_angle: float,
        max_angle: float,
        max_velocity_deg_s: float,
        pwm: ServoPwmConfig,
        driver: str = "mock",
    ) -> None:
        self.id = motor_id
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.max_velocity_deg_s = float(max_velocity_deg_s)
        self.commanded_angle = float(_clamp(0.0, self.min_angle, self.max_angle))
        self._pwm_cfg = pwm
        self._driver = driver
        self._lock = Lock()
        self._stopped = False

        # Mock-visible output for tests / local dev.
        self.last_pwm_duty_cycle: float | None = None
        self._gpio: Any | None = None
        self._pwm: Any | None = None

    @property
    def estimated_angle(self) -> None:
        return None

    def set_target_angle(self, angle: float) -> None:
        with self._lock:
            if self._stopped:
                return
            clamped = _clamp(float(angle), self.min_angle, self.max_angle)
            self.commanded_angle = clamped
            self._apply_pwm_for_angle(clamped)

    def stop_now(self) -> None:
        with self._lock:
            self._stopped = True
            self.last_pwm_duty_cycle = 0.0
            self._stop_pwm_locked()

    def clear_stop(self) -> None:
        with self._lock:
            self._stopped = False

    def shutdown(self) -> None:
        with self._lock:
            self._stopped = True
            self._stop_pwm_locked()

    def _apply_pwm_for_angle(self, angle: float) -> None:
        duty = self._angle_to_duty_cycle(angle)
        if self._driver == "mock":
            self.last_pwm_duty_cycle = duty
            return

        pwm = self._ensure_pwm()
        pwm.ChangeDutyCycle(duty)
        self.last_pwm_duty_cycle = duty

    def _ensure_pwm(self) -> Any:
        if self._pwm is not None:
            return self._pwm

        # Hardware integration intentionally minimal: keep GPIO imports local.
        try:
            import RPi.GPIO as GPIO  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("Servo motor requires RPi.GPIO on Raspberry Pi") from exc

        if self._pwm_cfg.signal_pin is None:
            raise ValueError("Servo motor needs signal_pin for hardware driver")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pwm_cfg.signal_pin, GPIO.OUT)
        self._gpio = GPIO
        self._pwm = GPIO.PWM(self._pwm_cfg.signal_pin, self._pwm_cfg.frequency_hz)
        self._pwm.start(0.0)
        return self._pwm

    def _stop_pwm_locked(self) -> None:
        if self._pwm is None:
            return
        try:
            self._pwm.ChangeDutyCycle(0.0)
            self._pwm.stop()
        finally:
            self._pwm = None

    def _angle_to_duty_cycle(self, angle: float) -> float:
        span = self.max_angle - self.min_angle
        if span <= 0:
            return 0.0
        progress = (angle - self.min_angle) / span
        pulse_us = self._pwm_cfg.min_pulse_us + progress * (
            self._pwm_cfg.max_pulse_us - self._pwm_cfg.min_pulse_us
        )
        period_us = 1_000_000.0 / self._pwm_cfg.frequency_hz
        return pulse_us / period_us * 100.0


class StepperMotor:
    def __init__(
        self,
        *,
        motor_id: str,
        min_angle: float,
        max_angle: float,
        max_velocity_deg_s: float,
        steps_per_rev: int,
        microsteps: int = 1,
        gear_ratio: float = 1.0,
        driver: str = "mock",
        step_fn: Callable[[int], None] | None = None,
        step_pin: int | None = None,
        dir_pin: int | None = None,
        enable_pin: int | None = None,
    ) -> None:
        self.id = motor_id
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.max_velocity_deg_s = float(max_velocity_deg_s)
        self.steps_per_rev = int(steps_per_rev)
        self.microsteps = int(microsteps)
        self.gear_ratio = float(gear_ratio)
        self._driver = driver

        self.commanded_angle = float(_clamp(0.0, self.min_angle, self.max_angle))
        self.current_steps = 0
        self.target_steps = 0

        self._lock = Lock()
        self._stop_event = Event()
        self._pause_event = Event()  # e-stop gate
        self._step_fn = step_fn or self._default_step_fn
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin

        self._thread = Thread(
            target=self._run_pulse_loop,
            name=f"stepper-pulse-{motor_id}",
            daemon=True,
        )
        self._thread.start()

    @property
    def estimated_angle(self) -> float:
        with self._lock:
            return self._steps_to_angle(self.current_steps)

    def set_target_angle(self, angle: float) -> None:
        with self._lock:
            clamped = _clamp(float(angle), self.min_angle, self.max_angle)
            self.commanded_angle = clamped
            self.target_steps = self._angle_to_steps(clamped)

    def stop_now(self) -> None:
        self._pause_event.set()

    def clear_stop(self) -> None:
        self._pause_event.clear()

    def shutdown(self) -> None:
        self._stop_event.set()

    def _run_pulse_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._pause_event.is_set():
                sleep(0.01)
                continue

            with self._lock:
                delta = self.target_steps - self.current_steps
                if delta == 0:
                    sleep(0.002)
                    continue
                direction = 1 if delta > 0 else -1
                max_steps_per_s = self._max_steps_per_s()
                step_delay_s = 1.0 / max_steps_per_s if max_steps_per_s > 0 else 0.01

            # Execute one pulse (outside lock).
            try:
                self._step_fn(direction)
            except Exception:
                # Keep it minimal: treat step failure as a stop.
                self._pause_event.set()
                sleep(0.05)
                continue

            with self._lock:
                self.current_steps += direction

            sleep(max(0.0005, step_delay_s))

    def _max_steps_per_s(self) -> float:
        # Map max_velocity_deg_s to max steps/s.
        steps_per_degree = (
            self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0
        )
        return max(1.0, abs(self.max_velocity_deg_s) * abs(steps_per_degree))

    def _angle_to_steps(self, angle: float) -> int:
        steps_per_degree = (
            self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0
        )
        return int(round(angle * steps_per_degree))

    def _steps_to_angle(self, steps: int) -> float:
        steps_per_degree = (
            self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0
        )
        if steps_per_degree == 0:
            return 0.0
        return steps / steps_per_degree

    def _default_step_fn(self, direction: int) -> None:
        # Mock driver: do nothing besides counting steps.
        if self._driver == "mock":
            return

        # Hardware driver
        if self.step_pin is None or self.dir_pin is None:
            raise ValueError(f"Stepper {self.id} needs step_pin and dir_pin for hardware driver")

        try:
            import RPi.GPIO as GPIO  # type: ignore
        except ImportError as exc:
            raise RuntimeError(f"Stepper {self.id} requires RPi.GPIO on Raspberry Pi") from exc

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        if self.enable_pin is not None:
            GPIO.setup(self.enable_pin, GPIO.OUT)
            GPIO.output(self.enable_pin, GPIO.LOW)  # Enable (active low)

        # Set direction
        GPIO.output(self.dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)

        # Pulse step
        GPIO.output(self.step_pin, GPIO.HIGH)
        sleep(0.000001)  # 1us pulse
        GPIO.output(self.step_pin, GPIO.LOW)
