"""Motor implementations for mock and Raspberry Pi GPIO drivers."""

from __future__ import annotations

from threading import Event, Lock, Thread, Timer
from time import sleep

from motor_control.models import MotorConfig, MotorKind, MotorState

DEFAULT_SERVO_FREQUENCY_HZ = 50.0
DEFAULT_SERVO_MIN_PULSE_US = 500.0
DEFAULT_SERVO_MAX_PULSE_US = 2500.0
DEFAULT_SERVO_HOLD_TIME_S = 0.8
DEFAULT_STEPPER_MAX_VELOCITY_DEG_S = 45.0
DEFAULT_STEPPER_PULSE_US = 200.0

_GPIO_LOCK = Lock()
_GPIO_MODE_SET = False


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, value))


def _get_gpio_module():
    global _GPIO_MODE_SET

    try:
        import RPi.GPIO as GPIO  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("Hardware motor profile requires RPi.GPIO on Raspberry Pi") from exc

    with _GPIO_LOCK:
        if not _GPIO_MODE_SET:
            GPIO.setmode(GPIO.BCM)
            _GPIO_MODE_SET = True

    return GPIO


class ServoMotor:
    def __init__(self, config: MotorConfig) -> None:
        if config.kind is not MotorKind.SERVO:
            raise ValueError(f"{config.id} is not a servo motor config")

        driver_options = dict(config.driver_options)
        self.id = config.id
        self.kind = config.kind
        self.min_angle = float(config.min_angle)
        self.max_angle = float(config.max_angle)
        self.home_angle = float(config.home_angle)
        self.driver_type = config.driver_type
        self.current_angle = float(config.home_angle)
        self.target_angle = float(config.home_angle)
        self.homed = True
        self.ready = True
        self.moving = False
        self.last_command: str | None = None
        self.last_error: str | None = None
        self.last_pwm_duty_cycle: float | None = None

        self._lock = Lock()
        self._stopped = False
        self._physical_angle = float(config.home_angle)
        self._zero_offset_physical = float(config.zero_offset)
        self._signal_pin = driver_options.get("signal_pin")
        self._frequency_hz = float(driver_options.get("frequency_hz", DEFAULT_SERVO_FREQUENCY_HZ))
        self._min_pulse_us = float(driver_options.get("min_pulse_us", DEFAULT_SERVO_MIN_PULSE_US))
        self._max_pulse_us = float(driver_options.get("max_pulse_us", DEFAULT_SERVO_MAX_PULSE_US))
        self._hold_time_s = float(driver_options.get("hold_time_s", DEFAULT_SERVO_HOLD_TIME_S))
        self._pwm = None
        self._release_timer: Timer | None = None
        self._release_token = 0

        if self._uses_hardware():
            try:
                self._init_hardware()
            except RuntimeError as exc:
                self.ready = False
                self.last_error = str(exc)

    def snapshot(self) -> MotorState:
        with self._lock:
            return MotorState(
                id=self.id,
                kind=self.kind,
                current_angle=self.current_angle,
                target_angle=self.target_angle,
                hw_angle=None,
                moving=self.moving,
                homed=self.homed,
                ready=self.ready,
                last_command=self.last_command,
                last_error=self.last_error,
            )

    def home(self) -> None:
        self.move_to(self.home_angle, duration_ms=0)
        with self._lock:
            self.homed = True
            self.ready = True
            self.last_command = "HOME_SERVO"

    def move_to(self, target_angle: float, *, duration_ms: int) -> None:
        with self._lock:
            if self._stopped:
                return
            clamped_target = _clamp(float(target_angle), self.min_angle, self.max_angle)
            physical_angle = _clamp(
                clamped_target + self._zero_offset_physical,
                self.min_angle,
                self.max_angle,
            )
            self.current_angle = clamped_target
            self.target_angle = clamped_target
            self._physical_angle = physical_angle
            self.moving = False
            self.last_command = (
                f"UPDATE_SERVO angle={clamped_target:.3f} duration_ms={int(duration_ms)}"
            )
            self._apply_pwm_for_angle_locked(physical_angle)

    def stop_now(self) -> None:
        with self._lock:
            self._stopped = True
            self.moving = False
            self.last_command = "STOP_SERVO"
            self._disable_pwm_locked()

    def clear_stop(self) -> None:
        with self._lock:
            self._stopped = False

    def set_current_angle_zero(self) -> None:
        with self._lock:
            self._zero_offset_physical = self._physical_angle
            self.current_angle = 0.0
            self.target_angle = 0.0
            self.last_command = "ZERO_SERVO"

    def shutdown(self) -> None:
        with self._lock:
            self._stopped = True
            if self._release_timer is not None:
                self._release_timer.cancel()
                self._release_timer = None
            pwm = self._pwm
            self._pwm = None
            signal_pin = self._signal_pin

        if pwm is not None:
            pwm.ChangeDutyCycle(0.0)
            pwm.stop()

        if self._uses_hardware() and signal_pin is not None:
            gpio = _get_gpio_module()
            gpio.cleanup(signal_pin)

    def _uses_hardware(self) -> bool:
        return self.driver_type == "mg995_servo"

    def _init_hardware(self) -> None:
        if self._signal_pin is None:
            raise ValueError(f"{self.id} servo hardware driver needs signal_pin")

        gpio = _get_gpio_module()
        gpio.setup(self._signal_pin, gpio.OUT)
        pwm = gpio.PWM(self._signal_pin, self._frequency_hz)
        pwm.start(0.0)
        self._pwm = pwm

    def _disable_pwm_locked(self) -> None:
        self.last_pwm_duty_cycle = 0.0
        self._release_token += 1
        if self._release_timer is not None:
            self._release_timer.cancel()
            self._release_timer = None
        if self._pwm is not None:
            self._pwm.ChangeDutyCycle(0.0)

    def _apply_pwm_for_angle_locked(self, angle: float) -> None:
        duty_cycle = self._angle_to_duty_cycle(angle)
        self.last_pwm_duty_cycle = duty_cycle
        if not self._uses_hardware():
            return

        if self._pwm is None:
            self._init_hardware()

        assert self._pwm is not None
        self._pwm.ChangeDutyCycle(duty_cycle)

        self._release_token += 1
        release_token = self._release_token
        if self._release_timer is not None:
            self._release_timer.cancel()

        self._release_timer = Timer(self._hold_time_s, self._release_pwm_if_current, [release_token])
        self._release_timer.daemon = True
        self._release_timer.start()

    def _release_pwm_if_current(self, release_token: int) -> None:
        with self._lock:
            if release_token != self._release_token:
                return
            self._disable_pwm_locked()

    def _angle_to_duty_cycle(self, angle: float) -> float:
        span = self.max_angle - self.min_angle
        if span <= 0:
            return 0.0

        progress = (float(angle) - self.min_angle) / span
        pulse_us = self._min_pulse_us + progress * (self._max_pulse_us - self._min_pulse_us)
        period_us = 1_000_000.0 / self._frequency_hz
        return pulse_us / period_us * 100.0


class StepperMotor:
    def __init__(self, config: MotorConfig) -> None:
        if config.kind is not MotorKind.STEPPER:
            raise ValueError(f"{config.id} is not a stepper motor config")

        driver_options = dict(config.driver_options)
        self.id = config.id
        self.kind = config.kind
        self.min_angle = float(config.min_angle)
        self.max_angle = float(config.max_angle)
        self.home_angle = float(config.home_angle)
        self.driver_type = config.driver_type
        self.requires_homing = bool(config.requires_homing)
        self.max_velocity_deg_s = float(
            driver_options.get("max_velocity_deg_s", DEFAULT_STEPPER_MAX_VELOCITY_DEG_S)
        )
        self.steps_per_rev = int(config.steps_per_rev or 200)
        self.microsteps = int(config.microsteps or 1)
        self.gear_ratio = float(config.gear_ratio or 1.0)

        self.current_angle = float(config.home_angle)
        self.target_angle = float(config.home_angle)
        self.homed = not self.requires_homing
        self.ready = True
        self.moving = False
        self.last_command: str | None = None
        self.last_error: str | None = None

        self._lock = Lock()
        self._stop_event = Event()
        self._pause_event = Event()
        self._zero_offset_physical = float(config.zero_offset)
        self._step_pin = driver_options.get("step_pin")
        self._dir_pin = driver_options.get("dir_pin")
        self._enable_pin = driver_options.get("enable_pin")
        self._step_pulse_s = float(driver_options.get("step_pulse_us", DEFAULT_STEPPER_PULSE_US)) / 1_000_000.0

        home_steps = self._angle_to_steps(self.home_angle)
        self.current_steps = home_steps
        self.target_steps = home_steps

        if self._uses_hardware():
            try:
                self._init_hardware()
            except RuntimeError as exc:
                self.ready = False
                self.last_error = str(exc)

        self._thread = Thread(
            target=self._run_pulse_loop,
            name=f"stepper-pulse-{self.id}",
            daemon=True,
        )
        self._thread.start()

    def snapshot(self) -> MotorState:
        with self._lock:
            hw_angle = None
            if self._uses_hardware():
                hw_angle = self._steps_to_angle(self.current_steps) - self._zero_offset_physical
            return MotorState(
                id=self.id,
                kind=self.kind,
                current_angle=self.current_angle,
                target_angle=self.target_angle,
                hw_angle=hw_angle,
                moving=self.moving,
                homed=self.homed,
                ready=self.ready,
                last_command=self.last_command,
                last_error=self.last_error,
            )

    def home(self) -> None:
        with self._lock:
            home_steps = self._angle_to_steps(self.home_angle)
            self.current_steps = home_steps
            self.target_steps = home_steps
            self.current_angle = self.home_angle
            self.target_angle = self.home_angle
            self.homed = True
            self.ready = True
            self.moving = False
            self.last_command = "HOME_STEPPER"

    def move_to(self, target_angle: float, *, duration_ms: int) -> None:
        with self._lock:
            if self._stopped_or_unhomed():
                return

            clamped_target = _clamp(float(target_angle), self.min_angle, self.max_angle)
            physical_angle = _clamp(
                clamped_target + self._zero_offset_physical,
                self.min_angle,
                self.max_angle,
            )
            self.current_angle = clamped_target
            self.target_angle = clamped_target
            self.target_steps = self._angle_to_steps(physical_angle)
            self.moving = self.target_steps != self.current_steps
            self.last_command = (
                f"UPDATE_STEPPER angle={clamped_target:.3f} duration_ms={int(duration_ms)}"
            )

    def stop_now(self) -> None:
        self._pause_event.set()
        with self._lock:
            self.moving = False
            self.last_command = "STOP_STEPPER"

    def clear_stop(self) -> None:
        self._pause_event.clear()

    def set_current_angle_zero(self) -> None:
        with self._lock:
            self._zero_offset_physical = 0.0
            self.current_steps = 0
            self.target_steps = 0
            self.current_angle = 0.0
            self.target_angle = 0.0
            self.moving = False
            self.last_command = "ZERO_STEPPER"

    def shutdown(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=0.5)

        if self._uses_hardware():
            gpio = _get_gpio_module()
            if self._step_pin is not None:
                gpio.output(self._step_pin, gpio.LOW)
                gpio.cleanup(self._step_pin)
            if self._dir_pin is not None:
                gpio.cleanup(self._dir_pin)
            if self._enable_pin is not None:
                gpio.cleanup(self._enable_pin)

    def _uses_hardware(self) -> bool:
        return self.driver_type == "nema17_step_dir"

    def _stopped_or_unhomed(self) -> bool:
        return self._pause_event.is_set() or (self.requires_homing and not self.homed)

    def _init_hardware(self) -> None:
        if self._step_pin is None or self._dir_pin is None:
            raise ValueError(f"{self.id} stepper hardware driver needs step_pin and dir_pin")

        gpio = _get_gpio_module()
        gpio.setup(self._step_pin, gpio.OUT)
        gpio.setup(self._dir_pin, gpio.OUT)
        gpio.output(self._step_pin, gpio.LOW)
        gpio.output(self._dir_pin, gpio.LOW)

        if self._enable_pin is not None:
            gpio.setup(self._enable_pin, gpio.OUT)
            gpio.output(self._enable_pin, gpio.LOW)

    def _run_pulse_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._pause_event.is_set():
                sleep(0.01)
                continue

            should_wait = 0.0
            with self._lock:
                if self.requires_homing and not self.homed:
                    should_wait = 0.01
                else:
                    delta = self.target_steps - self.current_steps
                    if delta == 0:
                        self.moving = False
                        should_wait = 0.002
                    else:
                        direction = 1 if delta > 0 else -1
                        step_period_s = 1.0 / self._max_steps_per_s()
                        self.moving = True

            if should_wait > 0.0:
                sleep(should_wait)
                continue

            try:
                self._pulse_once(direction)
            except Exception as exc:  # pragma: no cover
                with self._lock:
                    self.last_error = str(exc)
                    self.moving = False
                self._pause_event.set()
                sleep(0.05)
                continue

            with self._lock:
                self.current_steps += direction
                self.moving = self.current_steps != self.target_steps

            sleep(max(0.0, step_period_s - (self._step_pulse_s * 2.0)))

    def _max_steps_per_s(self) -> float:
        steps_per_degree = self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0
        return max(1.0, abs(self.max_velocity_deg_s) * abs(steps_per_degree))

    def _angle_to_steps(self, angle: float) -> int:
        steps_per_degree = self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0
        return int(round(angle * steps_per_degree))

    def _steps_to_angle(self, steps: int) -> float:
        steps_per_degree = self.steps_per_rev * self.microsteps * self.gear_ratio / 360.0
        if steps_per_degree == 0:
            return 0.0
        return steps / steps_per_degree

    def _pulse_once(self, direction: int) -> None:
        if not self._uses_hardware():
            return

        gpio = _get_gpio_module()
        assert self._dir_pin is not None
        assert self._step_pin is not None

        gpio.output(self._dir_pin, gpio.HIGH if direction > 0 else gpio.LOW)
        gpio.output(self._step_pin, gpio.HIGH)
        sleep(self._step_pulse_s)
        gpio.output(self._step_pin, gpio.LOW)
        sleep(self._step_pulse_s)
