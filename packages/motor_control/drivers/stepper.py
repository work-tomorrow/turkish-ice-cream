"""Stepper hardware drivers."""

from __future__ import annotations

from time import sleep


class StepDirStepperDriver:
    """Drive a NEMA17-style stepper through a STEP/DIR driver board."""

    def __init__(
        self,
        *,
        step_pin: int | None,
        dir_pin: int | None,
        enable_pin: int | None = None,
        limit_pin: int | None = None,
        gpio_mode: str = "BCM",
        enable_active_low: bool = True,
        direction_high_for_positive: bool = True,
        limit_active_low: bool = True,
        step_pulse_us: float = 800.0,
        home_direction: int = -1,
        home_step_delay_s: float = 0.003,
        home_backoff_steps: int = 0,
        max_home_steps: int = 10_000,
        cleanup_on_stop: bool = False,
    ) -> None:
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.limit_pin = limit_pin
        self.gpio_mode = gpio_mode
        self.enable_active_low = enable_active_low
        self.direction_high_for_positive = direction_high_for_positive
        self.limit_active_low = limit_active_low
        self.step_pulse_us = step_pulse_us
        self.home_direction = 1 if home_direction >= 0 else -1
        self.home_step_delay_s = home_step_delay_s
        self.home_backoff_steps = home_backoff_steps
        self.max_home_steps = max_home_steps
        self.cleanup_on_stop = cleanup_on_stop
        self._gpio: object | None = None
        self.enabled = False
        self.position_steps = 0
        self.stopped = False

    def setup(self) -> None:
        if self.step_pin is None or self.dir_pin is None:
            raise ValueError("StepDirStepperDriver needs step_pin and dir_pin")
        if self.step_pulse_us <= 0:
            raise ValueError("step_pulse_us must be positive")
        if self.max_home_steps <= 0:
            raise ValueError("max_home_steps must be positive")

        gpio = self._import_gpio()
        mode = gpio.BCM if self.gpio_mode == "BCM" else gpio.BOARD
        gpio.setmode(mode)
        gpio.setup(self.step_pin, gpio.OUT)
        gpio.setup(self.dir_pin, gpio.OUT)
        self._gpio = gpio
        if self.enable_pin is not None:
            gpio.setup(self.enable_pin, gpio.OUT)
            self._write_enable(False)
        if self.limit_pin is not None:
            pull = gpio.PUD_UP if self.limit_active_low else gpio.PUD_DOWN
            gpio.setup(self.limit_pin, gpio.IN, pull_up_down=pull)
        self.stopped = False

    def home(self) -> None:
        if self.limit_pin is None:
            self.position_steps = 0
            return

        self._ensure_setup()
        self._write_enable(True)
        self._write_direction(self.home_direction)
        steps_taken = 0
        while not self._limit_triggered():
            if steps_taken >= self.max_home_steps:
                raise RuntimeError("Stepper homing failed before max_home_steps")
            self._pulse_once()
            steps_taken += 1
            sleep(self.home_step_delay_s)

        if self.home_backoff_steps:
            self._write_direction(-self.home_direction)
            for _ in range(self.home_backoff_steps):
                self._pulse_once()
                sleep(self.home_step_delay_s)
        self.position_steps = 0

    def move_steps(self, steps: int, duration_ms: int) -> None:
        if steps == 0:
            return

        self._ensure_setup()
        self._write_enable(True)
        direction = 1 if steps > 0 else -1
        count = abs(steps)
        self._write_direction(direction)
        step_delay_s = self._step_delay_s(count, duration_ms)
        for _ in range(count):
            if self.stopped:
                break
            self._pulse_once()
            self.position_steps += direction
            if step_delay_s > 0:
                sleep(step_delay_s)

    def stop(self) -> None:
        self.stopped = True
        self._write_enable(False)
        if self.cleanup_on_stop:
            self.cleanup()

    def cleanup(self) -> None:
        if self._gpio is None:
            return
        pins = [self.step_pin, self.dir_pin, self.enable_pin, self.limit_pin]
        for pin in pins:
            if pin is not None:
                self._gpio.cleanup(pin)
        self._gpio = None

    def read_limit(self) -> bool:
        if self.limit_pin is None:
            return False
        self._ensure_setup()
        return self._limit_triggered()

    def _ensure_setup(self) -> None:
        if self._gpio is None:
            raise RuntimeError("StepDirStepperDriver.setup() must be called first")

    def _write_enable(self, enabled: bool) -> None:
        self.enabled = enabled
        if self._gpio is None or self.enable_pin is None:
            return
        active = self._gpio.LOW if self.enable_active_low else self._gpio.HIGH
        inactive = self._gpio.HIGH if self.enable_active_low else self._gpio.LOW
        self._gpio.output(self.enable_pin, active if enabled else inactive)

    def _write_direction(self, direction: int) -> None:
        self._ensure_setup()
        assert self._gpio is not None
        positive_high = (
            self._gpio.HIGH if self.direction_high_for_positive else self._gpio.LOW
        )
        negative_high = (
            self._gpio.LOW if self.direction_high_for_positive else self._gpio.HIGH
        )
        output = positive_high if direction >= 0 else negative_high
        self._gpio.output(self.dir_pin, output)

    def _pulse_once(self) -> None:
        self._ensure_setup()
        assert self._gpio is not None
        half_period_s = self.step_pulse_us / 2_000_000.0
        self._gpio.output(self.step_pin, self._gpio.HIGH)
        sleep(half_period_s)
        self._gpio.output(self.step_pin, self._gpio.LOW)
        sleep(half_period_s)

    def _limit_triggered(self) -> bool:
        self._ensure_setup()
        assert self._gpio is not None
        value = self._gpio.input(self.limit_pin)
        if self.limit_active_low:
            return value == self._gpio.LOW
        return value == self._gpio.HIGH

    def _step_delay_s(self, count: int, duration_ms: int) -> float:
        if duration_ms <= 0 or count <= 0:
            return 0.0
        total_s = duration_ms / 1000.0
        pulse_s = self.step_pulse_us / 1_000_000.0
        return max(0.0, total_s / count - pulse_s)

    def _import_gpio(self) -> object:
        try:
            import RPi.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError(
                "StepDirStepperDriver requires RPi.GPIO on Raspberry Pi hardware"
            ) from exc
        return GPIO
