"""MG995 servo hardware driver."""

from __future__ import annotations


class MG995ServoDriver:
    """Drive an MG995 servo using a Raspberry Pi PWM GPIO pin."""

    def __init__(
        self,
        signal_pin: int | None,
        *,
        frequency_hz: float = 50.0,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        min_pulse_us: float = 500.0,
        max_pulse_us: float = 2500.0,
        gpio_mode: str = "BCM",
        cleanup_on_stop: bool = False,
        initial_angle: float | None = None,
    ) -> None:
        self.signal_pin = signal_pin
        self.frequency_hz = frequency_hz
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.gpio_mode = gpio_mode
        self.cleanup_on_stop = cleanup_on_stop
        self.initial_angle = initial_angle
        self._gpio: object | None = None
        self._pwm: object | None = None
        self.current_angle: float | None = None

    def setup(self) -> None:
        if self.signal_pin is None:
            raise ValueError("MG995ServoDriver needs signal_pin")
        if self.frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        if self.min_angle >= self.max_angle:
            raise ValueError("min_angle must be less than max_angle")
        if self.min_pulse_us >= self.max_pulse_us:
            raise ValueError("min_pulse_us must be less than max_pulse_us")

        gpio = self._import_gpio()
        mode = gpio.BCM if self.gpio_mode == "BCM" else gpio.BOARD
        gpio.setmode(mode)
        gpio.setup(self.signal_pin, gpio.OUT)
        pwm = gpio.PWM(self.signal_pin, self.frequency_hz)
        pwm.start(0.0)
        self._gpio = gpio
        self._pwm = pwm
        if self.initial_angle is not None:
            self.set_angle(self.initial_angle)

    def set_angle(self, angle: float) -> None:
        if self._pwm is None:
            raise RuntimeError("MG995ServoDriver.setup() must be called first")
        clamped = min(self.max_angle, max(self.min_angle, angle))
        self._pwm.ChangeDutyCycle(self._angle_to_duty_cycle(clamped))
        self.current_angle = clamped

    def stop(self) -> None:
        if self._pwm is not None:
            self._pwm.ChangeDutyCycle(0.0)
        if self.cleanup_on_stop:
            self.cleanup()

    def cleanup(self) -> None:
        if self._pwm is not None:
            self._pwm.stop()
            self._pwm = None
        if self._gpio is not None and self.signal_pin is not None:
            self._gpio.cleanup(self.signal_pin)
        self._gpio = None

    def _angle_to_duty_cycle(self, angle: float) -> float:
        span = self.max_angle - self.min_angle
        progress = (angle - self.min_angle) / span
        pulse_us = self.min_pulse_us + progress * (
            self.max_pulse_us - self.min_pulse_us
        )
        period_us = 1_000_000.0 / self.frequency_hz
        return pulse_us / period_us * 100.0

    def _import_gpio(self) -> object:
        try:
            import RPi.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError(
                "MG995ServoDriver requires RPi.GPIO on Raspberry Pi hardware"
            ) from exc
        return GPIO
