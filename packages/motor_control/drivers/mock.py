"""Mock drivers for local development and tests."""

from __future__ import annotations


class MockStepperDriver:
    """In-memory stand-in for a STEP/DIR stepper driver."""

    def __init__(self) -> None:
        self.enabled = False
        self.step_count = 0
        self.stopped = False

    def setup(self) -> None:
        self.stopped = False

    def home(self) -> None:
        self.step_count = 0

    def move_steps(self, steps: int, duration_ms: int) -> None:
        self.enabled = True
        self.step_count += steps

    def stop(self) -> None:
        self.stopped = True
        self.enabled = False


class MockTTDriver:
    """In-memory stand-in for a TT motor driver."""

    def __init__(self) -> None:
        self.stopped = False

    def setup(self) -> None:
        self.stopped = False

    def move_to_angle_approx(self, angle: float, duration_ms: int) -> None:
        # TODO: replace with PWM/time calibration or encoder feedback.
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True


class MockServoDriver:
    """In-memory stand-in for an MG995 servo PWM driver."""

    def __init__(self) -> None:
        self.angle: float | None = None
        self.stopped = False

    def setup(self) -> None:
        self.stopped = False

    def set_angle(self, angle: float) -> None:
        self.angle = angle
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True


class MockPM30225Driver:
    """In-memory stand-in for the PM30225 coreless motor controller."""

    def __init__(self) -> None:
        self.commands: list[str] = []
        self.stopped = False

    def setup(self) -> None:
        self.stopped = False

    def send_command(self, command: str) -> None:
        self.commands.append(command)

    def stop(self) -> None:
        self.stopped = True
        self.send_command("STOP")
