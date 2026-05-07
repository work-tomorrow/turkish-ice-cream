"""Stepper hardware drivers."""


class StepDirStepperDriver:
    """TODO: Raspberry Pi STEP/DIR implementation."""

    def __init__(self, **options: object) -> None:
        self.options = options

    def setup(self) -> None:
        raise NotImplementedError("STEP/DIR setup is not implemented yet")

    def home(self) -> None:
        raise NotImplementedError("STEP/DIR homing is not implemented yet")

    def move_steps(self, steps: int, duration_ms: int) -> None:
        raise NotImplementedError("STEP/DIR movement is not implemented yet")

    def stop(self) -> None:
        raise NotImplementedError("STEP/DIR stop is not implemented yet")
