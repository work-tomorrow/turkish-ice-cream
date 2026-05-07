"""TT motor hardware drivers."""


class TTDriver:
    """TODO: Raspberry Pi TT motor implementation."""

    def __init__(self, **options: object) -> None:
        self.options = options

    def setup(self) -> None:
        raise NotImplementedError("TT motor setup is not implemented yet")

    def move_to_angle_approx(self, angle: float, duration_ms: int) -> None:
        raise NotImplementedError("TT motor control is not implemented yet")

    def stop(self) -> None:
        raise NotImplementedError("TT motor stop is not implemented yet")
