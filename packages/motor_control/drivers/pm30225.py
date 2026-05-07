"""PM30225 coreless motor serial driver."""


class PM30225SerialDriver:
    """TODO: Serial/UART implementation for the PM30225 motor controller."""

    def __init__(self, **options: object) -> None:
        self.options = options

    def setup(self) -> None:
        raise NotImplementedError("PM30225 serial setup is not implemented yet")

    def send_command(self, command: str) -> None:
        raise NotImplementedError("PM30225 command send is not implemented yet")

    def stop(self) -> None:
        raise NotImplementedError("PM30225 stop is not implemented yet")
