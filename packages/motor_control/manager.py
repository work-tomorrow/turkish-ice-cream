"""Public entry point for mixed motor control."""

from __future__ import annotations

from collections.abc import Iterable
from threading import Condition, Event, Lock, RLock, Thread
from time import monotonic

from motor_control.config import DEFAULT_MOTOR_CONFIGS
from motor_control.controllers import (
    MG995ServoController,
    MotorController,
    PM30225Controller,
    StepperController,
    TTController,
)
from motor_control.drivers import (
    MG995ServoDriver,
    MockPM30225Driver,
    MockServoDriver,
    MockStepperDriver,
    MockTTDriver,
    PM30225SerialDriver,
    StepDirStepperDriver,
    TTDriver,
)
from motor_control.models import MotorConfig, MotorKind, MotorManagerState


class MotorManager:
    """Single public API for controlling all configured motors."""

    def __init__(
        self,
        configs: Iterable[MotorConfig] | None = None,
        control_hz: float = 50.0,
    ) -> None:
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")
        self.configs = list(configs or DEFAULT_MOTOR_CONFIGS)
        self.control_hz = control_hz
        self._control_interval_s = 1.0 / control_hz
        self._lock = RLock()
        self._command_lock = Lock()
        self._command_condition = Condition(self._command_lock)
        self._latest_command: tuple[dict[str, float], int, int] | None = None
        self._processed_command_id = 0
        self._running = False
        self._stop_event = Event()
        self._worker: Thread | None = None
        self._controllers = self._build_controllers(self.configs)

    def start(self) -> None:
        """Set up all drivers and start the background worker."""

        with self._lock:
            if self._running:
                return
            for controller in self._controllers.values():
                controller.setup()
            self._stop_event.clear()
            self._worker = Thread(
                target=self._run_worker,
                name="motor-control-worker",
                daemon=True,
            )
            self._running = True
            self._worker.start()

    def stop(self) -> None:
        """Stop the background worker and all motors."""

        worker: Thread | None

        with self._lock:
            if not self._running:
                worker = self._worker
            else:
                self._running = False
                worker = self._worker
            self._stop_event.set()

        with self._command_condition:
            self._command_condition.notify_all()

        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)

        for controller in self._controllers.values():
            controller.stop()

        with self._lock:
            self._worker = None

    def home_all(self) -> None:
        """Home every motor that requires a home position."""

        with self._lock:
            for controller in self._controllers.values():
                controller.home()

    def set_target(self, targets: dict[str, float], duration_ms: int) -> None:
        """Store the latest target for the background worker.

        This method does not execute motor movement directly. It validates the
        request, overwrites any previous pending target, notifies the worker,
        and returns immediately.
        """

        self._validate_targets(targets, duration_ms)
        with self._command_condition:
            command_id = self._processed_command_id + 1
            if self._latest_command is not None:
                command_id = max(command_id, self._latest_command[2] + 1)
            self._latest_command = (dict(targets), duration_ms, command_id)
            self._command_condition.notify()

    def get_state(self) -> MotorManagerState:
        """Return a snapshot of all motor states."""

        with self._lock:
            running = self._running
        return MotorManagerState(
            running=running,
            motors={
                motor_id: controller.get_state()
                for motor_id, controller in self._controllers.items()
            },
        )

    def set_current_angle_zero(self, motor_id: str | None = None) -> None:
        """Redefine one or all controllers' current internal angle as zero."""

        if motor_id is not None and motor_id not in self._controllers:
            raise ValueError(f"Unknown motor id: {motor_id}")

        controllers = (
            [self._controllers[motor_id]]
            if motor_id is not None
            else list(self._controllers.values())
        )

        with self._command_condition:
            self._latest_command = None

        for controller in controllers:
            controller.set_current_angle_zero()

    def emergency_stop(self) -> None:
        """Immediately stop all motors."""

        self.stop()

    def cleanup(self) -> None:
        """Release all driver resources."""

        self.stop()

    @property
    def motor_ids(self) -> tuple[str, ...]:
        """Configured motor ids."""

        return tuple(self._controllers)

    def _validate_targets(self, targets: dict[str, float], duration_ms: int) -> None:
        if duration_ms <= 0:
            raise ValueError("duration_ms must be positive")

        expected = set(self._controllers)
        actual = set(targets)
        missing = expected - actual
        extra = actual - expected
        if missing or extra:
            if extra:
                unknown = ", ".join(sorted(extra))
                raise ValueError(f"Unknown motor id(s): {unknown}")
            missing_ids = ", ".join(sorted(missing))
            raise ValueError(f"Missing target(s) for motor id(s): {missing_ids}")

        config_by_id = {config.id: config for config in self.configs}
        for motor_id, angle in targets.items():
            config = config_by_id[motor_id]
            if angle < config.min_angle or angle > config.max_angle:
                raise ValueError(
                    f"Motor {motor_id!r} angle {angle} outside "
                    f"[{config.min_angle}, {config.max_angle}]"
                )

        for motor_id in targets:
            state = self._controllers[motor_id].get_state()
            if not state.ready:
                raise RuntimeError(f"Motor {motor_id!r} is not ready")

    def _run_worker(self) -> None:
        last_tick = monotonic()
        controllers = self._controllers
        while not self._stop_event.is_set():
            command = self._wait_for_command_or_tick()
            if command is None:
                continue

            now = monotonic()
            dt_ms = (now - last_tick) * 1000.0
            last_tick = now
            targets, duration_ms, command_id = command
            for motor_id, angle in targets.items():
                if self._stop_event.is_set():
                    return
                controllers[motor_id].update(
                    target_angle=angle,
                    duration_ms=duration_ms,
                    command_id=command_id,
                    dt_ms=dt_ms,
                )

            with self._command_condition:
                self._processed_command_id = max(
                    self._processed_command_id,
                    command_id,
                )

    def _wait_for_command_or_tick(self) -> tuple[dict[str, float], int, int] | None:
        with self._command_condition:
            if self._latest_command is None and not self._stop_event.is_set():
                self._command_condition.wait(timeout=self._control_interval_s)
            elif not self._stop_event.is_set():
                self._command_condition.wait(timeout=self._control_interval_s)

            if self._stop_event.is_set() or self._latest_command is None:
                return None
            return self._latest_command

    def _build_controllers(
        self,
        configs: list[MotorConfig],
    ) -> dict[str, MotorController]:
        if not configs:
            raise ValueError("MotorManager expects at least one motor config")

        controllers: dict[str, MotorController] = {}
        for config in configs:
            if config.id in controllers:
                raise ValueError(f"Duplicate motor id: {config.id}")
            controllers[config.id] = self._build_controller(config)
        return controllers

    def _build_controller(self, config: MotorConfig) -> MotorController:
        if config.kind is MotorKind.STEPPER:
            if config.driver_type == "mock_stepper":
                return StepperController(config, MockStepperDriver())
            if config.driver_type in {"step_dir", "nema17_step_dir"}:
                return StepperController(
                    config,
                    StepDirStepperDriver(**config.driver_options),
                )
        elif config.kind is MotorKind.SERVO:
            if config.driver_type == "mock_servo":
                return MG995ServoController(config, MockServoDriver())
            if config.driver_type in {"mg995", "mg995_servo"}:
                return MG995ServoController(
                    config,
                    MG995ServoDriver(**config.driver_options),
                )
        elif config.kind is MotorKind.TT:
            if config.driver_type == "mock_tt":
                return TTController(config, MockTTDriver())
            if config.driver_type == "tt_motor":
                return TTController(
                    config,
                    TTDriver(**config.driver_options),
                )
        elif config.kind is MotorKind.PM30225_SERIAL:
            if config.driver_type == "mock_pm30225":
                return PM30225Controller(config, MockPM30225Driver())
            if config.driver_type == "pm30225_serial":
                return PM30225Controller(
                    config,
                    PM30225SerialDriver(**config.driver_options),
                )

        raise ValueError(
            f"Unsupported motor kind/driver: {config.kind}/{config.driver_type}"
        )
