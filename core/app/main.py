import asyncio
import logging
from contextlib import asynccontextmanager, suppress
from typing import Any

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from core.app.config import settings
from core.app.robot_controller import RobotController
from core.app.models import Hand, RobotState
from core.app.router import api_router
from core.app.vision_state import vision_stale_monitor
from motor_control import MotorManager

logger = logging.getLogger(__name__)
_CORE_SIGNAL_PIN = 26


class _CoreStartupSignalPin:
    """Drive a dedicated GPIO pin HIGH during core runtime."""

    def __init__(self, pin: int) -> None:
        self._pin = pin
        self._gpio: Any | None = None

    def set_high(self) -> None:
        try:
            import RPi.GPIO as GPIO  # type: ignore
        except ImportError:
            return
        except Exception as exc:
            logger.warning(
                "Failed to import RPi.GPIO for startup signal: %s",
                exc,
            )
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.OUT)
            GPIO.output(self._pin, GPIO.HIGH)
            self._gpio = GPIO
        except Exception as exc:
            logger.warning(
                "Failed to set startup signal GPIO pin %s HIGH: %s",
                self._pin,
                exc,
            )

    def set_low(self) -> None:
        if self._gpio is None:
            return

        try:
            self._gpio.output(self._pin, self._gpio.LOW)
        except Exception as exc:
            logger.warning(
                "Failed to set startup signal GPIO pin %s LOW: %s",
                self._pin,
                exc,
            )


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.core_signal_pin = _CoreStartupSignalPin(_CORE_SIGNAL_PIN)
    app.state.core_signal_pin.set_high()

    app.state.running = False
    app.state.robot_state = RobotState.idle
    app.state.hand = Hand()
    app.state.vision_ready = False
    app.state.last_hand_result_at = None
    app.state.motor_manager = MotorManager()
    app.state.motor_manager_ready = False
    app.state.debug_session_active = False
    app.state.debug_move_task = None
    app.state.debug_move_lock = asyncio.Lock()
    app.state.debug_move_target_y_cm = None
    app.state.debug_move_target_z_cm = None
    app.state.debug_move_duration_s = None

    app.state.vision_stale_task = asyncio.create_task(
        vision_stale_monitor(
            app,
            timeout_seconds=settings.vision_stale_timeout_seconds,
        )
    )

    app.state.controller = RobotController(app)
    app.state.controller_task = asyncio.create_task(app.state.controller.run())

    try:
        yield
    finally:
        app.state.vision_stale_task.cancel()
        with suppress(asyncio.CancelledError):
            await app.state.vision_stale_task

        app.state.controller_task.cancel()
        with suppress(asyncio.CancelledError):
            await app.state.controller_task

        debug_move_task = getattr(app.state, "debug_move_task", None)
        if debug_move_task is not None and not debug_move_task.done():
            debug_move_task.cancel()
            with suppress(asyncio.CancelledError):
                await debug_move_task

        app.state.motor_manager.cleanup()
        app.state.core_signal_pin.set_low()


app = FastAPI(
    title=settings.project_name,
    description=settings.description,
    version=settings.version,
    validate_response=True,
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=settings.cors_allow_origin_regex,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router)
