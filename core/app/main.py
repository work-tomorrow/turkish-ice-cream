import asyncio
from contextlib import asynccontextmanager, suppress

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from core.app.config import settings
from core.app.models import Hand, RobotState
from core.app.router import api_router
from core.app.vision_state import vision_stale_monitor
from motor_control import MotorManager
from motor_control.mock_motors import build_mock_motors


def _create_motor_manager() -> MotorManager:
    try:
        return MotorManager()
    except RuntimeError:
        return MotorManager(motors=build_mock_motors())


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.running = False
    app.state.robot_state = RobotState.idle
    app.state.tease_count = 0
    app.state.max_tease = 10
    app.state.hand = Hand()
    app.state.vision_ready = False
    app.state.last_hand_result_at = None
    app.state.motor_manager = _create_motor_manager()
    app.state.motor_manager_ready = False
    app.state.vision_stale_task = asyncio.create_task(
        vision_stale_monitor(
            app,
            timeout_seconds=settings.vision_stale_timeout_seconds,
        )
    )
    try:
        yield
    finally:
        app.state.vision_stale_task.cancel()
        with suppress(asyncio.CancelledError):
            await app.state.vision_stale_task
        app.state.motor_manager.cleanup()


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
