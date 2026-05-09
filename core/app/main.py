import asyncio
from contextlib import asynccontextmanager, suppress

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from core.app.config import settings
from core.app.robot_controller import RobotController
from core.app.models import Hand, RobotState
from core.app.router import api_router
from core.app.vision_state import vision_stale_monitor
from motor_control import MotorManager


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.running = False
    app.state.robot_state = RobotState.idle
    app.state.hand = Hand()
    app.state.vision_ready = False
    app.state.last_hand_result_at = None
    app.state.motor_manager = MotorManager()
    app.state.motor_manager_ready = False
    app.state.circle_demo_running = False
    app.state.circle_demo_task = None
    app.state.circle_demo_lock = asyncio.Lock()

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

        circle_task = getattr(app.state, "circle_demo_task", None)
        if circle_task is not None and not circle_task.done():
            app.state.circle_demo_running = False
            circle_task.cancel()
            with suppress(asyncio.CancelledError):
                await circle_task

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
