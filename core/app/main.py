from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from core.app.config import settings
from core.app.models import Hand, RobotState
from core.app.router import api_router
from motor_control import MotorManager
from motor_control.config import build_m1_m4_hardware_motors


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.running = False
    app.state.emergency_stopped = False
    app.state.robot_state = RobotState.idle
    app.state.tease_count = 0
    app.state.max_tease = 10
    app.state.hand = Hand()
    app.state.motor_manager = MotorManager(
        motors=build_m1_m4_hardware_motors(
            stepper_options={
                "m1": {"step_pin": 17, "dir_pin": 27, "driver": "hardware"},
                "m2": {"step_pin": 23, "dir_pin": 24, "driver": "hardware"},
            },
            servo_options={
                "m3": {"signal_pin": 18},
                "m4": {"signal_pin": 19},
            },
        )
    )
    app.state.motor_manager_ready = False
    try:
        yield
    finally:
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
