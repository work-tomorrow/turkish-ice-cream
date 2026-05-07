from contextlib import asynccontextmanager

from fastapi import FastAPI

from core.app.config import settings
from core.app.models import ActionName, Hand
from core.app.router import api_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.running = False
    app.state.emergency_stopped = False
    app.state.current_action = ActionName.home
    app.state.tease_count = 0
    app.state.max_tease = 10
    app.state.hand = Hand()
    yield


app = FastAPI(
    title=settings.project_name,
    description=settings.description,
    version=settings.version,
    validate_response=True,
    lifespan=lifespan,
)

app.include_router(api_router)
