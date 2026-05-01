from fastapi import FastAPI

from core.app.config import settings
from core.app.router import api_router

app = FastAPI(
    title=settings.project_name,
    description=settings.description,
    version=settings.version,
)

app.include_router(api_router)
