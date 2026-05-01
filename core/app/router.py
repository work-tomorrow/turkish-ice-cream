from fastapi import APIRouter

from core.app.config import settings

api_router = APIRouter()


@api_router.get("/")
async def root() -> dict[str, str]:
    return {"message": "Turkish Ice Cream Core API"}


@api_router.get("/health")
async def health() -> dict[str, str]:
    return {"status": "ok"}


v1_router = APIRouter(prefix=settings.api_v1_prefix, tags=["v1"])


@v1_router.get("/ping")
async def ping() -> dict[str, bool]:
    return {"pong": True}


api_router.include_router(v1_router)
