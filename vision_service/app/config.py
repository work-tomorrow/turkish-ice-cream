import os
from pathlib import Path

from dotenv import load_dotenv
from pydantic import BaseModel, ConfigDict


VISION_SERVICE_ENV_FILE = Path(__file__).resolve().parents[1] / ".env"
load_dotenv(VISION_SERVICE_ENV_FILE)


class Settings(BaseModel):
    model_config = ConfigDict(extra="ignore")

    project_name: str = "Turkish Ice Cream Vision Service"
    description: str = (
        "Desktop hand detection service with camera preview and outbound "
        "RPI result publishing."
    )
    version: str = "0.1.0"
    rpi_hand_ingest_ws_url: str = os.getenv(
        "RPI_HAND_INGEST_WS_URL",
        "ws://127.0.0.1:8000/ws/hand-ingest",
    )
    rpi_reconnect_seconds: float = float(os.getenv("RPI_RECONNECT_SECONDS", "1.0"))
    cors_allow_origin_regex: str = (
        r"^https?://("
        r"localhost|"
        r"127(?:\.\d{1,3}){3}|"
        r"10(?:\.\d{1,3}){3}|"
        r"192\.168(?:\.\d{1,3}){2}|"
        r"172\.(?:1[6-9]|2\d|3[0-1])(?:\.\d{1,3}){2}"
        r")(?::\d+)?$"
    )


settings = Settings()
