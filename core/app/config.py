import os
from pathlib import Path

from dotenv import load_dotenv
from pydantic import BaseModel, ConfigDict, Field

CORE_ENV_FILE = Path(__file__).resolve().parents[1] / ".env"
load_dotenv(CORE_ENV_FILE)


def _env_float(name: str, default: float) -> float:
    return float(os.getenv(name, str(default)))


class Settings(BaseModel):
    """Application configuration.

    Extend later with pydantic-settings for env parsing.
    """

    model_config = ConfigDict(extra="ignore")

    project_name: str = "Turkish Ice Cream Core API"
    description: str = (
        "Backend API for the Turkish ice cream project, including session, "
        "vision, stop, and motor calibration controls."
    )
    version: str = "0.1.0"
    api_v1_prefix: str = "/api/v1"
    vision_stale_timeout_seconds: float = float(
        os.getenv("VISION_STALE_TIMEOUT_SECONDS", "1.0")
    )
    vanish_m1_deg: float = Field(
        default_factory=lambda: _env_float("VANISH_M1_DEG", 0.0)
    )
    vanish_m2_deg: float = Field(
        default_factory=lambda: _env_float("VANISH_M2_DEG", 90.0)
    )
    vanish_m3_deg: float = Field(
        default_factory=lambda: _env_float("VANISH_M3_DEG", 90.0)
    )
    vanish_m4_deg: float = Field(
        default_factory=lambda: _env_float("VANISH_M4_DEG", 90.0)
    )
    cors_allow_origin_regex: str = (
        r"^https?://("
        r"localhost|"
        r"127(?:\.\d{1,3}){3}|"
        r"10(?:\.\d{1,3}){3}|"
        r"192\.168(?:\.\d{1,3}){2}|"
        r"172\.(?:1[6-9]|2\d|3[0-1])(?:\.\d{1,3}){2}"
        r")(?::\d+)?$"
    )

    @property
    def vanish_motor_targets(self) -> dict[str, float]:
        return {
            "m1": self.vanish_m1_deg,
            "m2": self.vanish_m2_deg,
            "m3": self.vanish_m3_deg,
            "m4": self.vanish_m4_deg,
        }


settings = Settings()
