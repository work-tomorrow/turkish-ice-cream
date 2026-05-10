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
    ik_j1_min_deg: float = Field(
        default_factory=lambda: _env_float("IK_J1_MIN_DEG", -45.0)
    )
    ik_j1_max_deg: float = Field(
        default_factory=lambda: _env_float("IK_J1_MAX_DEG", 45.0)
    )
    ik_j2_min_deg: float = Field(
        default_factory=lambda: _env_float("IK_J2_MIN_DEG", -30.0)
    )
    ik_j2_max_deg: float = Field(
        default_factory=lambda: _env_float("IK_J2_MAX_DEG", 90.0)
    )
    ik_j3_min_deg: float = Field(
        default_factory=lambda: _env_float("IK_J3_MIN_DEG", -60.0)
    )
    ik_j3_max_deg: float = Field(
        default_factory=lambda: _env_float("IK_J3_MAX_DEG", 60.0)
    )
    ik_j4_min_deg: float = Field(
        default_factory=lambda: _env_float("IK_J4_MIN_DEG", -60.0)
    )
    ik_j4_max_deg: float = Field(
        default_factory=lambda: _env_float("IK_J4_MAX_DEG", 60.0)
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

    @property
    def ik_joint_limits_deg(self) -> dict[str, tuple[float, float]]:
        return {
            "J1": (self.ik_j1_min_deg, self.ik_j1_max_deg),
            "J2": (self.ik_j2_min_deg, self.ik_j2_max_deg),
            "J3": (self.ik_j3_min_deg, self.ik_j3_max_deg),
            "J4": (self.ik_j4_min_deg, self.ik_j4_max_deg),
        }


settings = Settings()
