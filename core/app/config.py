from pydantic import BaseModel, ConfigDict


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
    vision_stale_timeout_seconds: float = 1.0
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
