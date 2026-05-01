from pydantic import BaseModel, ConfigDict


class Settings(BaseModel):
    """Application configuration.

    Extend later with pydantic-settings for env parsing.
    """

    model_config = ConfigDict(extra="ignore")

    project_name: str = "Turkish Ice Cream Core API"
    description: str = "Backend API for the Turkish ice cream project."
    version: str = "0.1.0"
    api_v1_prefix: str = "/api/v1"


settings = Settings()
