import os
from pathlib import Path

from dotenv import load_dotenv
import uvicorn


if __name__ == "__main__":
    load_dotenv(Path(__file__).resolve().parent / ".env")
    uvicorn.run(
        "vision_service.app.main:app",
        host=os.getenv("VISION_SERVICE_HOST", "127.0.0.1"),
        port=int(os.getenv("VISION_SERVICE_PORT", "8010")),
        reload=True,
    )
