# AGENTS.md - Project Agent Guide

This file captures the current working memory for the whole
`turkish-ice-cream` checkout. Treat package-level `AGENTS.md` files as more
specific when you work inside those packages.

## Project Shape

This repo is a demo stack for the Turkish ice cream robot:

- `core/`: Raspberry Pi-side FastAPI service. It owns robot session state,
  motor manager lifetime, controller loop, and the public API used by the demo
  web.
- `vision_service/`: desktop-side FastAPI service. It owns camera capture,
  MediaPipe hand detection, annotated preview frames, and pushes hand results to
  the RPI core service.
- `packages/motor_control/`: reusable motor package. It hides stepper/servo
  hardware details behind `MotorManager`.
- `packages/hand_tracking/`: reusable camera and hand-detection package. It
  hides Picamera2 / MediaPipe setup behind simple APIs.
- `web/`: Vite / Vue / Element Plus demo UI.

Use the real files as source of truth when memory and code disagree. Hardware
maps and endpoint shapes have changed during development, so verify against
`core/app/router.py`, `core/app/models.py`, and `packages/motor_control/config.py`
before editing.

## Working Style

- Keep changes small and tied to the user's request.
- Prefer simple, navigable code over deep abstraction.
- If the user asks for a direct source transformation, edit the files directly.
- Do not rewrite unrelated dirty files. This repo often has local work in
  progress.
- For FastAPI responses, prefer explicit nested Pydantic model instances over
  dict literals when the field is model-typed.
- This is a `packages/` layout project. If imports fail, check the active Poetry
  environment and run `poetry install` before redesigning package code.
- Avoid committing generated caches such as `__pycache__` and `.DS_Store`.

## Backend Contract

The current core API style is slim `main.py`, route-heavy `router.py`, and
shared demo runtime state in `app.state`.

Important current route concepts:

- Keep `/ping`.
- `GET /status` returns nested `system`, `robot`, and `vision` objects.
- Keep motor status and calibration under `/motors`.
- Do not mix calibration data, demo counters, or old tease fields into
  `/status`.
- `DELETE /session` is the stop command for the current interaction and should
  stop motors immediately.
- Camera preview routes live in `vision_service`, not in `core`.

The old tease flow was intentionally removed. Do not reintroduce `max_tease`,
`tease_count`, or tease UI fields unless the user explicitly asks for that
behavior again.

## Motor Control

`MotorManager` is the main entry point for upper layers. The core service should
not know low-level motor driver details.

Before changing motor behavior, read:

- `packages/motor_control/AGENTS.md`
- `packages/motor_control/manager.py`
- `packages/motor_control/config.py`
- `packages/motor_control/motors.py`
- `packages/motor_control/mock_motors.py`

Current code source of truth:

- `TURKISH_ICE_CREAM_MOTOR_PROFILE=mock` is the safe local default.
- `TURKISH_ICE_CREAM_MOTOR_PROFILE=hardware` enables hardware motors.
- At the time this guide was written, `config.py` builds `m1` as a stepper and
  `m2`, `m3`, `m4` as servos in hardware mode.
- Calibration jog defaults to 3 degrees in the core route layer.
- Zero calibration means "treat the current physical pose as logical zero".

Keep `motor_control` hardware-agnostic at the manager boundary. Hardware imports
such as GPIO should stay in low-level motor implementation files.

## Hand Tracking And Vision

`hand_tracking` owns camera, frame, and detector details. Upper layers should
use `HandDetector`, `create_default_camera`, and JPEG / MJPEG helpers instead of
configuring MediaPipe or Picamera2 directly.

`vision_service` owns the desktop camera loop:

- It keeps the camera preview local.
- It exposes `/hand`, `/vision/frame`, and `/vision/stream`.
- It pushes hand detection updates to the RPI core ingest WebSocket configured
  by `RPI_HAND_INGEST_WS_URL`.

Do not move camera preview endpoints back into `core` unless the architecture is
explicitly changed.

## Frontend

The web app is a simple demo surface, not a full product shell. Use Vue,
Element Plus, and the existing component structure.

When editing the UI:

- Mirror the current API instead of inventing a broader app flow.
- Keep controls basic, clear, and demo-friendly.
- Keep motor calibration under the `/motors` API shape.
- Keep vision preview pointed at `VITE_VISION_API_BASE_URL`.
- Do not add tease controls unless requested.

When running npm commands in `web/`, use the user's Node environment:

```sh
source ~/.nvm/nvm.sh
npm run build
```

## Environment And Commands

Python dependencies are managed with Poetry:

```sh
poetry install
poetry run python -m core
poetry run python -m vision_service
```

Useful verification commands:

```sh
poetry run python -m compileall core packages vision_service
poetry run python -c "import motor_control; import hand_tracking"
```

For web verification:

```sh
cd web
source ~/.nvm/nvm.sh
npm run build
```

Local `uvicorn` bind attempts can fail in the sandbox. When that happens, use
direct function-level checks, OpenAPI generation, compile checks, or targeted
tests instead of assuming the app is broken.

## Memory Notes

Durable user preferences for this repo:

- Prefer entity/resource endpoint names such as `/motors`.
- Keep `/status` lifecycle-oriented: `system`, `robot`, `vision`.
- Keep motor configuration explanations focused on the few fields an operator
  actually edits.
- In `motor_control`, favor discoverability and a flatter layout.
- Do not add queues, movement planners, or internal background loops unless the
  user explicitly asks for that level of machinery.
- When hardware identity is known, use it. When uncertain, verify the current
  `config.py`.
