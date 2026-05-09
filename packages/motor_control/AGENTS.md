# AGENTS.md - `motor_control` Agent Guide

This file defines how future agents should work inside
`packages/motor_control/`. Treat it as the local operating manual for this
package.

## Mission

`motor_control` owns only motor-control concerns. It hides the mixed hardware
details behind one upper-layer API:

```python
from motor_control import MotorManager

manager = MotorManager()
manager.home_all()

manager.set_target_angles({
    "m1": 0,
    "m2": 0,
    "m3": 90,
    "m4": 90,
})
```

The current robot has 4 motors:

- 1 stepper motor: `m1`
- 3 servo motors: `m2`, `m3`, `m4`

Upper layers should not need to know which motor is stepper or servo controlled.

## Current Design

`MotorManager` is a simple synchronous safety/control layer.

The upper layer, such as the Core API controller loop, is responsible for calling
`set_target_angles()` periodically, usually around every 100 ms.

```text
Core API controller loop
  -> compute target angles
  -> MotorManager.set_target_angles(targets)
      -> validate ids
      -> clamp angles
      -> apply speed limit
      -> send safe target to each motor
```

There is no internal worker thread, command queue, interpolation scheduler, or
motion planner inside `motor_control`.

## Public API

The main public entry point is:

```python
MotorManager
```

Supported manager methods:

```python
manager.set_target_angles(targets: dict[str, float]) -> None
manager.set_target(targets: dict[str, float], duration_ms: int = 0) -> None
manager.home_all() -> None
manager.stop() -> None
manager.get_state() -> dict
manager.set_current_angle_zero(motor_id: str | None = None) -> None
manager.cleanup() -> None
```

`set_target()` exists only as a compatibility wrapper around
`set_target_angles()`. The `duration_ms` argument is ignored.

## Control Contract

Upper layers must provide a complete target dictionary every control tick.

Example:

```python
{
    "m1": 0.0,
    "m2": 90.0,
    "m3": 90.0,
    "m4": 90.0,
}
```

Rules:

- `targets` must contain exactly all configured motor ids.
- Missing or extra motor ids must raise `ValueError`.
- `set_target_angles()` must clamp each logical angle into the motor's valid
  range.
- `set_target_angles()` must apply per-motor speed limiting.
- `MotorManager` is the only place that converts logical angles to raw motor
  angles. Low-level motor `move_to()` receives raw angles.
- The upper layer owns timing. Do not add a background worker inside
  `MotorManager`.
- Do not add a movement queue unless the product requirement changes.

## Stop / Cleanup Semantics

Use these meanings consistently:

```python
manager.stop()
```

Stops all motors and freezes `current_targets` at the current pose.

It must **not** release GPIO resources. After `stop()`, the manager should still
be usable, and future `set_target_angles()` calls should work.

```python
manager.cleanup()
```

Shuts down motors and releases GPIO resources.

This should only be used when the process or FastAPI lifespan is ending.

```python
manager.emergency_stop()
```

If kept for compatibility, it should be an alias for `stop()`.

The project currently does not use a separate emergency-stop lockout model.
Stopping means "stop motion now and allow future restart."

## State Semantics

`manager.get_state()` should return a simple dictionary shaped like:

```python
{
    "dt_s": 0.1,
    "current_targets": {
        "m1": 0.0,
        "m2": 90.0,
        "m3": 90.0,
        "m4": 90.0,
    },
    "motors": {
        "m1": {
            "id": "m1",
            "current_angle": 0.0,
            "target_angle": 0.0,
            "min_angle": -180.0,
            "max_angle": 180.0,
            "ready": True,
            "last_error": None,
        },
        ...
    },
}
```

Notes:

- `current_angle` and `target_angle` exposed through `get_state()` are logical
  angles after applying zero offset.
- `min_angle` and `max_angle` should also be logical limits after applying zero
  offset.
- `current_targets` is manager-side target memory after clamp and speed limiting.
- `set_current_angle_zero()` treats the current physical pose as logical zero.
- Low-level motors keep raw `current_angle`, `target_angle`, `min_angle`, and
  `max_angle`; do not reset or shift those values for calibration.

## Hard Rules

- Keep `MotorManager` as the only main public entry point.
- Do not put FastAPI, hand detection, robot behavior, session state, or web code
  in this package.
- Do not make upper layers choose driver types, pins, protocols, or motor
  hardware details.
- Keep hardware-specific imports such as `RPi.GPIO` inside motor implementation
  files, not in `manager.py`.
- `manager.py` should stay hardware-agnostic.
- `set_target_angles()` must require exactly all configured motor ids.
- Do not silently accept missing target motors.
- Do not add command queues.
- Do not add an internal control-loop thread.
- Do not reintroduce interpolation, `command_id`, `MotionPlanner`, `Executor`,
  or old async movement concepts unless explicitly requested.
- Do not make normal `stop()` call `cleanup()`.
- Keep mock motors runnable on a normal development machine without Raspberry Pi
  hardware.
- Do not pretend incomplete hardware integrations are production-ready. Leave
  explicit TODOs or clear errors when hardware behavior is not implemented.

## Read First

Before changing behavior, inspect these files:

- `manager.py`: public API, target validation, angle clamp, speed limiting, stop,
  state reporting.
- `config.py`: default motor construction, ids, pins, angle limits, hardware/mock
  selection.
- `models.py`: shared data structures and motor kinds.
- `motors.py`: Raspberry Pi GPIO motor implementations.
- `mock_motors.py`: mock motors for local development.
- tests under `tests/`: package behavior contract.

## Current Architecture

Keep this dependency direction:

```text
upper layer
  -> MotorManager
      -> ServoMotor / StepperMotor / MockMotor
```

Responsibilities:

- `MotorManager`
  - Owns configured motors.
  - Validates complete target payloads.
  - Applies angle clamp.
  - Applies speed limit.
  - Calls each motor's `move_to()`.
  - Maintains `current_targets`.
  - Provides `home_all()`, `stop()`, `get_state()`, `set_current_angle_zero()`,
    and `cleanup()`.

- `motors.py`
  - Owns low-level hardware behavior.
  - Keeps Raspberry Pi GPIO code inside motor classes.
  - Provides `ServoMotor` and `StepperMotor`.

- `mock_motors.py`
  - Provides local mock motors.
  - Must not require Raspberry Pi hardware.

- `config.py`
  - Builds default motor objects.
  - Owns motor ids, limits, pins, and profile selection.

## Existing Implementation Notes

- `MotorManager()` should create the default 4-motor setup.
- Default motors may be mock or hardware depending on project configuration.
- `set_target()` is a compatibility wrapper and should call
  `set_target_angles()`.
- `duration_ms` is currently ignored.
- `dt_s` is used for speed limiting.
- Default `dt_s` is `0.1`, matching the expected upper-layer control tick.
- `max_velocity_deg_s` is per-motor and can be tuned later.
- `home_all()` moves all motors to their configured home angle.
- `stop()` should stop motion and freeze targets, not release GPIO.
- `cleanup()` releases hardware resources and should usually be called only when
  the app shuts down.

## How To Add Hardware Support

- Implement hardware behavior inside motor classes, not in `manager.py`.

- Keep Raspberry Pi library imports inside `motors.py`.

- For servos:
  - Configure GPIO PWM.
  - Convert angle to duty cycle.
  - Provide `move_to()`, `stop_now()`, `snapshot()`, and `cleanup()`.

- For steppers:
  - Configure STEP/DIR/optional ENABLE pins.
  - Convert angle to steps.
  - Provide `move_to()`, `stop_now()`, `snapshot()`, and `cleanup()`.
  - `move_to()` must only update the latest target and return quickly. A
    background worker should generate STEP pulses so long physical moves do not
    block Core API routes such as stop or status.

## Testing Requirements

For package-level behavior, use mock-only unit tests.

Add or update tests when changing:

- motor ids or default config shape
- target validation behavior
- angle clamp behavior
- speed limiting behavior
- stop behavior
- cleanup behavior
- zeroing behavior
- `get_state()` output shape
- servo duty-cycle conversion
- stepper angle-to-step conversion

Hardware tests should be opt-in and clearly separated from normal unit tests.
Do not make the default test command require Raspberry Pi hardware.

## Editing Guidelines

- Prefer small, local changes over broad refactors.
- Keep public API changes rare and deliberate.
- If motor ids change, update tests and upper-layer callers in the same change.
- Keep `manager.py` simple and hardware-agnostic.
- Keep comments short and useful.
- Use TODO comments only where a real hardware decision or implementation is
  still missing.
- Remove generated `__pycache__` directories before finishing.

## Common Pitfalls

- Do not let `manager.py` import Raspberry Pi GPIO or serial libraries.
- Do not silently accept missing target motors.
- Do not queue target commands.
- Do not reintroduce the old background worker architecture.
- Do not make `stop()` call `cleanup()`.
- Do not call `cleanup()` during normal session stop.
- Do not assume servo release means physical braking; for hobby servos, stopping
  PWM may release holding torque depending on the servo.
- Do not assume software stepping is precise enough for high-speed coordinated
  motion. If timing becomes critical, move stepping to a better backend.
