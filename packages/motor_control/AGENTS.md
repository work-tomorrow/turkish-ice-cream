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
manager.start()
manager.home_all()
manager.set_target(targets, duration_ms)
```

The current robot has 4 motors:

- 2 stepper motors: `m1`, `m2`
- 2 MG995 servo motors: `m3`, `m4`

Upper layers should not need to know which motor is stepper or servo
controlled.

## Control tick contract (robot arm)

- **Single control loop**: Only `MotorManager._run_worker` runs the periodic tick
  (after `start()`). One tick should stay short (order of `1000 / control_hz` ms,
  e.g. ~20 ms at 50 Hz).
- **Coordinated motion**: Each tick applies the **same** `command_id` and the same
  interpolation clock to every motor. All joints receive their setpoint for that
  instant; no joint should block the tick while others wait (`sleep` loops,
  walking thousands of steps, blocking I/O).
- **`set_target` semantics**: Stores the **latest** command only (overwrite, no
  queue). Returns immediately; it does not move hardware itself.
- **Execution vs control**: `Motor.update()` must be a **non-blocking** tick
  handler. Long step pulses belong in an execution layer (shared scheduler,
  pigpio/DMA, external MCU, etc.), not inside the manager tick.

## MotorState.current_angle (Option A — trajectory setpoint)

- **`current_angle` / `target_angle`**: Trajectory **setpoints** from the same
  interpolation as `duration_ms` (what the motion planner intends at this tick).
- **`hw_angle` (optional)**: When present, best-effort **actual** angle from
  hardware step/pulse state (e.g. stepper steps executed). Real hardware may lag
  the trajectory; do not assume `current_angle == hw_angle` every tick.
- Option B (actual == measured steps only) is **not** used; tests and callers
  assume trajectory semantics unless they opt into `hw_angle`.

## Stepper execution backends

- **`rpi_gpio_shared`** (default for real STEP/DIR on Pi): Per-tick chunks are
  handled by a **single shared** background scheduler thread that walks pulses
  (round-robin across motors when multiple axes have work). This keeps one time
  base for coordinated motion without per-motor control threads.
- **`pigpio` / external MCU**: Preferred for production timing; document when
  wired (not required for mock-only tests).
- **`driver_options`**: `step_execution_backend`, `max_steps_per_tick` — see
  `config.py` / `motors.py`.

## Hard Rules

- Keep `MotorManager` as the only main public entry point.
- Do not put FastAPI, hand detection, robot behavior, session state, or web
  code in this package.
- Do not make upper layers choose driver types, pins, protocols, or motor
  hardware details.
- Preserve the `set_target(targets: dict[str, float], duration_ms: int)` contract.
- `set_target()` must require exactly all configured motor ids.
- New targets overwrite previous targets. Do not add a movement queue unless
  the user explicitly changes the product requirement.
- `set_target()` must not execute motor movement directly. It should validate
  and store the latest target, then return quickly.
- `MotorManager.start()` owns the background control-loop thread.
- The control loop must repeatedly read the latest target. Do not change it
  back to "execute one full command then check for the next command".
- Do not call motor hardware methods while holding
  `MotorManager._lock`.
- `MotorManager._command_condition` protects only latest-command state and
  worker wakeups.
- Per-motor state must be protected inside each motor instance, not by manager.
- Keep hardware-specific code inside the motor implementation module, not in
  `manager.py`.
- Do not pretend incomplete hardware integrations are production-ready. Leave
  `NotImplementedError` or explicit TODOs until real hardware behavior exists.
- Keep mock tests runnable on a normal development machine without Raspberry Pi
  hardware.

## Read First

Before changing behavior, inspect these files:

- `manager.py`: public API, target validation, controller construction.
- `config.py`: default 4-motor configuration, ids, and tunable hardware
  parameters.
- `models.py`: shared data structures and motor kinds.
- `motors.py`: per-motor state, hardware access, and motor-type behavior.
- `tests/test_motor_control.py`: current behavior contract.

## Current Architecture

Keep this dependency direction:

```text
upper layer
  -> MotorManager
      -> StepperMotor / ServoMotor / PM30225Motor
```

Responsibilities:

- `MotorManager`: validate complete target payloads, enforce angle ranges,
  expose `start()`, `stop()`, `home_all()`, `set_target()`, `get_state()`, and
  `emergency_stop()`. It also owns the background control-loop thread.
- `motors.py`: keep one class per supported motor family. Each motor class owns
  both its runtime state and its hardware/mock behavior.
- `config.py`: hardware identity, limits, pins/options, and driver selection.

## Existing Implementation Notes

- `pyproject.toml` should include `motor_control` from `packages`.
- The default config uses mock motor modes so tests can run without hardware.
- `set_target()` is non-blocking at the API layer. It stores the latest command;
  the manager control loop applies it after `start()` is called.
- The control loop currently runs at `control_hz=50.0` by default.
- Optional observability: `TURKISH_ICE_CREAM_MOTOR_TICK_WARN_MS` — log a warning when
  a tick exceeds this many milliseconds; `MotorManager(..., tick_timing_hook=...)`
  receives each tick duration in ms (for tests or profiling).
- Stepper motors currently require `home_all()` before `set_target()`.
- Default configs use mock motor modes. Hardware configs for `m1` through `m4`
  should be built with `build_m1_m4_hardware_configs()` after tuning the pin,
  angle, pulse-width, and stepper constants in `config.py`.
- Set `TURKISH_ICE_CREAM_MOTOR_PROFILE=hardware` in deployment to make
  `MotorManager()` use the hardware config. The default profile is `mock` so
  local tests run without Raspberry Pi hardware.

## How To Add Hardware Support

- Implement GPIO setup, enable, disable, direction, step pulse, limit switch
  read, stop, and cleanup inside `StepperMotor`.
- Keep Raspberry Pi library imports inside `motors.py`, never in `manager.py`.
- Replace the current mock-friendly immediate stepping with timed motion.
- Implement real homing: slow seek toward limit switch, debounce, optional
  backoff, then assign `home_angle`.
- Tune `HARDWARE_SERVO_OPTIONS` in `config.py` for MG995 pulse width, PWM
  frequency, GPIO mode, and signal pins.

## Testing Requirements

For package-level behavior, use mock-only unit tests:

```sh
python3 -m unittest tests.test_motor_control
```

Add or update tests when changing:

- motor ids or default config shape
- validation behavior
- background worker behavior
- control-loop retargeting behavior
- concurrent `set_target()` / `get_state()` behavior while the worker is running
- homing readiness behavior
- angle-to-step conversion
- servo duty-cycle conversion
- emergency stop behavior

Hardware tests should be opt-in and clearly separated from normal unit tests.
Do not make the default test command require Raspberry Pi hardware.

## Editing Guidelines

- Prefer small, local changes over broad refactors.
- Keep public API changes rare and deliberate.
- If motor ids change, update tests and any upper-layer caller in the same
  change.
- If adding a new motor kind, add it to `MotorKind`, config construction,
  controller construction, and tests.
- Keep the current flat split: `manager.py`, `config.py`, `models.py`, and
  `motors.py`. Add new files only when a new hardware family or real
  complexity appears.
- Keep comments short and useful. Use TODO comments only where a real hardware
  decision or implementation is still missing.
- Remove generated `__pycache__` directories before finishing.

## Common Pitfalls

- Do not let `manager.py` import Raspberry Pi GPIO or Serial libraries.
- Do not hold manager lifecycle locks while calling driver methods.
- Do not silently accept missing target motors.
- Do not queue target commands.
- Do not move motor execution back into `set_target()`.
- Do not make motor updates long-running/blocking. They should be short
  per-tick updates so high-frequency retargeting remains responsive.
- Do not make `home_all()` optional for motors configured with
  `requires_homing=True`.
