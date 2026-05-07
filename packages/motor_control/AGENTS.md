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

The current robot has 5 motors:

- 2 stepper motors: `stepper_1`, `stepper_2`
- 2 TT motors: `tt_1`, `tt_2`
- 1 PM30225 coreless motor from 強茂 over Serial/UART: `pm30225`

Upper layers should not need to know which motor is stepper, TT, or PM30225
controlled.

## Hard Rules

- Keep `MotorManager` as the only main public entry point.
- Do not put FastAPI, hand detection, robot behavior, session state, or web
  code in this package.
- Do not make upper layers choose driver types, pins, protocols, or motor
  hardware details.
- Preserve the `set_target(targets: dict[str, float], duration_ms: int)` contract.
- `set_target()` must require exactly all 5 known motor ids.
- New targets overwrite previous targets. Do not add a movement queue unless
  the user explicitly changes the product requirement.
- `set_target()` must not execute motor movement directly. It should validate
  and store the latest target, then return quickly.
- `MotorManager.start()` owns the background control-loop thread.
- The control loop must repeatedly read the latest target. Do not change it
  back to "execute one full command then check for the next command".
- Do not call controller or driver hardware methods while holding
  `MotorManager._lock`.
- `MotorManager._command_condition` protects only latest-command state and
  worker wakeups.
- Controller state must be protected inside each controller, not by manager.
- Keep hardware-specific code in `drivers/`, not in `manager.py`.
- Keep motor-type angle semantics in `controllers/`, not in upper layers.
- Do not pretend incomplete hardware drivers are production-ready. Leave
  `NotImplementedError` or explicit TODOs until real hardware behavior exists.
- Keep mock tests runnable on a normal development machine without Raspberry Pi
  hardware.

## Read First

Before changing behavior, inspect these files:

- `manager.py`: public API, target validation, controller construction.
- `config.py`: default 5-motor configuration and ids.
- `models.py`: shared data structures and motor kinds.
- `controllers/base.py`: controller protocol and shared state handling.
- `controllers/stepper.py`: stepper angle-to-step behavior and homing state.
- `controllers/tt.py`: TT angle behavior, currently approximate.
- `controllers/pm30225.py`: serial command formatting for the PM30225 motor.
- `drivers/mock.py`: mock drivers for local tests.
- `drivers/stepper.py`: TODO STEP/DIR Raspberry Pi driver.
- `drivers/tt.py`: TODO TT motor PWM/direction driver.
- `drivers/pm30225.py`: TODO Serial/UART PM30225 driver.
- `tests/test_motor_control.py`: current behavior contract.

## Current Architecture

Keep this dependency direction:

```text
upper layer
  -> MotorManager
      -> MotorController
          -> MotorDriver
          -> optional FeedbackReader
```

Responsibilities:

- `MotorManager`: validate complete target payloads, enforce angle ranges,
  expose `start()`, `stop()`, `home_all()`, `set_target()`, `get_state()`, and
  `emergency_stop()`. It also owns the background control-loop thread.
- `controllers/`: translate angle-level commands into motor-type-specific
  behavior. Controllers should support small per-tick updates.
- `drivers/`: touch GPIO, PWM, Serial/UART, motor HATs, or other hardware APIs.
- `config.py`: hardware identity, limits, pins/options, and driver selection.

Controller vs driver rule of thumb:

- A controller answers: "Given a target angle and duration, what does this motor
  type need to do?"
- A driver answers: "How do I send that command to this piece of hardware?"
- If the code mentions angle limits, homing state, approximate control, or
  angle-to-step conversion, it belongs in `controllers/`.
- If the code mentions GPIO pins, PWM duty cycle, step pulses, serial ports, or
  vendor libraries, it belongs in `drivers/`.

## Existing Implementation Notes

- `pyproject.toml` should include `motor_control` from `packages`.
- The default config uses mock drivers so tests can run without hardware.
- `set_target()` is non-blocking at the API layer. It stores the latest command;
  the manager control loop applies it after `start()` is called.
- The control loop currently runs at `control_hz=50.0` by default.
- Stepper motors currently require `home_all()` before `set_target()`.
- TT motors currently use `position_control_mode="approximate"` because encoder
  feedback is not decided.
- The PM30225 motor currently uses this intended command family:

```text
SET_TARGET motor_id angle duration_ms
HOME motor_id
STOP
STATUS
```

## How To Add Hardware Support

Stepper work:

- Implement GPIO setup, enable, disable, direction, step pulse, limit switch
  read, stop, and cleanup in `drivers/stepper.py`.
- Keep Raspberry Pi library imports inside `drivers/stepper.py`.
- Improve `StepperController` in `controllers/stepper.py` after the driver
  exists.
- Replace the current mock-friendly immediate stepping with timed motion.
- Implement real homing: slow seek toward limit switch, debounce, optional
  backoff, then assign `home_angle`.

TT motor work:

- Decide whether each TT motor has encoder feedback.
- If feedback exists, add the smallest feedback abstraction needed and update
  `TTController` in `controllers/tt.py`.
- If no feedback exists, keep `MotorState.approximate=True` and make the
  approximation explicit in config/tests/docs.
- Implement actual PWM/direction/brake behavior in `drivers/tt.py`.

PM30225 motor work:

- Implement `PM30225SerialDriver` in `drivers/pm30225.py`.
- Prefer newline-delimited commands unless the PM30225 spec says otherwise.
- Add acknowledgement/timeout handling if the PM30225 supports it.
- Keep PM30225 protocol formatting inside the PM30225 driver/controller, not
  in `MotorManager`.

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
- approximate vs closed-loop TT behavior
- serial command formatting
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
- Keep the current split by responsibility and motor type. Add new files only
  when a new hardware family or real complexity appears.
- Keep comments short and useful. Use TODO comments only where a real hardware
  decision or implementation is still missing.
- Remove generated `__pycache__` directories before finishing.

## Common Pitfalls

- Do not let `manager.py` import Raspberry Pi GPIO or Serial libraries.
- Do not hold manager lifecycle locks while calling driver methods.
- Do not silently accept missing target motors.
- Do not queue target commands.
- Do not move controller execution back into `set_target()`.
- Do not make controller updates long-running/blocking. They should be short
  per-tick updates so high-frequency retargeting remains responsive.
- Do not hide that TT angle control is approximate without feedback.
- Do not mix mock behavior into real hardware drivers.
- Do not make `home_all()` optional for motors configured with
  `requires_homing=True`.
