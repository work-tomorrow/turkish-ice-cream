"""Minimal default motor instances (m1~m4).

This module intentionally avoids MotorConfig/MotorState abstractions.
"""

from __future__ import annotations

import os
from typing import Any

from motor_control.models import MotorConfig, MotorKind

STEPPER_ANGLE_RANGE = (-180.0, 180.0)
SERVO_ANGLE_RANGE = (0.0, 180.0)

# Reasonable defaults for local tests.
DEFAULT_MAX_VELOCITY_DEG_S = 90.0

STEPPER_STEPS_PER_REV = 200
STEPPER_MICROSTEPS = 16

SERVO_FREQUENCY_HZ = 50.0
SERVO_MIN_PULSE_US = 500.0
SERVO_MAX_PULSE_US = 2500.0


def build_default_motors() -> dict[str, ServoMotor | StepperMotor]:
    profile = os.getenv("TURKISH_ICE_CREAM_MOTOR_PROFILE", "mock").strip().lower()
    if profile == "mock":
        return build_m1_m4_mock_motors()
    if profile == "hardware":
        return build_m1_m4_hardware_motors()
    raise ValueError("TURKISH_ICE_CREAM_MOTOR_PROFILE must be 'mock' or 'hardware'")


def build_m1_m4_mock_motors() -> dict[str, ServoMotor | StepperMotor]:
    return {
        "m1": StepperMotor(
            motor_id="m1",
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            max_velocity_deg_s=DEFAULT_MAX_VELOCITY_DEG_S,
            steps_per_rev=STEPPER_STEPS_PER_REV,
            microsteps=STEPPER_MICROSTEPS,
            driver="mock",
        ),
        "m2": StepperMotor(
            motor_id="m2",
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            max_velocity_deg_s=DEFAULT_MAX_VELOCITY_DEG_S,
            steps_per_rev=STEPPER_STEPS_PER_REV,
            microsteps=STEPPER_MICROSTEPS,
            driver="mock",
        ),
        "m3": ServoMotor(
            motor_id="m3",
            min_angle=SERVO_ANGLE_RANGE[0],
            max_angle=SERVO_ANGLE_RANGE[1],
            max_velocity_deg_s=DEFAULT_MAX_VELOCITY_DEG_S,
            pwm=ServoPwmConfig(
                signal_pin=None,
                frequency_hz=SERVO_FREQUENCY_HZ,
                min_pulse_us=SERVO_MIN_PULSE_US,
                max_pulse_us=SERVO_MAX_PULSE_US,
            ),
            driver="mock",
        ),
        "m4": ServoMotor(
            motor_id="m4",
            min_angle=SERVO_ANGLE_RANGE[0],
            max_angle=SERVO_ANGLE_RANGE[1],
            max_velocity_deg_s=DEFAULT_MAX_VELOCITY_DEG_S,
            pwm=ServoPwmConfig(
                signal_pin=None,
                frequency_hz=SERVO_FREQUENCY_HZ,
                min_pulse_us=SERVO_MIN_PULSE_US,
                max_pulse_us=SERVO_MAX_PULSE_US,
            ),
            driver="mock",
        ),
    }


def build_m1_m4_hardware_motors(
    *,
    stepper_options: dict[str, dict[str, Any]] | None = None,
    servo_options: dict[str, dict[str, Any]] | None = None,
) -> dict[str, ServoMotor | StepperMotor]:
    # Keep this as a thin option merge; actual GPIO stepping can be implemented later.
    stepper_options = stepper_options or {}
    servo_options = servo_options or {}

    def _stepper(motor_id: str) -> StepperMotor:
        opts = stepper_options.get(motor_id, {})
        max_vel = float(opts.get("max_velocity_deg_s", DEFAULT_MAX_VELOCITY_DEG_S))
        steps_per_rev = int(opts.get("steps_per_rev", STEPPER_STEPS_PER_REV))
        microsteps = int(opts.get("microsteps", STEPPER_MICROSTEPS))
        gear_ratio = float(opts.get("gear_ratio", 1.0))
        driver = opts.get("driver", "hardware")
        step_pin = opts.get("step_pin")
        dir_pin = opts.get("dir_pin")
        enable_pin = opts.get("enable_pin")
        return StepperMotor(
            motor_id=motor_id,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            max_velocity_deg_s=max_vel,
            steps_per_rev=steps_per_rev,
            microsteps=microsteps,
            gear_ratio=gear_ratio,
            driver=driver,
            step_pin=step_pin,
            dir_pin=dir_pin,
            enable_pin=enable_pin,
        )

    def _servo(motor_id: str) -> ServoMotor:
        opts = servo_options.get(motor_id, {})
        max_vel = float(opts.get("max_velocity_deg_s", DEFAULT_MAX_VELOCITY_DEG_S))
        driver = opts.get("driver", "hardware")
        return ServoMotor(
            motor_id=motor_id,
            min_angle=float(opts.get("min_angle", SERVO_ANGLE_RANGE[0])),
            max_angle=float(opts.get("max_angle", SERVO_ANGLE_RANGE[1])),
            max_velocity_deg_s=max_vel,
            pwm=ServoPwmConfig(
                signal_pin=opts.get("signal_pin"),
                frequency_hz=float(opts.get("frequency_hz", SERVO_FREQUENCY_HZ)),
                min_pulse_us=float(opts.get("min_pulse_us", SERVO_MIN_PULSE_US)),
                max_pulse_us=float(opts.get("max_pulse_us", SERVO_MAX_PULSE_US)),
            ),
            driver=driver,
        )

    return {"m1": _stepper("m1"), "m2": _stepper("m2"), "m3": _servo("m3"), "m4": _servo("m4")}


def build_default_motor_configs() -> list[MotorConfig]:
    return [
        MotorConfig(
            id="m1",
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            driver_type="mock_stepper",
        ),
        MotorConfig(
            id="m2",
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            driver_type="mock_stepper",
        ),
        MotorConfig(
            id="m3",
            kind=MotorKind.SERVO,
            min_angle=SERVO_ANGLE_RANGE[0],
            max_angle=SERVO_ANGLE_RANGE[1],
            driver_type="mock_servo",
        ),
        MotorConfig(
            id="m4",
            kind=MotorKind.SERVO,
            min_angle=SERVO_ANGLE_RANGE[0],
            max_angle=SERVO_ANGLE_RANGE[1],
            driver_type="mock_servo",
        ),
    ]


def build_m1_m4_hardware_configs(
    *,
    stepper_options: dict[str, dict[str, Any]] | None = None,
    servo_options: dict[str, dict[str, Any]] | None = None,
) -> list[MotorConfig]:
    stepper_options = stepper_options or {}
    servo_options = servo_options or {}

    def _stepper_config(motor_id: str) -> MotorConfig:
        opts = stepper_options.get(motor_id, {})
        driver_type = opts.get("driver", "nema17_step_dir")
        driver_options = {
            "max_velocity_deg_s": float(opts.get("max_velocity_deg_s", DEFAULT_MAX_VELOCITY_DEG_S)),
            "steps_per_rev": int(opts.get("steps_per_rev", STEPPER_STEPS_PER_REV)),
            "microsteps": int(opts.get("microsteps", STEPPER_MICROSTEPS)),
            "gear_ratio": float(opts.get("gear_ratio", 1.0)),
            "step_pin": opts.get("step_pin"),
            "dir_pin": opts.get("dir_pin"),
            "enable_pin": opts.get("enable_pin"),
        }
        return MotorConfig(
            id=motor_id,
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            driver_type=driver_type,
            driver_options=driver_options,
        )

    def _servo_config(motor_id: str) -> MotorConfig:
        opts = servo_options.get(motor_id, {})
        driver_type = opts.get("driver", "mg995_servo")
        driver_options = {
            "max_velocity_deg_s": float(opts.get("max_velocity_deg_s", DEFAULT_MAX_VELOCITY_DEG_S)),
            "min_angle": float(opts.get("min_angle", SERVO_ANGLE_RANGE[0])),
            "max_angle": float(opts.get("max_angle", SERVO_ANGLE_RANGE[1])),
            "frequency_hz": float(opts.get("frequency_hz", SERVO_FREQUENCY_HZ)),
            "min_pulse_us": float(opts.get("min_pulse_us", SERVO_MIN_PULSE_US)),
            "max_pulse_us": float(opts.get("max_pulse_us", SERVO_MAX_PULSE_US)),
            "signal_pin": opts.get("signal_pin"),
        }
        return MotorConfig(
            id=motor_id,
            kind=MotorKind.SERVO,
            min_angle=float(opts.get("min_angle", SERVO_ANGLE_RANGE[0])),
            max_angle=float(opts.get("max_angle", SERVO_ANGLE_RANGE[1])),
            driver_type=driver_type,
            driver_options=driver_options,
        )

    return [_stepper_config("m1"), _stepper_config("m2"), _servo_config("m3"), _servo_config("m4")]


DEFAULT_MOTOR_CONFIGS = build_default_motor_configs()
