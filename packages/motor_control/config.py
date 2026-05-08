"""Default motor configuration and instance builders for m1~m4."""

from __future__ import annotations

import os
from typing import Any, Iterable

from motor_control.models import MotorConfig, MotorKind
from motor_control.motors import ServoMotor, StepperMotor

STEPPER_ANGLE_RANGE = (-180.0, 180.0)
SERVO_ANGLE_RANGE = (0.0, 180.0)

STEPPER_HOME_ANGLE = 0.0
SERVO_HOME_ANGLE = 90.0

DEFAULT_STEPPER_MAX_VELOCITY_DEG_S = 45.0
DEFAULT_SERVO_MAX_VELOCITY_DEG_S = 90.0

STEPPER_STEPS_PER_REV = 200
STEPPER_MICROSTEPS = 16

SERVO_FREQUENCY_HZ = 50.0
SERVO_MIN_PULSE_US = 500.0
SERVO_MAX_PULSE_US = 2500.0
SERVO_HOLD_TIME_S = 0.8

HARDWARE_STEPPER_OPTIONS: dict[str, dict[str, Any]] = {
    "m1": {"step_pin": 17, "dir_pin": 27, "driver": "hardware"},
    "m2": {"step_pin": 23, "dir_pin": 24, "driver": "hardware"},
}

HARDWARE_SERVO_OPTIONS: dict[str, dict[str, Any]] = {
    "m3": {"signal_pin": 18},
    "m4": {"signal_pin": 19},
}


def _merge_options(
    defaults: dict[str, dict[str, Any]],
    overrides: dict[str, dict[str, Any]] | None,
) -> dict[str, dict[str, Any]]:
    merged = {motor_id: dict(options) for motor_id, options in defaults.items()}
    for motor_id, options in (overrides or {}).items():
        merged.setdefault(motor_id, {})
        merged[motor_id].update(options)
    return merged


def build_m1_m4_mock_configs() -> list[MotorConfig]:
    return [
        MotorConfig(
            id="m1",
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            home_angle=STEPPER_HOME_ANGLE,
            driver_type="mock_stepper",
            driver_options={
                "max_velocity_deg_s": DEFAULT_STEPPER_MAX_VELOCITY_DEG_S,
            },
            requires_homing=True,
            steps_per_rev=STEPPER_STEPS_PER_REV,
            microsteps=STEPPER_MICROSTEPS,
        ),
        MotorConfig(
            id="m2",
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            home_angle=STEPPER_HOME_ANGLE,
            driver_type="mock_stepper",
            driver_options={
                "max_velocity_deg_s": DEFAULT_STEPPER_MAX_VELOCITY_DEG_S,
            },
            requires_homing=True,
            steps_per_rev=STEPPER_STEPS_PER_REV,
            microsteps=STEPPER_MICROSTEPS,
        ),
        MotorConfig(
            id="m3",
            kind=MotorKind.SERVO,
            min_angle=SERVO_ANGLE_RANGE[0],
            max_angle=SERVO_ANGLE_RANGE[1],
            home_angle=SERVO_HOME_ANGLE,
            driver_type="mock_servo",
            driver_options={
                "frequency_hz": SERVO_FREQUENCY_HZ,
                "min_pulse_us": SERVO_MIN_PULSE_US,
                "max_pulse_us": SERVO_MAX_PULSE_US,
                "hold_time_s": SERVO_HOLD_TIME_S,
                "max_velocity_deg_s": DEFAULT_SERVO_MAX_VELOCITY_DEG_S,
            },
        ),
        MotorConfig(
            id="m4",
            kind=MotorKind.SERVO,
            min_angle=SERVO_ANGLE_RANGE[0],
            max_angle=SERVO_ANGLE_RANGE[1],
            home_angle=SERVO_HOME_ANGLE,
            driver_type="mock_servo",
            driver_options={
                "frequency_hz": SERVO_FREQUENCY_HZ,
                "min_pulse_us": SERVO_MIN_PULSE_US,
                "max_pulse_us": SERVO_MAX_PULSE_US,
                "hold_time_s": SERVO_HOLD_TIME_S,
                "max_velocity_deg_s": DEFAULT_SERVO_MAX_VELOCITY_DEG_S,
            },
        ),
    ]


def build_m1_m4_hardware_configs(
    *,
    stepper_options: dict[str, dict[str, Any]] | None = None,
    servo_options: dict[str, dict[str, Any]] | None = None,
) -> list[MotorConfig]:
    resolved_stepper_options = _merge_options(HARDWARE_STEPPER_OPTIONS, stepper_options)
    resolved_servo_options = _merge_options(HARDWARE_SERVO_OPTIONS, servo_options)

    return [
        MotorConfig(
            id="m1",
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            home_angle=STEPPER_HOME_ANGLE,
            driver_type="nema17_step_dir",
            driver_options={
                "max_velocity_deg_s": DEFAULT_STEPPER_MAX_VELOCITY_DEG_S,
                "step_pulse_us": 200.0,
                **resolved_stepper_options["m1"],
            },
            requires_homing=True,
            steps_per_rev=int(
                resolved_stepper_options["m1"].get("steps_per_rev", STEPPER_STEPS_PER_REV)
            ),
            microsteps=int(
                resolved_stepper_options["m1"].get("microsteps", STEPPER_MICROSTEPS)
            ),
            gear_ratio=float(resolved_stepper_options["m1"].get("gear_ratio", 1.0)),
        ),
        MotorConfig(
            id="m2",
            kind=MotorKind.STEPPER,
            min_angle=STEPPER_ANGLE_RANGE[0],
            max_angle=STEPPER_ANGLE_RANGE[1],
            home_angle=STEPPER_HOME_ANGLE,
            driver_type="nema17_step_dir",
            driver_options={
                "max_velocity_deg_s": DEFAULT_STEPPER_MAX_VELOCITY_DEG_S,
                "step_pulse_us": 200.0,
                **resolved_stepper_options["m2"],
            },
            requires_homing=True,
            steps_per_rev=int(
                resolved_stepper_options["m2"].get("steps_per_rev", STEPPER_STEPS_PER_REV)
            ),
            microsteps=int(
                resolved_stepper_options["m2"].get("microsteps", STEPPER_MICROSTEPS)
            ),
            gear_ratio=float(resolved_stepper_options["m2"].get("gear_ratio", 1.0)),
        ),
        MotorConfig(
            id="m3",
            kind=MotorKind.SERVO,
            min_angle=float(resolved_servo_options["m3"].get("min_angle", SERVO_ANGLE_RANGE[0])),
            max_angle=float(resolved_servo_options["m3"].get("max_angle", SERVO_ANGLE_RANGE[1])),
            home_angle=float(resolved_servo_options["m3"].get("home_angle", SERVO_HOME_ANGLE)),
            driver_type="mg995_servo",
            driver_options={
                "frequency_hz": SERVO_FREQUENCY_HZ,
                "min_pulse_us": SERVO_MIN_PULSE_US,
                "max_pulse_us": SERVO_MAX_PULSE_US,
                "hold_time_s": SERVO_HOLD_TIME_S,
                "max_velocity_deg_s": DEFAULT_SERVO_MAX_VELOCITY_DEG_S,
                **resolved_servo_options["m3"],
            },
        ),
        MotorConfig(
            id="m4",
            kind=MotorKind.SERVO,
            min_angle=float(resolved_servo_options["m4"].get("min_angle", SERVO_ANGLE_RANGE[0])),
            max_angle=float(resolved_servo_options["m4"].get("max_angle", SERVO_ANGLE_RANGE[1])),
            home_angle=float(resolved_servo_options["m4"].get("home_angle", SERVO_HOME_ANGLE)),
            driver_type="mg995_servo",
            driver_options={
                "frequency_hz": SERVO_FREQUENCY_HZ,
                "min_pulse_us": SERVO_MIN_PULSE_US,
                "max_pulse_us": SERVO_MAX_PULSE_US,
                "hold_time_s": SERVO_HOLD_TIME_S,
                "max_velocity_deg_s": DEFAULT_SERVO_MAX_VELOCITY_DEG_S,
                **resolved_servo_options["m4"],
            },
        ),
    ]


def build_default_motor_configs() -> list[MotorConfig]:
    profile = os.getenv("TURKISH_ICE_CREAM_MOTOR_PROFILE", "mock").strip().lower()
    if profile == "mock":
        return build_m1_m4_mock_configs()
    if profile == "hardware":
        return build_m1_m4_hardware_configs()
    raise ValueError("TURKISH_ICE_CREAM_MOTOR_PROFILE must be 'mock' or 'hardware'")


DEFAULT_MOTOR_CONFIGS = build_default_motor_configs()


def build_motors_from_configs(
    configs: Iterable[MotorConfig],
) -> dict[str, ServoMotor | StepperMotor]:
    motors: dict[str, ServoMotor | StepperMotor] = {}
    for config in configs:
        if config.kind is MotorKind.STEPPER:
            motors[config.id] = StepperMotor(config)
            continue
        if config.kind is MotorKind.SERVO:
            motors[config.id] = ServoMotor(config)
            continue
        raise ValueError(f"Unsupported motor kind for {config.id}: {config.kind}")
    return motors


def build_default_motors() -> dict[str, ServoMotor | StepperMotor]:
    return build_motors_from_configs(build_default_motor_configs())
