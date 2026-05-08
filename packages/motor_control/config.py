"""Default 4-motor configuration for the Turkish ice cream robot."""

from __future__ import annotations

from copy import deepcopy
import os
from typing import Any

from motor_control.models import MotorConfig, MotorKind

MOTOR_IDS = ("m1", "m2", "m3", "m4")

# Tunable motor parameters live here so hardware calibration does not require
# touching controller or driver code.
STEPPER_DEFAULTS: dict[str, Any] = {
    "min_angle": -180.0,
    "max_angle": 180.0,
    "home_angle": 0.0,
    "steps_per_rev": 200,
    "microsteps": 16,
    "gear_ratio": 1.0,
}

SERVO_DEFAULTS: dict[str, Any] = {
    "min_angle": 0.0,
    "max_angle": 180.0,
    "home_angle": 90.0,
    "frequency_hz": 50.0,
    "min_pulse_us": 500.0,
    "max_pulse_us": 2500.0,
    "initial_angle": 90.0,
}

HARDWARE_STEPPER_OPTIONS: dict[str, dict[str, Any]] = {
    "m1": {
        "step_pin": None,
        "dir_pin": None,
        "enable_pin": None,
        "limit_pin": None,
        "gpio_mode": "BCM",
        "enable_active_low": True,
        "direction_high_for_positive": True,
        "limit_active_low": True,
        "step_pulse_us": 800.0,
        "home_direction": -1,
        "home_step_delay_s": 0.003,
        "home_backoff_steps": 0,
        "max_home_steps": 10_000,
        "cleanup_on_stop": False,
    },
    "m2": {
        "step_pin": None,
        "dir_pin": None,
        "enable_pin": None,
        "limit_pin": None,
        "gpio_mode": "BCM",
        "enable_active_low": True,
        "direction_high_for_positive": True,
        "limit_active_low": True,
        "step_pulse_us": 800.0,
        "home_direction": -1,
        "home_step_delay_s": 0.003,
        "home_backoff_steps": 0,
        "max_home_steps": 10_000,
        "cleanup_on_stop": False,
    },
}

HARDWARE_SERVO_OPTIONS: dict[str, dict[str, Any]] = {
    "m3": {
        "signal_pin": None,
        "frequency_hz": SERVO_DEFAULTS["frequency_hz"],
        "min_angle": SERVO_DEFAULTS["min_angle"],
        "max_angle": SERVO_DEFAULTS["max_angle"],
        "min_pulse_us": SERVO_DEFAULTS["min_pulse_us"],
        "max_pulse_us": SERVO_DEFAULTS["max_pulse_us"],
        "gpio_mode": "BCM",
        "cleanup_on_stop": False,
        "initial_angle": SERVO_DEFAULTS["initial_angle"],
    },
    "m4": {
        "signal_pin": None,
        "frequency_hz": SERVO_DEFAULTS["frequency_hz"],
        "min_angle": SERVO_DEFAULTS["min_angle"],
        "max_angle": SERVO_DEFAULTS["max_angle"],
        "min_pulse_us": SERVO_DEFAULTS["min_pulse_us"],
        "max_pulse_us": SERVO_DEFAULTS["max_pulse_us"],
        "gpio_mode": "BCM",
        "cleanup_on_stop": False,
        "initial_angle": SERVO_DEFAULTS["initial_angle"],
    },
}


def build_m1_m4_mock_configs() -> list[MotorConfig]:
    """Build a local-test config: m1/m2 steppers and m3/m4 servos."""

    return [
        _stepper_config("m1", driver_type="mock_stepper", driver_options={}),
        _stepper_config("m2", driver_type="mock_stepper", driver_options={}),
        _servo_config("m3", driver_type="mock_servo", driver_options={}),
        _servo_config("m4", driver_type="mock_servo", driver_options={}),
    ]


def build_m1_m4_hardware_configs(
    *,
    stepper_options: dict[str, dict[str, Any]] | None = None,
    servo_options: dict[str, dict[str, Any]] | None = None,
) -> list[MotorConfig]:
    """Build a Raspberry Pi hardware config after filling pin/options above."""

    steppers = _merged_options(HARDWARE_STEPPER_OPTIONS, stepper_options)
    servos = _merged_options(HARDWARE_SERVO_OPTIONS, servo_options)
    return [
        _stepper_config("m1", driver_type="nema17_step_dir", driver_options=steppers["m1"]),
        _stepper_config("m2", driver_type="nema17_step_dir", driver_options=steppers["m2"]),
        _servo_config("m3", driver_type="mg995_servo", driver_options=servos["m3"]),
        _servo_config("m4", driver_type="mg995_servo", driver_options=servos["m4"]),
    ]


def build_default_motor_configs() -> list[MotorConfig]:
    """Build configs from TURKISH_ICE_CREAM_MOTOR_PROFILE."""

    profile = os.getenv("TURKISH_ICE_CREAM_MOTOR_PROFILE", "mock").strip().lower()
    if profile == "mock":
        return build_m1_m4_mock_configs()
    if profile == "hardware":
        return build_m1_m4_hardware_configs()
    raise ValueError(
        "TURKISH_ICE_CREAM_MOTOR_PROFILE must be either 'mock' or 'hardware'"
    )


def _merged_options(
    defaults: dict[str, dict[str, Any]],
    overrides: dict[str, dict[str, Any]] | None,
) -> dict[str, dict[str, Any]]:
    merged = deepcopy(defaults)
    if overrides is None:
        return merged

    for motor_id, options in overrides.items():
        if motor_id not in merged:
            raise ValueError(f"Unknown motor id: {motor_id}")
        merged[motor_id].update(options)
    return merged


def _stepper_config(
    motor_id: str,
    *,
    driver_type: str,
    driver_options: dict[str, Any],
) -> MotorConfig:
    return MotorConfig(
        id=motor_id,
        kind=MotorKind.STEPPER,
        min_angle=STEPPER_DEFAULTS["min_angle"],
        max_angle=STEPPER_DEFAULTS["max_angle"],
        home_angle=STEPPER_DEFAULTS["home_angle"],
        driver_type=driver_type,
        driver_options=driver_options,
        requires_homing=True,
        steps_per_rev=STEPPER_DEFAULTS["steps_per_rev"],
        microsteps=STEPPER_DEFAULTS["microsteps"],
        gear_ratio=STEPPER_DEFAULTS["gear_ratio"],
    )


def _servo_config(
    motor_id: str,
    *,
    driver_type: str,
    driver_options: dict[str, Any],
) -> MotorConfig:
    return MotorConfig(
        id=motor_id,
        kind=MotorKind.SERVO,
        min_angle=SERVO_DEFAULTS["min_angle"],
        max_angle=SERVO_DEFAULTS["max_angle"],
        home_angle=SERVO_DEFAULTS["home_angle"],
        driver_type=driver_type,
        driver_options=driver_options,
    )


DEFAULT_MOTOR_CONFIGS = build_default_motor_configs()
