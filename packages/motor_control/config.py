"""Minimal motor runtime profile and wiring for m1~m4.

Pin numbers are BCM GPIO numbers.
"""

from __future__ import annotations

import os
from typing import Literal

from motor_control.mock_motors import MockMotor, build_mock_motors
from motor_control.motors import ServoMotor, StepperMotor

MOTOR_PROFILE_ENV = "TURKISH_ICE_CREAM_MOTOR_PROFILE"
MotorProfile = Literal["mock", "hardware"]


def get_motor_profile() -> MotorProfile:
    profile = os.getenv(MOTOR_PROFILE_ENV, "mock").strip().lower()
    if profile not in {"mock", "hardware"}:
        raise ValueError(
            (f"{MOTOR_PROFILE_ENV} must be 'mock' or 'hardware', " f"got {profile!r}")
        )

    return profile


def build_default_motors(
    profile: MotorProfile | None = None,
) -> dict[str, MockMotor | StepperMotor | ServoMotor]:
    selected_profile = profile or get_motor_profile()
    if selected_profile == "mock":
        return build_mock_motors()

    if selected_profile == "hardware":
        return build_hardware_motors()

    raise ValueError(f"Unknown motor profile: {selected_profile!r}")


def build_hardware_motors() -> dict[str, StepperMotor | ServoMotor]:
    return {
        "m1": StepperMotor(
            "m1",
            step_pin=20,
            dir_pin=16,
            enable_pin=21,
            enable_active_high=True,
            min_angle=0,
            max_angle=0,
            steps_per_second=400.0,
        ),  # 轉盤變大
        "m2": ServoMotor(
            "m2", pin=18, max_angle=105, min_angle=-5, home_angle=25
        ),  # 下變大
        "m3": ServoMotor(
            "m3", pin=23, max_angle=150, min_angle=30, home_angle=90
        ),  # 低頭變大
        "m4": ServoMotor(
            "m4", pin=24, max_angle=150, min_angle=30, home_angle=90
        ),  # 順時變大
    }
