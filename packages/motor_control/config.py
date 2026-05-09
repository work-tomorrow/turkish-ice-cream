"""Minimal default motor wiring for m1~m4.

Pin numbers are BCM GPIO numbers.
"""

from __future__ import annotations

from motor_control.motors import ServoMotor, StepperMotor


def build_default_motors() -> dict[str, StepperMotor | ServoMotor]:
    return {
        "m1": StepperMotor("m1", step_pin=17, dir_pin=27),
        "m2": StepperMotor("m2", step_pin=23, dir_pin=24),
        "m3": ServoMotor("m3", pin=18),
        "m4": ServoMotor("m4", pin=19),
    }
