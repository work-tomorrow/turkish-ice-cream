import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages"))

from motor_control.motors import ServoMotor, ServoPwmConfig


class FakePwm:
    instances: list["FakePwm"] = []

    def __init__(self, pin: int, frequency: float) -> None:
        self.pin = pin
        self.frequency = frequency
        self.started: list[float] = []
        self.duty_cycles: list[float] = []
        self.stopped = False
        FakePwm.instances.append(self)

    def start(self, duty_cycle: float) -> None:
        self.started.append(duty_cycle)

    def ChangeDutyCycle(self, duty_cycle: float) -> None:
        self.duty_cycles.append(duty_cycle)

    def stop(self) -> None:
        self.stopped = True


class FakeGpio:
    BCM = "BCM"
    OUT = "OUT"

    def __init__(self) -> None:
        self.modes: list[str] = []
        self.setups: list[tuple[int, str]] = []

    def setmode(self, mode: str) -> None:
        self.modes.append(mode)

    def setup(self, pin: int, mode: str) -> None:
        self.setups.append((pin, mode))

    def PWM(self, pin: int, frequency: float) -> FakePwm:
        return FakePwm(pin, frequency)


class ServoPwmTest(unittest.TestCase):
    def setUp(self) -> None:
        FakePwm.instances.clear()
        self.gpio = FakeGpio()
        rpi_module = types.ModuleType("RPi")
        rpi_module.GPIO = self.gpio
        self.modules = {
            "RPi": rpi_module,
            "RPi.GPIO": self.gpio,
        }

    def test_hardware_servo_reuses_pwm_between_angle_updates(self) -> None:
        servo = ServoMotor(
            motor_id="m4",
            min_angle=0.0,
            max_angle=180.0,
            max_velocity_deg_s=90.0,
            pwm=ServoPwmConfig(signal_pin=19),
            driver="hardware",
        )

        with patch.dict(sys.modules, self.modules):
            servo.set_target_angle(10.0)
            servo.set_target_angle(20.0)
            servo.shutdown()

        self.assertEqual(len(FakePwm.instances), 1)
        pwm = FakePwm.instances[0]
        self.assertEqual(pwm.pin, 19)
        self.assertEqual(pwm.started, [0.0])
        self.assertEqual(len(pwm.duty_cycles), 3)
        self.assertAlmostEqual(pwm.duty_cycles[0], 3.0555555555555554)
        self.assertAlmostEqual(pwm.duty_cycles[1], 3.611111111111111)
        self.assertEqual(pwm.duty_cycles[2], 0.0)
        self.assertTrue(pwm.stopped)


if __name__ == "__main__":
    unittest.main()
