import unittest
from threading import Event
from unittest.mock import patch

from motor_control import (
    MotorKind,
    MotorManager,
    StepperMotor,
    build_default_motors,
    get_motor_profile,
)
from motor_control.config import MOTOR_PROFILE_ENV
from motor_control.mock_motors import MockMotor
from motor_control.mock_motors import build_mock_motors


class MotorManagerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manager = MotorManager(
            motors=build_mock_motors(),
            dt_s=0.1,
            max_velocity_deg_s={
                "m1": 45.0,
                "m2": 180.0,
                "m3": 180.0,
                "m4": 180.0,
            },
        )

    def tearDown(self) -> None:
        self.manager.cleanup()

    def test_loads_four_m1_m4_motors(self) -> None:
        state = self.manager.get_state()

        self.assertEqual(set(state["motors"]), {"m1", "m2", "m3", "m4"})
        self.assertEqual(
            state["max_velocity_deg_s"],
            {"m1": 45.0, "m2": 180.0, "m3": 180.0, "m4": 180.0},
        )
        self.assertEqual(state["motors"]["m1"]["kind"], MotorKind.STEPPER)
        self.assertEqual(state["motors"]["m2"]["kind"], MotorKind.SERVO)
        self.assertEqual(state["motors"]["m3"]["kind"], MotorKind.SERVO)
        self.assertEqual(state["motors"]["m4"]["kind"], MotorKind.SERVO)

    def test_default_motor_profile_is_explicit_mock(self) -> None:
        with patch.dict("os.environ", {}, clear=True):
            self.assertEqual(get_motor_profile(), "mock")
            motors = build_default_motors()

        self.assertEqual(set(motors), {"m1", "m2", "m3", "m4"})
        self.assertTrue(all(isinstance(motor, MockMotor) for motor in motors.values()))

    def test_rejects_unknown_motor_profile(self) -> None:
        with patch.dict("os.environ", {MOTOR_PROFILE_ENV: "surprise"}):
            with self.assertRaises(ValueError):
                build_default_motors()

    def test_rejects_incomplete_targets(self) -> None:
        with self.assertRaises(ValueError):
            self.manager.set_target_angles({"m1": 0.0})

    def test_rejects_unknown_targets(self) -> None:
        targets = {"m1": 0.0, "m2": 0.0, "m3": 90.0, "m4": 90.0, "m5": 0.0}

        with self.assertRaises(ValueError):
            self.manager.set_target_angles(targets)

    def test_clamps_target_into_motor_range(self) -> None:
        self.manager.set_target_angles(
            {"m1": 999.0, "m2": -999.0, "m3": 999.0, "m4": -999.0}
        )

        state = self.manager.get_state()
        self.assertEqual(state["motors"]["m1"]["current_angle"], 4.5)
        self.assertEqual(state["motors"]["m2"]["current_angle"], 72.0)
        self.assertEqual(state["motors"]["m3"]["current_angle"], 108.0)
        self.assertEqual(state["motors"]["m4"]["current_angle"], 72.0)

    def test_speed_limit_caps_single_tick_motion(self) -> None:
        self.manager.set_target_angles(
            {"m1": 180.0, "m2": -180.0, "m3": 180.0, "m4": 0.0}
        )

        state = self.manager.get_state()
        self.assertEqual(state["motors"]["m1"]["current_angle"], 4.5)
        self.assertEqual(state["motors"]["m2"]["current_angle"], 72.0)
        self.assertEqual(state["motors"]["m3"]["current_angle"], 108.0)
        self.assertEqual(state["motors"]["m4"]["current_angle"], 72.0)

    def test_default_servo_speed_limit_is_30_deg_per_100ms(self) -> None:
        manager = MotorManager(motors=build_mock_motors(), dt_s=0.1)
        self.addCleanup(manager.cleanup)

        manager.set_target_angles({"m1": 180.0, "m2": 180.0, "m3": 0.0, "m4": 180.0})
        state = manager.get_state()

        self.assertEqual(state["motors"]["m2"]["current_angle"], 120.0)
        self.assertEqual(state["motors"]["m3"]["current_angle"], 60.0)
        self.assertEqual(state["motors"]["m4"]["current_angle"], 120.0)

    def test_home_all_moves_to_physical_home_after_zero_redefinition(self) -> None:
        self.manager.set_target_angles({"m1": 0, "m2": 0, "m3": 120.0, "m4": 60.0})
        self.manager.set_current_angle_zero("m3")

        self.manager.home_all()
        state = self.manager.get_state()

        self.assertEqual(state["motors"]["m1"]["target_angle"], 0.0)
        self.assertEqual(state["motors"]["m2"]["target_angle"], 90.0)
        self.assertEqual(state["motors"]["m3"]["target_angle"], -18.0)
        self.assertEqual(state["motors"]["m4"]["target_angle"], 90.0)

    def test_emergency_stop_freezes_pose_but_next_command_can_continue(self) -> None:
        self.manager.set_target_angles({"m1": 4.5, "m2": -4.5, "m3": 108.0, "m4": 72.0})
        self.manager.emergency_stop()
        state = self.manager.get_state()

        self.assertEqual(state["current_targets"]["m1"], 4.5)
        self.assertEqual(state["current_targets"]["m3"], 108.0)

        self.manager.set_target_angles({"m1": 0.0, "m2": 0.0, "m3": 90.0, "m4": 90.0})
        state = self.manager.get_state()
        self.assertEqual(state["motors"]["m1"]["current_angle"], 0.0)
        self.assertEqual(state["motors"]["m3"]["current_angle"], 90.0)

    def test_set_current_angle_zero_redefines_future_logical_angles(self) -> None:
        self.manager.set_target_angles({"m1": 0.0, "m2": 0.0, "m3": 108.0, "m4": 90.0})
        self.manager.set_current_angle_zero("m3")

        state = self.manager.get_state()
        self.assertEqual(state["motors"]["m3"]["current_angle"], 0.0)
        self.assertEqual(state["motors"]["m3"]["target_angle"], 0.0)
        self.assertEqual(state["motors"]["m3"]["min_angle"], -108.0)
        self.assertEqual(state["motors"]["m3"]["max_angle"], 72.0)
        self.assertEqual(self.manager.motors["m3"].min_angle, 0.0)
        self.assertEqual(self.manager.motors["m3"].max_angle, 180.0)

        self.manager.set_target_angles({"m1": 0.0, "m2": 0.0, "m3": 3.0, "m4": 90.0})
        state = self.manager.get_state()
        self.assertEqual(state["motors"]["m3"]["current_angle"], 3.0)
        self.assertEqual(self.manager.motors["m3"].current_angle, 111.0)

    def test_set_current_angle_zero_replaces_existing_offset(self) -> None:
        self.manager.set_target_angles({"m1": 0.0, "m2": 0.0, "m3": 108.0, "m4": 90.0})
        self.manager.set_current_angle_zero("m3")
        self.manager.set_target_angles({"m1": 0.0, "m2": 0.0, "m3": 3.0, "m4": 90.0})
        self.manager.set_current_angle_zero("m3")

        state = self.manager.get_state()
        self.assertEqual(state["motors"]["m3"]["current_angle"], 0.0)
        self.assertEqual(state["motors"]["m3"]["target_angle"], 0.0)
        self.assertEqual(state["motors"]["m3"]["min_angle"], -111.0)
        self.assertEqual(state["motors"]["m3"]["max_angle"], 69.0)

    def test_set_current_angle_zero_for_all_resets_all_logical_angles(self) -> None:
        self.manager.set_target_angles({"m1": 4.5, "m2": -4.5, "m3": 108.0, "m4": 72.0})
        self.manager.set_current_angle_zero()

        state = self.manager.get_state()
        for motor_state in state["motors"].values():
            self.assertEqual(motor_state["current_angle"], 0.0)
            self.assertEqual(motor_state["target_angle"], 0.0)

    def test_set_current_angle_zero_rejects_unknown_motor(self) -> None:
        with self.assertRaises(ValueError):
            self.manager.set_current_angle_zero("unknown")


class FakeGPIO:
    BCM = "BCM"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    def setwarnings(self, value: bool) -> None:
        pass

    def setmode(self, mode: str) -> None:
        pass

    def setup(self, pin: int, mode: str) -> None:
        pass

    def output(self, pin: int, value: int) -> None:
        pass

    def cleanup(self, pin: int) -> None:
        pass


class StepperMotorTest(unittest.TestCase):
    def test_stepper_target_angle_comes_from_integer_steps(self) -> None:
        fake_gpio = FakeGPIO()

        with patch("motor_control.motors._gpio", return_value=fake_gpio):
            motor = StepperMotor(
                "m1",
                step_pin=16,
                dir_pin=20,
                min_angle=-15.0,
                max_angle=15.0,
                steps_per_rev=36,
                microsteps=1,
                steps_per_second=1.0,
            )

            try:
                motor.move_to(5.0)
                state = motor.snapshot()

                self.assertEqual(state.target_angle, 10.0)
                self.assertLessEqual(state.target_angle, motor.max_angle)
            finally:
                motor.stop_now()
                motor.cleanup()

    def test_stepper_quantized_target_stays_inside_limits(self) -> None:
        fake_gpio = FakeGPIO()

        with patch("motor_control.motors._gpio", return_value=fake_gpio):
            motor = StepperMotor(
                "m1",
                step_pin=16,
                dir_pin=20,
                min_angle=-15.0,
                max_angle=15.0,
                steps_per_rev=36,
                microsteps=1,
                steps_per_second=1.0,
            )

            try:
                motor.move_to(15.0)
                state = motor.snapshot()

                self.assertEqual(state.target_angle, 10.0)
                self.assertLessEqual(state.target_angle, motor.max_angle)
            finally:
                motor.stop_now()
                motor.cleanup()

    def test_move_to_returns_while_worker_is_still_stepping(self) -> None:
        fake_gpio = FakeGPIO()

        with patch("motor_control.motors._gpio", return_value=fake_gpio):
            motor = StepperMotor(
                "m1",
                step_pin=16,
                dir_pin=20,
                steps_per_rev=200,
                microsteps=16,
                steps_per_second=1.0,
            )
            pulse_started = Event()
            release_pulse = Event()

            def blocking_pulse(direction: int) -> None:
                pulse_started.set()
                release_pulse.wait(timeout=1.0)

            motor._pulse = blocking_pulse  # type: ignore[method-assign]

            try:
                motor.move_to(180.0)

                self.assertTrue(pulse_started.wait(timeout=0.2))
                state = motor.snapshot()
                self.assertTrue(state.moving)
                self.assertEqual(state.target_angle, 180.0)
                self.assertNotEqual(state.current_angle, 180.0)
            finally:
                motor.stop_now()
                release_pulse.set()
                motor.cleanup()


if __name__ == "__main__":
    unittest.main()
