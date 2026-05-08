import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages"))

from motor_control import MotorManager
from motor_control.config import build_default_motor_configs, build_m1_m4_hardware_configs
from motor_control.motors import ServoMotor
from motor_control.models import MotorConfig, MotorKind


class MotorManagerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manager = MotorManager()
        self.targets = {
            "m1": 10.0,
            "m2": -10.0,
            "m3": 45.0,
            "m4": 135.0,
        }

    def tearDown(self) -> None:
        self.manager.stop()

    def wait_until(self, condition, timeout: float = 1.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(0.01)
        self.fail("condition was not met before timeout")

    def test_loads_four_m1_m4_motors(self) -> None:
        state = self.manager.get_state()

        self.assertEqual(len(state.motors), 4)
        self.assertEqual(set(state.motors), {"m1", "m2", "m3", "m4"})
        self.assertEqual(state.motors["m1"].kind, MotorKind.STEPPER)
        self.assertEqual(state.motors["m2"].kind, MotorKind.STEPPER)
        self.assertEqual(state.motors["m3"].kind, MotorKind.SERVO)
        self.assertEqual(state.motors["m4"].kind, MotorKind.SERVO)

    def test_hardware_config_keeps_adjustable_parameters(self) -> None:
        configs = build_m1_m4_hardware_configs(
            stepper_options={
                "m1": {"step_pin": 17, "dir_pin": 27, "enable_pin": 22},
                "m2": {"step_pin": 5, "dir_pin": 6, "enable_pin": 13},
            },
            servo_options={
                "m3": {"signal_pin": 18, "min_pulse_us": 600.0},
                "m4": {"signal_pin": 19, "max_pulse_us": 2400.0},
            },
        )

        self.assertEqual([config.id for config in configs], ["m1", "m2", "m3", "m4"])
        self.assertEqual(configs[0].driver_type, "nema17_step_dir")
        self.assertEqual(configs[2].driver_type, "mg995_servo")
        self.assertEqual(configs[0].driver_options["step_pin"], 17)
        self.assertEqual(configs[2].driver_options["min_pulse_us"], 600.0)
        self.assertEqual(configs[3].driver_options["max_pulse_us"], 2400.0)
        self.assertEqual(configs[3].driver_options["min_pulse_us"], 500.0)

    def test_default_profile_is_mock_four_motor_config(self) -> None:
        with patch.dict("os.environ", {}, clear=True):
            configs = build_default_motor_configs()

        self.assertEqual([config.id for config in configs], ["m1", "m2", "m3", "m4"])
        self.assertEqual(configs[0].driver_type, "mock_stepper")
        self.assertEqual(configs[2].driver_type, "mock_servo")

    def test_rejects_incomplete_targets(self) -> None:
        targets = dict(self.targets)
        targets.pop("m4")

        with self.assertRaises(ValueError):
            self.manager.set_target(targets, duration_ms=500)

    def test_rejects_unknown_targets(self) -> None:
        targets = dict(self.targets)
        targets["unknown"] = 1.0

        with self.assertRaises(ValueError):
            self.manager.set_target(targets, duration_ms=500)

    def test_rejects_out_of_range_angle(self) -> None:
        targets = dict(self.targets)
        targets["m3"] = 999.0

        with self.assertRaises(ValueError):
            self.manager.set_target(targets, duration_ms=500)

    def test_requires_homing_before_stepper_target(self) -> None:
        with self.assertRaises(RuntimeError):
            self.manager.set_target(self.targets, duration_ms=500)

    def test_home_all_then_set_target_updates_state(self) -> None:
        self.manager.home_all()
        self.manager.start()

        self.manager.set_target(self.targets, duration_ms=500)
        self.wait_until(lambda: self.manager.get_state().motors["m4"].current_angle == 135.0)

        state = self.manager.get_state()
        self.assertEqual(state.motors["m1"].current_angle, 10.0)
        self.assertEqual(state.motors["m2"].current_angle, -10.0)
        self.assertEqual(state.motors["m3"].current_angle, 45.0)
        self.assertEqual(state.motors["m4"].current_angle, 135.0)
        self.assertIn("UPDATE_SERVO", state.motors["m4"].last_command or "")

    def test_new_target_overwrites_previous_command(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=500)

        next_targets = {"m1": 1.0, "m2": 2.0, "m3": 3.0, "m4": 4.0}
        self.manager.set_target(next_targets, duration_ms=250)
        self.wait_until(lambda: self.manager.get_state().motors["m4"].current_angle == 4.0)

        state = self.manager.get_state()
        self.assertEqual(state.motors["m1"].target_angle, 1.0)
        self.assertEqual(state.motors["m4"].target_angle, 4.0)
        self.assertIn("250", state.motors["m4"].last_command or "")

    def test_set_target_does_not_execute_before_start(self) -> None:
        self.manager.home_all()

        self.manager.set_target(self.targets, duration_ms=500)
        state = self.manager.get_state()

        self.assertEqual(state.motors["m4"].current_angle, 90.0)

    def test_latest_target_before_start_overwrites_previous_target(self) -> None:
        self.manager.home_all()
        next_targets = {"m1": 1.0, "m2": 2.0, "m3": 3.0, "m4": 4.0}

        self.manager.set_target(self.targets, duration_ms=500)
        self.manager.set_target(next_targets, duration_ms=250)
        self.manager.start()
        self.wait_until(lambda: self.manager.get_state().motors["m4"].current_angle == 4.0)

        state = self.manager.get_state()
        self.assertEqual(state.motors["m4"].target_angle, 4.0)
        self.assertNotEqual(state.motors["m4"].target_angle, 135.0)

    def test_latest_target_updates_while_control_loop_is_running(self) -> None:
        self.manager.home_all()
        self.manager.start()
        first_targets = {"m1": 50.0, "m2": 40.0, "m3": 30.0, "m4": 20.0}
        latest_targets = {"m1": -5.0, "m2": -4.0, "m3": 3.0, "m4": 2.0}

        self.manager.set_target(first_targets, duration_ms=1000)
        time.sleep(0.05)
        self.manager.set_target(latest_targets, duration_ms=100)
        self.wait_until(
            lambda: self.manager.get_state().motors["m4"].current_angle == 2.0,
            timeout=1.5,
        )

        state = self.manager.get_state()
        self.assertEqual(state.motors["m1"].target_angle, -5.0)
        self.assertEqual(state.motors["m4"].target_angle, 2.0)
        self.assertIn("100", state.motors["m4"].last_command or "")

    def test_repeated_set_target_and_get_state_while_running(self) -> None:
        self.manager.home_all()
        self.manager.start()

        for index in range(20):
            angle = float(index % 5)
            self.manager.set_target(
                {"m1": angle, "m2": angle, "m3": angle, "m4": angle},
                duration_ms=50,
            )
            state = self.manager.get_state()
            self.assertEqual(len(state.motors), 4)

    def test_emergency_stop_stops_manager(self) -> None:
        self.manager.start()

        self.manager.emergency_stop()
        state = self.manager.get_state()

        self.assertFalse(state.running)

    def test_set_current_angle_zero_updates_one_motor_state(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=100)
        self.wait_until(lambda: self.manager.get_state().motors["m3"].current_angle == 45.0)

        self.manager.set_current_angle_zero("m3")

        state = self.manager.get_state()
        self.assertEqual(state.motors["m3"].current_angle, 0.0)
        self.assertEqual(state.motors["m3"].target_angle, 0.0)
        self.assertEqual(state.motors["m4"].current_angle, 135.0)

    def test_set_current_angle_zero_redefines_future_absolute_angles(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=100)
        self.wait_until(lambda: self.manager.get_state().motors["m3"].current_angle == 45.0)

        self.manager.set_current_angle_zero("m3")
        next_targets = {"m1": 10.0, "m2": -10.0, "m3": 3.0, "m4": 135.0}
        self.manager.set_target(next_targets, duration_ms=100)
        self.wait_until(lambda: self.manager.get_state().motors["m3"].current_angle == 3.0)

        state = self.manager.get_state()
        self.assertEqual(state.motors["m3"].current_angle, 3.0)
        self.assertEqual(state.motors["m3"].target_angle, 3.0)

    def test_set_current_angle_zero_updates_all_motor_states(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=100)
        self.wait_until(lambda: self.manager.get_state().motors["m4"].current_angle == 135.0)

        self.manager.set_current_angle_zero()

        state = self.manager.get_state()
        self.assertTrue(
            all(motor.current_angle == 0.0 for motor in state.motors.values())
        )
        self.assertTrue(
            all(motor.target_angle == 0.0 for motor in state.motors.values())
        )

    def test_set_current_angle_zero_resets_stepper_steps(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=100)
        self.wait_until(lambda: self.manager.get_state().motors["m1"].current_angle == 10.0)

        stepper = self.manager._motors["m1"]
        self.manager.set_current_angle_zero("m1")

        self.assertEqual(self.manager.get_state().motors["m1"].current_angle, 0.0)
        self.assertEqual(stepper.current_steps, 0)

    def test_set_current_angle_zero_rejects_unknown_motor(self) -> None:
        with self.assertRaises(ValueError):
            self.manager.set_current_angle_zero("unknown")

    def test_servo_motor_converts_angle_to_duty_cycle(self) -> None:
        servo = ServoMotor(
            MotorConfig(
                id="m3",
                kind=MotorKind.SERVO,
                min_angle=0.0,
                max_angle=180.0,
                driver_type="mg995_servo",
                driver_options={"signal_pin": 18},
            )
        )

        self.assertAlmostEqual(servo._angle_to_duty_cycle(0.0), 2.5)
        self.assertAlmostEqual(servo._angle_to_duty_cycle(90.0), 7.5)
        self.assertAlmostEqual(servo._angle_to_duty_cycle(180.0), 12.5)

    def test_tick_timing_hook_records_tick_duration(self) -> None:
        tick_ms: list[float] = []
        manager = MotorManager(tick_timing_hook=lambda ms: tick_ms.append(ms))
        try:
            manager.home_all()
            manager.start()
            manager.set_target(self.targets, duration_ms=50)
            self.wait_until(lambda: len(tick_ms) >= 1)
            self.assertTrue(all(ms >= 0.0 for ms in tick_ms))
        finally:
            manager.stop()

    def test_mock_stepper_leaves_hw_angle_unset(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=50)
        self.wait_until(lambda: self.manager.get_state().motors["m1"].current_angle == 10.0)

        state = self.manager.get_state()
        self.assertIsNone(state.motors["m1"].hw_angle)


if __name__ == "__main__":
    unittest.main()
