import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages"))

from motor_control import MotorManager


class MotorManagerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manager = MotorManager()
        self.targets = {
            "stepper_1": 10.0,
            "stepper_2": -10.0,
            "tt_1": 20.0,
            "tt_2": -20.0,
            "pm30225": 30.0,
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

    def test_loads_five_mixed_motors(self) -> None:
        state = self.manager.get_state()

        self.assertEqual(len(state.motors), 5)
        self.assertEqual(
            set(state.motors),
            {"stepper_1", "stepper_2", "tt_1", "tt_2", "pm30225"},
        )

    def test_rejects_incomplete_targets(self) -> None:
        targets = dict(self.targets)
        targets.pop("tt_2")

        with self.assertRaises(ValueError):
            self.manager.set_target(targets, duration_ms=500)

    def test_rejects_unknown_targets(self) -> None:
        targets = dict(self.targets)
        targets["unknown"] = 1.0

        with self.assertRaises(ValueError):
            self.manager.set_target(targets, duration_ms=500)

    def test_rejects_out_of_range_angle(self) -> None:
        targets = dict(self.targets)
        targets["tt_1"] = 999.0

        with self.assertRaises(ValueError):
            self.manager.set_target(targets, duration_ms=500)

    def test_requires_homing_before_stepper_target(self) -> None:
        with self.assertRaises(RuntimeError):
            self.manager.set_target(self.targets, duration_ms=500)

    def test_home_all_then_set_target_updates_state(self) -> None:
        self.manager.home_all()
        self.manager.start()

        self.manager.set_target(self.targets, duration_ms=500)
        self.wait_until(
            lambda: self.manager.get_state().motors["pm30225"].current_angle
            == 30.0
        )

        state = self.manager.get_state()
        self.assertEqual(state.motors["stepper_1"].current_angle, 10.0)
        self.assertEqual(state.motors["tt_1"].current_angle, 20.0)
        self.assertEqual(state.motors["pm30225"].current_angle, 30.0)
        self.assertTrue(state.motors["tt_1"].approximate)

    def test_new_target_overwrites_previous_command(self) -> None:
        self.manager.home_all()
        self.manager.start()
        self.manager.set_target(self.targets, duration_ms=500)

        next_targets = {
            "stepper_1": 1.0,
            "stepper_2": 2.0,
            "tt_1": 3.0,
            "tt_2": 4.0,
            "pm30225": 5.0,
        }
        self.manager.set_target(next_targets, duration_ms=250)
        self.wait_until(
            lambda: self.manager.get_state().motors["pm30225"].current_angle
            == 5.0
        )

        state = self.manager.get_state()
        self.assertEqual(state.motors["stepper_1"].target_angle, 1.0)
        self.assertEqual(state.motors["pm30225"].target_angle, 5.0)
        self.assertIn("250", state.motors["pm30225"].last_command or "")

    def test_set_target_does_not_execute_before_start(self) -> None:
        self.manager.home_all()

        self.manager.set_target(self.targets, duration_ms=500)
        state = self.manager.get_state()

        self.assertEqual(state.motors["pm30225"].current_angle, 0.0)

    def test_latest_target_before_start_overwrites_previous_target(self) -> None:
        self.manager.home_all()
        next_targets = {
            "stepper_1": 1.0,
            "stepper_2": 2.0,
            "tt_1": 3.0,
            "tt_2": 4.0,
            "pm30225": 5.0,
        }

        self.manager.set_target(self.targets, duration_ms=500)
        self.manager.set_target(next_targets, duration_ms=250)
        self.manager.start()
        self.wait_until(
            lambda: self.manager.get_state().motors["pm30225"].current_angle
            == 5.0
        )

        state = self.manager.get_state()
        self.assertEqual(state.motors["pm30225"].target_angle, 5.0)
        self.assertNotEqual(state.motors["pm30225"].target_angle, 30.0)

    def test_latest_target_updates_while_control_loop_is_running(self) -> None:
        self.manager.home_all()
        self.manager.start()
        first_targets = {
            "stepper_1": 50.0,
            "stepper_2": 40.0,
            "tt_1": 30.0,
            "tt_2": 20.0,
            "pm30225": 10.0,
        }
        latest_targets = {
            "stepper_1": -5.0,
            "stepper_2": -4.0,
            "tt_1": -3.0,
            "tt_2": -2.0,
            "pm30225": -1.0,
        }

        self.manager.set_target(first_targets, duration_ms=1000)
        time.sleep(0.05)
        self.manager.set_target(latest_targets, duration_ms=100)
        self.wait_until(
            lambda: self.manager.get_state().motors["pm30225"].current_angle
            == -1.0,
            timeout=1.5,
        )

        state = self.manager.get_state()
        self.assertEqual(state.motors["stepper_1"].target_angle, -5.0)
        self.assertEqual(state.motors["pm30225"].target_angle, -1.0)
        self.assertIn("100", state.motors["pm30225"].last_command or "")

    def test_repeated_set_target_and_get_state_while_running(self) -> None:
        self.manager.home_all()
        self.manager.start()

        for index in range(20):
            angle = float(index % 5)
            self.manager.set_target(
                {
                    "stepper_1": angle,
                    "stepper_2": angle,
                    "tt_1": angle,
                    "tt_2": angle,
                    "pm30225": angle,
                },
                duration_ms=50,
            )
            state = self.manager.get_state()
            self.assertEqual(len(state.motors), 5)

    def test_emergency_stop_stops_manager(self) -> None:
        self.manager.start()

        self.manager.emergency_stop()
        state = self.manager.get_state()

        self.assertFalse(state.running)


if __name__ == "__main__":
    unittest.main()
