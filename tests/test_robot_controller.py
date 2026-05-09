import inspect
import unittest
from types import SimpleNamespace

from core.app.controller import RobotController, build_ik_config
from core.app.kinematics_helper import ArmIKConfig, ArmIKHelper, JointLimit
from core.app.lab_escape_controller import (
    LabEscapeController,
    LabEscapeControllerResult,
    XYZPosition,
)
from core.app.models import Hand


class RobotControllerTest(unittest.TestCase):
    def test_lab_escape_controller_does_not_own_ik_or_motor_targets(self) -> None:
        source = inspect.getsource(LabEscapeController)

        self.assertNotIn("ArmIKHelper", source)
        self.assertNotIn("motor_state", source)
        self.assertNotIn("compute_targets", source)
        self.assertNotIn("sys.path", source)
        self.assertEqual(
            LabEscapeController.__init__.__globals__["ArmStateMachine"].__module__,
            "core.app.lab_escape.state_machine",
        )

        result_fields = set(LabEscapeControllerResult.__dataclass_fields__)
        self.assertIn("next_xyz_cm", result_fields)
        self.assertNotIn("targets", result_fields)

    def test_affine_calibration_maps_pixels_to_robot_yz_workspace(self) -> None:
        controller = RobotController(SimpleNamespace())

        top_left = controller._calibrate_hand_yz(
            Hand(detected=True, x=0.0, y=0.0, confidence=1.0)
        )
        bottom_right = controller._calibrate_hand_yz(
            Hand(detected=True, x=640.0, y=480.0, confidence=1.0)
        )

        self.assertIsNotNone(top_left)
        self.assertIsNotNone(bottom_right)
        assert top_left is not None
        assert bottom_right is not None
        self.assertEqual(top_left.y_cm, -18.0)
        self.assertEqual(top_left.z_cm, 14.0)
        self.assertEqual(bottom_right.y_cm, 18.0)
        self.assertEqual(bottom_right.z_cm, -10.0)

    def test_compute_targets_uses_fk_escape_then_ik_and_returns_complete_targets(
        self,
    ) -> None:
        controller = RobotController(SimpleNamespace())
        fake_ik = _FakeIK()
        fake_escape = _FakeEscape()
        controller.ik = fake_ik
        controller.escape_controller = fake_escape

        targets = controller.compute_targets(
            hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
            motor_state=_motor_state(),
        )

        self.assertEqual(set(targets), {"m1", "m2", "m3", "m4"})
        self.assertEqual(
            targets,
            {
                "m1": 1.0,
                "m2": 91.0,
                "m3": 92.0,
                "m4": 93.0,
            },
        )
        self.assertEqual(
            fake_ik.last_fk_angles,
            {"J1": 0.0, "J2": 90.0, "J3": 90.0, "J4": 90.0},
        )
        self.assertEqual(
            fake_escape.last_current_xyz,
            XYZPosition(x_cm=100.0, y_cm=0.0, z_cm=0.0),
        )
        self.assertEqual(fake_ik.last_solve_target, (100.0, 4.0, 5.0))

    def test_compute_targets_holds_current_targets_when_ik_fails(self) -> None:
        controller = RobotController(SimpleNamespace())
        controller.ik = _FailingIK()
        controller.escape_controller = _FakeEscape()

        with self.assertLogs("core.app.controller", level="WARNING"):
            targets = controller.compute_targets(
                hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
                motor_state=_motor_state(),
            )

        self.assertEqual(
            targets,
            {
                "m1": 0.0,
                "m2": 90.0,
                "m3": 90.0,
                "m4": 90.0,
            },
        )

    def test_ik_limits_are_derived_from_motor_manager_state(self) -> None:
        config = build_ik_config(
            {
                "m1": {"min_angle": -10.0, "max_angle": 10.0},
                "m2": {"min_angle": 30.0, "max_angle": 120.0},
                "m3": {"min_angle": 40.0, "max_angle": 130.0},
                "m4": {"min_angle": 50.0, "max_angle": 140.0},
            }
        )

        self.assertEqual(config.joint_limits["J1"].min_deg, -10.0)
        self.assertEqual(config.joint_limits["J1"].max_deg, 10.0)
        self.assertEqual(config.joint_limits["J2"].min_deg, 30.0)
        self.assertEqual(config.joint_limits["J2"].max_deg, 120.0)
        self.assertEqual(config.joint_limits["J2"].sign, -1.0)
        self.assertEqual(config.joint_limits["J2"].zero_deg, 90.0)


class KinematicsHelperTest(unittest.TestCase):
    def test_motor_angles_can_round_trip_to_model_angles_and_fk_xyz(self) -> None:
        helper = ArmIKHelper(
            ArmIKConfig(
                joint_limits={
                    "J1": JointLimit(-180.0, 180.0),
                    "J2": JointLimit(-180.0, 180.0),
                    "J3": JointLimit(-180.0, 180.0),
                    "J4": JointLimit(-180.0, 180.0),
                }
            )
        )

        model_angles = helper.motor_to_model_angles(
            {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}
        )
        xyz = helper.end_xyz_from_motor_angles(
            {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}
        )

        self.assertEqual(
            model_angles,
            {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0},
        )
        self.assertEqual(len(xyz), 3)
        for value in xyz:
            self.assertIsInstance(value, float)


class _FakeEscape:
    def __init__(self) -> None:
        self.last_current_xyz = None

    def compute_next_xyz(self, *, hand_robot_yz, current_xyz_cm, dt):
        self.last_current_xyz = current_xyz_cm
        return SimpleNamespace(
            next_xyz_cm=XYZPosition(x_cm=current_xyz_cm.x_cm, y_cm=4.0, z_cm=5.0),
            lab_state="NORMAL_ESCAPE",
            status="fake",
        )


class _FakeIK:
    def __init__(self) -> None:
        self.config = build_ik_config(_motor_state()["motors"])
        self.last_fk_angles = None
        self.last_solve_target = None

    def end_xyz_from_motor_angles(self, angles):
        self.last_fk_angles = dict(angles)
        return (100.0, 0.0, 0.0)

    def solve_xyz(self, *, target_x_cm, target_y_cm, target_z_cm, **kwargs):
        self.last_solve_target = (target_x_cm, target_y_cm, target_z_cm)
        return _FakeIKResult()


class _FailingIK(_FakeIK):
    def solve_xyz(self, **kwargs):
        raise RuntimeError("boom")

    def reset(self):
        pass


class _FakeIKResult:
    reachable = True
    error_cm = 0.0

    def motor_targets(self):
        return {"J1": 1.0, "J2": 91.0, "J3": 92.0, "J4": 93.0}


def _motor_state():
    return {
        "dt_s": 0.1,
        "motors": {
            "m1": {
                "current_angle": 0.0,
                "target_angle": 0.0,
                "min_angle": -45.0,
                "max_angle": 45.0,
            },
            "m2": {
                "current_angle": 90.0,
                "target_angle": 90.0,
                "min_angle": 45.0,
                "max_angle": 135.0,
            },
            "m3": {
                "current_angle": 90.0,
                "target_angle": 90.0,
                "min_angle": 45.0,
                "max_angle": 135.0,
            },
            "m4": {
                "current_angle": 90.0,
                "target_angle": 90.0,
                "min_angle": 45.0,
                "max_angle": 135.0,
            },
        },
    }


if __name__ == "__main__":
    unittest.main()
