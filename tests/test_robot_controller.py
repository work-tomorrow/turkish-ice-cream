import inspect
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.app import escape_controller
from core.app.config import Settings
from core.app.escape_controller import (
    EscapeController,
    EscapeStep,
    XYZPosition,
    YZPosition,
)
from core.app.kinematics_helper import ArmIKConfig, ArmIKHelper, JointLimit
from core.app.lab_escape import algorithms, machine
from core.app.lab_escape.common import LabEscapeConfig
from core.app.models import Hand
from core.app.robot_controller import (
    RobotController,
    ik_config_from_motors,
    motion_cycles,
)


class RobotControllerTest(unittest.TestCase):
    def test_vanish_motor_targets_default_and_env_override(self) -> None:
        with patch.dict("os.environ", {}, clear=True):
            self.assertEqual(
                Settings().vanish_motor_targets,
                {"m1": 0.0, "m2": 90.0, "m3": 90.0, "m4": 90.0},
            )

        with patch.dict(
            "os.environ",
            {
                "VANISH_M1_DEG": "11",
                "VANISH_M2_DEG": "22",
                "VANISH_M3_DEG": "33",
                "VANISH_M4_DEG": "44",
            },
        ):
            self.assertEqual(
                Settings().vanish_motor_targets,
                {"m1": 11.0, "m2": 22.0, "m3": 33.0, "m4": 44.0},
            )

    def test_escape_controller_does_not_own_ik_or_motor_targets(self) -> None:
        source = inspect.getsource(EscapeController)

        self.assertNotIn("ArmIKHelper", source)
        self.assertNotIn("motor_state", source)
        self.assertNotIn("next_targets", source)
        self.assertNotIn("sys.path", source)
        self.assertEqual(
            EscapeController.__init__.__globals__["EscapeStateMachine"].__module__,
            "core.app.lab_escape.machine",
        )

        result_fields = set(EscapeStep.__dataclass_fields__)
        self.assertIn("next_xyz_cm", result_fields)
        self.assertIn("visible", result_fields)
        self.assertIn("planned_landing_xyz_cm", result_fields)
        self.assertNotIn("targets", result_fields)

    def test_clamp_robot_yz_matches_rectangular_workspace_only(self) -> None:
        """Corners inside the rectangle must not be pulled toward the origin."""
        cfg = LabEscapeConfig()
        ctl = EscapeController(cfg)
        corner = YZPosition(y_cm=cfg.target_y_max_cm, z_cm=cfg.target_z_max_cm)
        clamped = ctl._clamp_robot_yz(corner)
        self.assertAlmostEqual(clamped.y_cm, corner.y_cm)
        self.assertAlmostEqual(clamped.z_cm, corner.z_cm)

    def test_lab_escape_workspace_z_and_y_spans_match_rectangle_size(self) -> None:
        """Origin can split width and height independently."""
        cfg = LabEscapeConfig(
            boundary_size_cm=25.0,
            boundary_height_cm=12.0,
            origin_to_left_boundary_cm=10.0,
            origin_to_top_boundary_cm=8.0,
        )
        self.assertAlmostEqual(
            cfg.target_y_max_cm - cfg.target_y_min_cm,
            cfg.boundary_size_cm,
        )
        self.assertAlmostEqual(
            cfg.target_z_max_cm - cfg.target_z_min_cm,
            cfg.boundary_height_cm,
        )
        self.assertAlmostEqual(cfg.target_z_min_cm, -cfg.origin_to_bottom_boundary_cm)

    def test_core_lab_escape_imports_are_packaged(self) -> None:
        for module in (
            escape_controller,
            machine,
            algorithms,
        ):
            source = inspect.getsource(module)
            self.assertNotIn("algorithm_lab-kosu", source)
            self.assertNotIn("sys.path", source)
            self.assertNotIn("from common import", source)
            self.assertNotIn("from machine import", source)
            self.assertNotIn("from algorithms import", source)

    def test_affine_calibration_maps_pixels_to_robot_yz_workspace(self) -> None:
        controller = RobotController(SimpleNamespace())
        config = controller.escape.config
        self.assertEqual(config.vision_width_px, 500.0)
        self.assertEqual(config.vision_height_px, 300.0)

        top_left = controller._calibrate_hand_yz(
            Hand(detected=True, x=172.0, y=165.0, confidence=1.0)
        )
        top_right = controller._calibrate_hand_yz(
            Hand(detected=True, x=450.0, y=200.0, confidence=1.0)
        )
        bottom_right = controller._calibrate_hand_yz(
            Hand(detected=True, x=450.0, y=350.0, confidence=1.0)
        )
        bottom_left = controller._calibrate_hand_yz(
            Hand(detected=True, x=160.0, y=350.0, confidence=1.0)
        )

        self.assertIsNotNone(top_left)
        self.assertIsNotNone(top_right)
        self.assertIsNotNone(bottom_right)
        self.assertIsNotNone(bottom_left)
        assert top_left is not None
        assert top_right is not None
        assert bottom_right is not None
        assert bottom_left is not None
        self.assertAlmostEqual(top_left.y_cm, -25.0)
        self.assertAlmostEqual(top_left.z_cm, 25.0)
        self.assertAlmostEqual(top_right.y_cm, 25.0)
        self.assertAlmostEqual(top_right.z_cm, 25.0)
        self.assertAlmostEqual(bottom_right.y_cm, 25.0)
        self.assertAlmostEqual(bottom_right.z_cm, -5.0)
        self.assertAlmostEqual(bottom_left.y_cm, -25.0)
        self.assertAlmostEqual(bottom_left.z_cm, -5.0)

    def test_escape_controller_and_state_machine_share_one_config(self) -> None:
        config = LabEscapeConfig(
            escape_trigger_distance_cm=9.0,
            max_command_step_cm=3.0,
            pixels_per_cm=10.0,
        )
        controller = EscapeController(config)

        self.assertIs(controller.config, config)
        self.assertIs(controller.machine.config, config)
        self.assertEqual(config.vision_width_px, 500.0)
        self.assertEqual(config.target_y_min_cm, -25.0)

    def test_motion_cycles_use_angle_delta_velocity_and_dt(self) -> None:
        self.assertEqual(
            motion_cycles(
                start_targets={"m1": 0.0, "m2": 90.0},
                target_targets={"m1": 30.0, "m2": 100.0},
                max_velocity_deg_s={"m1": 100.0, "m2": 50.0},
                dt_s=0.1,
            ),
            3,
        )

    def test_next_targets_uses_fk_escape_then_ik_and_returns_complete_targets(
        self,
    ) -> None:
        controller = RobotController(SimpleNamespace())
        fake_ik = _FakeIK()
        fake_escape = _FakeEscape()
        controller.ik = fake_ik
        controller.escape = fake_escape

        targets = controller.next_targets(
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

    def test_vanish_lift_returns_configured_pose_for_computed_cycles(self) -> None:
        controller = RobotController(SimpleNamespace())
        controller.ik = _FakeIK()
        controller.escape = _InvisibleEscape()
        controller.vanish_motor_targets = {
            "m1": 30.0,
            "m2": 90.0,
            "m3": 90.0,
            "m4": 90.0,
        }
        motor_state = _motor_state(
            max_velocity_deg_s={
                "m1": 100.0,
                "m2": 1000.0,
                "m3": 1000.0,
                "m4": 1000.0,
            }
        )

        for _ in range(3):
            self.assertEqual(
                controller.next_targets(
                    hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
                    motor_state=motor_state,
                ),
                {"m1": 30.0, "m2": 90.0, "m3": 90.0, "m4": 90.0},
            )

        self.assertEqual(controller._vanish_sequence.phase, "landing")

    def test_vanish_landing_returns_ik_targets_for_computed_cycles(self) -> None:
        controller = RobotController(SimpleNamespace())
        controller.ik = _FakeIK()
        controller.escape = _InvisibleEscape()
        controller.vanish_motor_targets = {
            "m1": 0.0,
            "m2": 90.0,
            "m3": 90.0,
            "m4": 90.0,
        }
        motor_state = _motor_state(
            max_velocity_deg_s={
                "m1": 20.0,
                "m2": 20.0,
                "m3": 20.0,
                "m4": 20.0,
            }
        )

        controller.next_targets(
            hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
            motor_state=motor_state,
        )
        expected_landing_targets = {
            "m1": 1.0,
            "m2": 91.0,
            "m3": 92.0,
            "m4": 93.0,
        }

        self.assertEqual(
            controller.next_targets(
                hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
                motor_state=motor_state,
            ),
            expected_landing_targets,
        )
        self.assertEqual(controller._vanish_sequence.phase, "landing")
        self.assertEqual(
            controller.next_targets(
                hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
                motor_state=motor_state,
            ),
            expected_landing_targets,
        )
        self.assertEqual(controller._vanish_sequence.phase, "idle")

    def test_vanish_landing_ik_failure_holds_current_targets_and_resets(self) -> None:
        controller = RobotController(SimpleNamespace())
        controller.ik = _FailingIK()
        controller.escape = _InvisibleEscape()
        controller.vanish_motor_targets = {
            "m1": 0.0,
            "m2": 90.0,
            "m3": 90.0,
            "m4": 90.0,
        }
        motor_state = _motor_state()

        controller.next_targets(
            hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
            motor_state=motor_state,
        )
        with self.assertLogs("core.app.robot_controller", level="WARNING"):
            targets = controller.next_targets(
                hand=Hand(detected=True, x=320.0, y=240.0, confidence=1.0),
                motor_state=motor_state,
            )

        self.assertEqual(
            targets,
            {"m1": 0.0, "m2": 90.0, "m3": 90.0, "m4": 90.0},
        )
        self.assertEqual(controller._vanish_sequence.phase, "idle")

    def test_next_targets_holds_current_targets_when_ik_fails(self) -> None:
        controller = RobotController(SimpleNamespace())
        controller.ik = _FailingIK()
        controller.escape = _FakeEscape()

        with self.assertLogs("core.app.robot_controller", level="WARNING"):
            targets = controller.next_targets(
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

    def test_debug_yz_targets_solves_robot_frame_yz(self) -> None:
        controller = RobotController(SimpleNamespace())
        fake_ik = _FakeIK()
        controller.ik = fake_ik

        targets, error_cm = controller.debug_yz_targets(
            y_cm=6.0,
            z_cm=7.0,
            update_ik_state=True,
            motor_state=_motor_state(),
        )

        self.assertEqual(error_cm, 0.0)
        self.assertEqual(fake_ik.last_solve_target, (0.0, 6.0, 7.0))
        self.assertEqual(
            targets,
            {
                "m1": 1.0,
                "m2": 91.0,
                "m3": 92.0,
                "m4": 93.0,
            },
        )

    def test_debug_yz_targets_returns_none_when_unreachable(self) -> None:
        controller = RobotController(SimpleNamespace())
        controller.ik = _UnreachableIK()

        targets, error_cm = controller.debug_yz_targets(
            y_cm=6.0,
            z_cm=7.0,
            update_ik_state=False,
            motor_state=_motor_state(),
        )

        self.assertIsNone(targets)
        self.assertEqual(error_cm, 4.0)

    def test_ik_limits_are_derived_from_motor_manager_state(self) -> None:
        config = ik_config_from_motors(
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

    def step(self, *, hand_robot_yz, current_xyz_cm, dt):
        self.last_current_xyz = current_xyz_cm
        return SimpleNamespace(
            next_xyz_cm=XYZPosition(x_cm=current_xyz_cm.x_cm, y_cm=4.0, z_cm=5.0),
            escape_state="NORMAL_ESCAPE",
            status="fake",
            visible=True,
            planned_landing_xyz_cm=None,
        )


class _InvisibleEscape:
    def __init__(self) -> None:
        self.landing_xyz = XYZPosition(x_cm=100.0, y_cm=6.0, z_cm=7.0)

    def step(self, *, hand_robot_yz, current_xyz_cm, dt):
        return SimpleNamespace(
            next_xyz_cm=current_xyz_cm,
            escape_state="BOUNDARY_ESCAPE",
            status="vanished",
            visible=False,
            planned_landing_xyz_cm=self.landing_xyz,
        )


class _FakeIK:
    def __init__(self) -> None:
        self.config = ik_config_from_motors(_motor_state()["motors"])
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


class _UnreachableIK(_FakeIK):
    def solve_xyz(self, **kwargs):
        self.last_solve_target = (
            kwargs["target_x_cm"],
            kwargs["target_y_cm"],
            kwargs["target_z_cm"],
        )
        return _UnreachableIKResult()


class _FakeIKResult:
    reachable = True
    error_cm = 0.0

    def motor_targets(self):
        return {"J1": 1.0, "J2": 91.0, "J3": 92.0, "J4": 93.0}


class _UnreachableIKResult:
    reachable = False
    error_cm = 4.0

    def motor_targets(self):
        return {}


def _motor_state(max_velocity_deg_s=None):
    max_velocity_deg_s = max_velocity_deg_s or {
        "m1": 3600.0,
        "m2": 3600.0,
        "m3": 3600.0,
        "m4": 3600.0,
    }
    return {
        "dt_s": 0.1,
        "current_targets": {
            "m1": 0.0,
            "m2": 90.0,
            "m3": 90.0,
            "m4": 90.0,
        },
        "max_velocity_deg_s": max_velocity_deg_s,
        "motors": {
            "m1": {
                "current_angle": 0.0,
                "target_angle": 0.0,
                "min_angle": 0,
                "max_angle": 0,
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
