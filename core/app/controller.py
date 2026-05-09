import asyncio
from dataclasses import dataclass
import logging

from core.app.kinematics_helper import ArmIKConfig, ArmIKHelper, JointLimit, clamp
from core.app.lab_escape_controller import (
    LabEscapeController,
    LabEscapeControllerConfig,
    XYZPosition,
    YZPosition,
)
from core.app.models import Hand, RobotState

logger = logging.getLogger(__name__)

CONTROL_INTERVAL_SECONDS = 0.1
REACHABLE_ERROR_THRESHOLD_CM = 2.0

JOINT_TO_MOTOR_ID = {
    "J1": "m1",
    "J2": "m2",
    "J3": "m3",
    "J4": "m4",
}
MOTOR_ID_TO_JOINT = {motor_id: joint for joint, motor_id in JOINT_TO_MOTOR_ID.items()}

JOINT_MODEL_CALIBRATION = {
    "J1": {"sign": 1.0, "zero_deg": 0.0},
    "J2": {"sign": -1.0, "zero_deg": 90.0},
    "J3": {"sign": 1.0, "zero_deg": 90.0},
    "J4": {"sign": 1.0, "zero_deg": 90.0},
}


@dataclass(frozen=True)
class AffineAxisCalibration:
    scale: float
    offset: float
    min_value: float
    max_value: float

    @classmethod
    def from_points(
        cls,
        *,
        input_min: float,
        input_max: float,
        output_min: float,
        output_max: float,
    ) -> "AffineAxisCalibration":
        scale = (output_max - output_min) / (input_max - input_min)
        offset = output_min - input_min * scale
        return cls(
            scale=scale,
            offset=offset,
            min_value=min(output_min, output_max),
            max_value=max(output_min, output_max),
        )

    def apply(self, value: float) -> float:
        calibrated = float(value) * self.scale + self.offset
        return clamp(calibrated, self.min_value, self.max_value)


@dataclass(frozen=True)
class CameraCalibrationConfig:
    y_from_pixel_x: AffineAxisCalibration
    z_from_pixel_y: AffineAxisCalibration
    min_hand_confidence: float = 0.2

    @classmethod
    def from_lab_config(
        cls,
        config: LabEscapeControllerConfig,
    ) -> "CameraCalibrationConfig":
        return cls(
            y_from_pixel_x=AffineAxisCalibration.from_points(
                input_min=0.0,
                input_max=config.vision_width_px,
                output_min=config.target_y_min_cm,
                output_max=config.target_y_max_cm,
            ),
            z_from_pixel_y=AffineAxisCalibration.from_points(
                input_min=0.0,
                input_max=config.vision_height_px,
                output_min=config.target_z_max_cm,
                output_max=config.target_z_min_cm,
            ),
        )


def build_default_ik_helper() -> ArmIKHelper:
    return ArmIKHelper(
        build_ik_config(
            {
                "m1": {"min_angle": -45.0, "max_angle": 45.0},
                "m2": {"min_angle": 45.0, "max_angle": 135.0},
                "m3": {"min_angle": 45.0, "max_angle": 135.0},
                "m4": {"min_angle": 45.0, "max_angle": 135.0},
            }
        )
    )


def build_ik_config(motors: dict) -> ArmIKConfig:
    return ArmIKConfig(
        max_extra_pitch_deg=12.0,
        max_extra_yaw_deg=10.0,
        joint_limits={
            joint: _joint_limit_from_motor_state(joint, motors[motor_id])
            for joint, motor_id in JOINT_TO_MOTOR_ID.items()
            if motor_id in motors
        },
        max_iterations=80,
        stop_error_cm=0.05,
        step_scale=0.4,
    )


def _joint_limit_from_motor_state(joint: str, motor_state: dict) -> JointLimit:
    calibration = JOINT_MODEL_CALIBRATION[joint]
    return JointLimit(
        min_deg=float(motor_state["min_angle"]),
        max_deg=float(motor_state["max_angle"]),
        sign=float(calibration["sign"]),
        zero_deg=float(calibration["zero_deg"]),
    )


class RobotController:
    def __init__(self, app):
        self.app = app
        escape_config = LabEscapeControllerConfig()
        self.escape_controller = LabEscapeController(escape_config)
        self.camera_calibration = CameraCalibrationConfig.from_lab_config(
            escape_config,
        )
        self.ik = build_default_ik_helper()
        self._was_started = False

    async def run(self) -> None:
        while True:
            if self.app.state.robot_state is not RobotState.started:
                if self._was_started:
                    self.escape_controller.reset()
                    self.ik.reset()
                    self._was_started = False
                await asyncio.sleep(CONTROL_INTERVAL_SECONDS)
                continue

            try:
                self._was_started = True
                self.step()
            except Exception:
                logger.exception("Robot controller failed")
                self.app.state.running = False
                self.app.state.robot_state = RobotState.error

            await asyncio.sleep(CONTROL_INTERVAL_SECONDS)

    def step(self) -> None:
        hand = self.app.state.hand
        motor_manager = self.app.state.motor_manager

        motor_state = motor_manager.get_state()

        targets = self.compute_targets(
            hand=hand,
            motor_state=motor_state,
        )

        motor_manager.set_target(targets)

    def compute_targets(self, *, hand: Hand, motor_state: dict) -> dict[str, float]:
        self._sync_ik_limits(motor_state)
        current_targets = self._current_motor_targets(motor_state)
        current_xyz = self._current_xyz(motor_state)
        hand_robot_yz = self._calibrate_hand_yz(hand)

        result = self.escape_controller.compute_next_xyz(
            hand_robot_yz=hand_robot_yz,
            current_xyz_cm=current_xyz,
            dt=float(motor_state.get("dt_s", CONTROL_INTERVAL_SECONDS)),
        )

        try:
            ik_result = self.ik.solve_xyz(
                target_x_cm=result.next_xyz_cm.x_cm,
                target_y_cm=result.next_xyz_cm.y_cm,
                target_z_cm=result.next_xyz_cm.z_cm,
                reachable_error_threshold_cm=REACHABLE_ERROR_THRESHOLD_CM,
            )
        except Exception as exc:
            logger.warning("IK failed; holding current motor targets: %s", exc)
            self.ik.reset()
            logger.debug(
                "escape controller held targets: state=%s xyz=(%.2f, %.2f, %.2f) status=%s error=%s",
                result.lab_state,
                result.next_xyz_cm.x_cm,
                result.next_xyz_cm.y_cm,
                result.next_xyz_cm.z_cm,
                result.status,
                exc,
            )
            return current_targets

        if not ik_result.reachable:
            logger.warning(
                "IK target unreachable; holding current motor targets: target=(%.2f, %.2f, %.2f) err=%.2f status=%s",
                result.next_xyz_cm.x_cm,
                result.next_xyz_cm.y_cm,
                result.next_xyz_cm.z_cm,
                ik_result.error_cm,
                result.status,
            )
            return current_targets

        targets = self._joint_targets_to_motor_targets(
            ik_result.motor_targets(),
            base_targets=current_targets,
        )
        logger.debug(
            "escape controller: state=%s xyz=(%.2f, %.2f, %.2f) reachable=%s err=%.2f status=%s",
            result.lab_state,
            result.next_xyz_cm.x_cm,
            result.next_xyz_cm.y_cm,
            result.next_xyz_cm.z_cm,
            ik_result.reachable,
            ik_result.error_cm,
            result.status,
        )
        return targets

    def _calibrate_hand_yz(self, hand: Hand) -> YZPosition | None:
        if not hand.detected or hand.x is None or hand.y is None:
            return None

        confidence = hand.confidence
        if (
            confidence is not None
            and confidence < self.camera_calibration.min_hand_confidence
        ):
            return None

        return YZPosition(
            y_cm=self.camera_calibration.y_from_pixel_x.apply(float(hand.x)),
            z_cm=self.camera_calibration.z_from_pixel_y.apply(float(hand.y)),
        )

    def _sync_ik_limits(self, motor_state: dict) -> None:
        motors = motor_state.get("motors", {})
        if not all(
            motor_id in motors
            and "min_angle" in motors[motor_id]
            and "max_angle" in motors[motor_id]
            for motor_id in JOINT_TO_MOTOR_ID.values()
        ):
            return

        next_config = build_ik_config(motors)
        current_config = getattr(self.ik, "config", None)
        if (
            current_config is not None
            and next_config.joint_limits == current_config.joint_limits
        ):
            return

        self.ik = ArmIKHelper(next_config)

    def _current_xyz(self, motor_state: dict) -> XYZPosition:
        x_cm, y_cm, z_cm = self.ik.end_xyz_from_motor_angles(
            self._current_joint_angles(motor_state)
        )
        return XYZPosition(x_cm=x_cm, y_cm=y_cm, z_cm=z_cm)

    def _current_joint_angles(self, motor_state: dict) -> dict[str, float]:
        motors = motor_state.get("motors", {})
        joint_angles: dict[str, float] = {}
        for motor_id, joint in MOTOR_ID_TO_JOINT.items():
            state = motors.get(motor_id, {})
            joint_angles[joint] = float(
                state.get("current_angle", state.get("target_angle", 0.0))
            )
        return joint_angles

    def _joint_targets_to_motor_targets(
        self,
        joint_targets: dict[str, float],
        *,
        base_targets: dict[str, float],
    ) -> dict[str, float]:
        targets = dict(base_targets)
        for joint, target in joint_targets.items():
            motor_id = JOINT_TO_MOTOR_ID.get(joint)
            if motor_id is not None and motor_id in targets:
                targets[motor_id] = float(target)
        return targets

    @staticmethod
    def _current_motor_targets(motor_state: dict) -> dict[str, float]:
        motors = motor_state.get("motors", {})
        return {
            motor_id: float(state.get("target_angle", state.get("current_angle", 0.0)))
            for motor_id, state in motors.items()
        }
