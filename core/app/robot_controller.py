import asyncio
from dataclasses import dataclass
import logging
import math

import numpy as np

from core.app.config import settings
from core.app.escape_controller import EscapeController, XYZPosition, YZPosition
from core.app.kinematics_helper import ArmIKConfig, ArmIKHelper, JointLimit
from core.app.lab_escape.common import LabEscapeConfig
from core.app.models import Hand, RobotState

logger = logging.getLogger(__name__)

CONTROL_INTERVAL_SECONDS = 0.1
REACHABLE_ERROR_THRESHOLD_CM = 10.0

JOINT_TO_MOTOR_ID = {
    "J1": "m1",
    "J2": "m2",
    "J3": "m3",
    "J4": "m4",
}
MOTOR_ID_TO_JOINT = {motor_id: joint for joint, motor_id in JOINT_TO_MOTOR_ID.items()}

JOINT_MODEL_CALIBRATION = {
    "J1": {"sign": 1.0, "zero_deg": 0.0},
    "J2": {"sign": 1.0, "zero_deg": 0.0},
    "J3": {"sign": 1.0, "zero_deg": 0.0},
    "J4": {"sign": -1.0, "zero_deg": 0.0},
}


VANISH_PHASE_IDLE = "idle"
VANISH_PHASE_LIFTING = "lifting"
VANISH_PHASE_LANDING = "landing"


@dataclass(frozen=True)
class CameraCalibrationConfig:
    image_to_robot_yz: np.ndarray
    min_hand_confidence: float = 0.2

    @classmethod
    def from_escape_config(
        cls,
        config: LabEscapeConfig,
    ) -> "CameraCalibrationConfig":
        robot_workspace_points = (
            (config.target_y_min_cm, config.target_z_max_cm),
            (config.target_y_max_cm, config.target_z_max_cm),
            (config.target_y_max_cm, config.target_z_min_cm),
            (config.target_y_min_cm, config.target_z_min_cm),
        )
        return cls(
            image_to_robot_yz=_homography_from_points(
                config.image_workspace_points_px,
                robot_workspace_points,
            ),
            min_hand_confidence=config.min_hand_confidence,
        )

    def apply(self, x_px: float, y_px: float) -> YZPosition:
        point = np.array([float(x_px), float(y_px), 1.0], dtype=float)
        mapped = self.image_to_robot_yz @ point
        scale = mapped[2]
        if abs(scale) < 1e-9:
            raise ValueError("Camera calibration produced an invalid point scale")

        return YZPosition(
            y_cm=float(mapped[0] / scale),
            z_cm=float(mapped[1] / scale),
        )


def _homography_from_points(
    image_points: tuple[tuple[float, float], ...],
    robot_points: tuple[tuple[float, float], ...],
) -> np.ndarray:
    if len(image_points) != 4 or len(robot_points) != 4:
        raise ValueError("Camera calibration needs exactly four image and robot points")

    rows = []
    values = []
    for (x, y), (target_y, target_z) in zip(image_points, robot_points):
        rows.append([x, y, 1.0, 0.0, 0.0, 0.0, -target_y * x, -target_y * y])
        values.append(target_y)
        rows.append([0.0, 0.0, 0.0, x, y, 1.0, -target_z * x, -target_z * y])
        values.append(target_z)

    solved = np.linalg.solve(
        np.array(rows, dtype=float),
        np.array(values, dtype=float),
    )
    h11, h12, h13, h21, h22, h23, h31, h32 = solved
    return np.array(
        [
            [h11, h12, h13],
            [h21, h22, h23],
            [h31, h32, 1.0],
        ],
        dtype=float,
    )


@dataclass
class VanishSequence:
    phase: str = VANISH_PHASE_IDLE
    landing_xyz_cm: XYZPosition | None = None
    lift_cycles_remaining: int = 0
    land_cycles_remaining: int = 0
    landing_targets: dict[str, float] | None = None


def motion_cycles(
    *,
    start_targets: dict[str, float],
    target_targets: dict[str, float],
    max_velocity_deg_s: dict[str, float],
    dt_s: float,
) -> int:
    """Compute how many controller ticks are needed under motor speed limits."""

    safe_dt_s = max(float(dt_s), 1e-6)
    cycles = 1
    for motor_id, target in target_targets.items():
        if motor_id not in start_targets:
            continue

        max_velocity = float(max_velocity_deg_s.get(motor_id, 180.0))
        if max_velocity <= 0.0:
            continue

        max_delta = max_velocity * safe_dt_s
        delta = abs(float(target) - float(start_targets[motor_id]))
        cycles = max(cycles, math.ceil(delta / max_delta))

    return max(cycles, 1)


def default_ik_helper() -> ArmIKHelper:
    return ArmIKHelper(
        ik_config_from_motors(
            {
                "m1": {"min_angle": -45.0, "max_angle": 45.0},
                "m2": {"min_angle": 45.0, "max_angle": 135.0},
                "m3": {"min_angle": 45.0, "max_angle": 135.0},
                "m4": {"min_angle": 45.0, "max_angle": 135.0},
            }
        )
    )


def ik_config_from_motors(motors: dict) -> ArmIKConfig:
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
        escape_config = LabEscapeConfig()
        self.escape = EscapeController(escape_config)
        self.camera_calibration = CameraCalibrationConfig.from_escape_config(
            escape_config,
        )
        self.ik = default_ik_helper()
        self.vanish_motor_targets = dict(settings.vanish_motor_targets)
        self._vanish_sequence = VanishSequence()
        self._was_started = False

    async def run(self) -> None:
        while True:
            if self.app.state.robot_state is not RobotState.started:
                if self._was_started:
                    self.escape.reset()
                    self.ik.reset()
                    self._reset_vanish_sequence()
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

        targets = self.next_targets(
            hand=hand,
            motor_state=motor_state,
        )

        motor_manager.set_target(targets)

    def next_targets(self, *, hand: Hand, motor_state: dict) -> dict[str, float]:
        self._sync_ik_limits(motor_state)
        current_targets = self._motor_targets(motor_state)

        if self._vanish_sequence.phase != VANISH_PHASE_IDLE:
            return self._run_vanish_sequence(
                current_targets=current_targets,
                motor_state=motor_state,
            )

        current_xyz = self._current_xyz(motor_state)
        hand_robot_yz = self._calibrate_hand_yz(hand)

        result = self.escape.step(
            hand_robot_yz=hand_robot_yz,
            current_xyz_cm=current_xyz,
            dt=float(motor_state.get("dt_s", CONTROL_INTERVAL_SECONDS)),
        )

        if not result.visible and result.planned_landing_xyz_cm is not None:
            self._start_vanish_sequence(
                landing_xyz_cm=result.planned_landing_xyz_cm,
                current_targets=current_targets,
                motor_state=motor_state,
            )
            return self._run_vanish_sequence(
                current_targets=current_targets, motor_state=motor_state
            )

        targets = self._motor_targets_for_xyz(
            result.next_xyz_cm,
            current_targets=current_targets,
            status=result.status,
            escape_state=result.escape_state,
        )
        return targets if targets is not None else current_targets

    def debug_yz_targets(
        self,
        *,
        y_cm: float,
        z_cm: float,
        update_ik_state: bool,
        motor_state: dict | None = None,
    ) -> tuple[dict[str, float] | None, float | None]:
        if motor_state is None:
            motor_state = self.app.state.motor_manager.get_state()
        self._sync_ik_limits(motor_state)
        current_targets = self._motor_targets(motor_state)

        ik_result = self.ik.solve_xyz(
            target_x_cm=0.0,
            target_y_cm=y_cm,
            target_z_cm=z_cm,
            update_state=update_ik_state,
            reachable_error_threshold_cm=REACHABLE_ERROR_THRESHOLD_CM,
        )
        if not ik_result.reachable:
            return None, ik_result.error_cm

        return (
            self._joint_targets_to_motor_targets(
                ik_result.motor_targets(),
                base_targets=current_targets,
            ),
            ik_result.error_cm,
        )

    def current_xyz_cm(self) -> XYZPosition:
        motor_state = self.app.state.motor_manager.get_state()
        self._sync_ik_limits(motor_state)
        return self._current_xyz(motor_state)

    def _motor_targets_for_xyz(
        self,
        xyz: XYZPosition,
        *,
        current_targets: dict[str, float],
        status: str,
        escape_state: str,
    ) -> dict[str, float] | None:
        try:
            ik_result = self.ik.solve_xyz(
                target_x_cm=xyz.x_cm,
                target_y_cm=xyz.y_cm,
                target_z_cm=xyz.z_cm,
                reachable_error_threshold_cm=REACHABLE_ERROR_THRESHOLD_CM,
            )
        except Exception as exc:
            logger.warning("IK failed; holding current motor targets: %s", exc)
            self.ik.reset()
            logger.debug(
                "escape controller held targets: state=%s xyz=(%.2f, %.2f, %.2f) status=%s error=%s",
                escape_state,
                xyz.x_cm,
                xyz.y_cm,
                xyz.z_cm,
                status,
                exc,
            )
            return None

        if not ik_result.reachable:
            logger.warning(
                "IK target unreachable; holding current motor targets: target=(%.2f, %.2f, %.2f) err=%.2f status=%s",
                xyz.x_cm,
                xyz.y_cm,
                xyz.z_cm,
                ik_result.error_cm,
                status,
            )
            return None

        targets = self._joint_targets_to_motor_targets(
            ik_result.motor_targets(),
            base_targets=current_targets,
        )
        logger.debug(
            "escape controller: state=%s xyz=(%.2f, %.2f, %.2f) reachable=%s err=%.2f status=%s",
            escape_state,
            xyz.x_cm,
            xyz.y_cm,
            xyz.z_cm,
            ik_result.reachable,
            ik_result.error_cm,
            status,
        )
        return targets

    def _start_vanish_sequence(
        self,
        *,
        landing_xyz_cm: XYZPosition,
        current_targets: dict[str, float],
        motor_state: dict,
    ) -> None:
        vanish_targets = self._complete_vanish_targets(current_targets)
        lift_cycles = self._cycles_for_motion(
            start_targets=current_targets,
            target_targets=vanish_targets,
            motor_state=motor_state,
        )
        self._vanish_sequence = VanishSequence(
            phase=VANISH_PHASE_LIFTING,
            landing_xyz_cm=landing_xyz_cm,
            lift_cycles_remaining=lift_cycles,
        )
        logger.debug(
            "starting vanish sequence: lift_cycles=%d landing_xyz=(%.2f, %.2f, %.2f)",
            lift_cycles,
            landing_xyz_cm.x_cm,
            landing_xyz_cm.y_cm,
            landing_xyz_cm.z_cm,
        )

    def _run_vanish_sequence(
        self,
        *,
        current_targets: dict[str, float],
        motor_state: dict,
    ) -> dict[str, float]:
        sequence = self._vanish_sequence

        if sequence.phase == VANISH_PHASE_LIFTING:
            targets = self._complete_vanish_targets(current_targets)
            sequence.lift_cycles_remaining -= 1
            if sequence.lift_cycles_remaining <= 0:
                sequence.phase = VANISH_PHASE_LANDING
            return targets

        if sequence.phase == VANISH_PHASE_LANDING:
            if sequence.landing_xyz_cm is None:
                self._reset_vanish_sequence()
                return current_targets

            if sequence.landing_targets is None:
                landing_targets = self._motor_targets_for_xyz(
                    sequence.landing_xyz_cm,
                    current_targets=current_targets,
                    status="vanish landing",
                    escape_state="BOUNDARY_ESCAPE",
                )
                if landing_targets is None:
                    logger.warning(
                        "Vanish landing IK failed; holding current motor targets"
                    )
                    self._reset_vanish_sequence()
                    return current_targets

                vanish_targets = self._complete_vanish_targets(current_targets)
                sequence.landing_targets = landing_targets
                sequence.land_cycles_remaining = self._cycles_for_motion(
                    start_targets=vanish_targets,
                    target_targets=landing_targets,
                    motor_state=motor_state,
                )
                logger.debug(
                    "vanish landing prepared: land_cycles=%d targets=%s",
                    sequence.land_cycles_remaining,
                    landing_targets,
                )

            sequence.land_cycles_remaining -= 1
            targets = dict(sequence.landing_targets)
            if sequence.land_cycles_remaining <= 0:
                self._reset_vanish_sequence()
            return targets

        self._reset_vanish_sequence()
        return current_targets

    def _complete_vanish_targets(
        self,
        base_targets: dict[str, float],
    ) -> dict[str, float]:
        targets = dict(base_targets)
        for motor_id, target in self.vanish_motor_targets.items():
            if motor_id in targets:
                targets[motor_id] = float(target)
        return targets

    def _cycles_for_motion(
        self,
        *,
        start_targets: dict[str, float],
        target_targets: dict[str, float],
        motor_state: dict,
    ) -> int:
        return motion_cycles(
            start_targets=start_targets,
            target_targets=target_targets,
            max_velocity_deg_s={
                motor_id: float(max_velocity)
                for motor_id, max_velocity in motor_state.get(
                    "max_velocity_deg_s",
                    {},
                ).items()
            },
            dt_s=float(motor_state.get("dt_s", CONTROL_INTERVAL_SECONDS)),
        )

    def _reset_vanish_sequence(self) -> None:
        self._vanish_sequence = VanishSequence()

    def _calibrate_hand_yz(self, hand: Hand) -> YZPosition | None:
        if not hand.detected or hand.x is None or hand.y is None:
            return None

        confidence = hand.confidence
        if (
            confidence is not None
            and confidence < self.camera_calibration.min_hand_confidence
        ):
            return None

        return self.camera_calibration.apply(float(hand.x), float(hand.y))

    def _sync_ik_limits(self, motor_state: dict) -> None:
        motors = motor_state.get("motors", {})
        if not all(
            motor_id in motors
            and "min_angle" in motors[motor_id]
            and "max_angle" in motors[motor_id]
            for motor_id in JOINT_TO_MOTOR_ID.values()
        ):
            return

        next_config = ik_config_from_motors(motors)
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
    def _motor_targets(motor_state: dict) -> dict[str, float]:
        current_targets = motor_state.get("current_targets")
        if current_targets:
            return {
                motor_id: float(target) for motor_id, target in current_targets.items()
            }

        motors = motor_state.get("motors", {})
        return {
            motor_id: float(state.get("target_angle", state.get("current_angle", 0.0)))
            for motor_id, state in motors.items()
        }
