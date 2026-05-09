from __future__ import annotations

from dataclasses import dataclass, field
import logging
import sys
from pathlib import Path
from typing import Any

from core.app.models import Hand

logger = logging.getLogger(__name__)

_LAB_DIR = Path(__file__).resolve().parents[2] / "lab_algo"
if str(_LAB_DIR) not in sys.path:
    sys.path.insert(0, str(_LAB_DIR))

from common import (  # noqa: E402
    Rect,
    Vec2,
    WorldConfig,
    clamp,
    clamp_position,
    move_toward,
)
from state_machine import ArmStateMachine  # noqa: E402


@dataclass(frozen=True)
class MotorJointCalibration:
    motor_id: str
    min_deg: float
    max_deg: float
    sign: float = 1.0
    zero_deg: float = 0.0


@dataclass(frozen=True)
class LabEscapeControllerConfig:
    vision_width_px: float = 640.0
    vision_height_px: float = 480.0

    # Conservative first hardware test workspace. These are IK target offsets.
    target_y_min_cm: float = -18.0
    target_y_max_cm: float = 18.0
    target_z_min_cm: float = -10.0
    target_z_max_cm: float = 14.0
    initial_target_y_cm: float = 0.0
    initial_target_z_cm: float = 0.0

    max_target_step_cm: float = 1.8
    reachable_error_threshold_cm: float = 2.0
    min_hand_confidence: float = 0.2

    # Safe logical motor targets. Servo zeros default to 90 so the first IK
    # command starts near the existing home pose instead of the hard endpoint.
    joint_calibration: dict[str, MotorJointCalibration] = field(
        default_factory=lambda: {
            "J1": MotorJointCalibration("m1", -45.0, 45.0, sign=1.0, zero_deg=0.0),
            "J2": MotorJointCalibration("m2", 45.0, 135.0, sign=-1.0, zero_deg=90.0),
            "J3": MotorJointCalibration("m3", 45.0, 135.0, sign=1.0, zero_deg=90.0),
            "J4": MotorJointCalibration("m4", 45.0, 135.0, sign=1.0, zero_deg=90.0),
        }
    )


@dataclass(frozen=True)
class LabEscapeControllerResult:
    targets: dict[str, float]
    status: str
    lab_state: str
    transition_reason: str
    hand_lab_pos: tuple[float, float] | None
    target_y_cm: float
    target_z_cm: float
    ik_reachable: bool
    ik_error_cm: float | None


class LabEscapeController:
    """
    Hardware-facing adapter around the lab state machine.

    The lab code still owns the escape state design and motion choice. This
    adapter only converts camera pixels to the lab world, converts lab positions
    to safe real Y/Z targets, and solves those targets into motor angles.
    """

    def __init__(self, config: LabEscapeControllerConfig | None = None) -> None:
        self.config = config or LabEscapeControllerConfig()
        self.world = self._build_world_config(self.config)
        self.state_machine = ArmStateMachine(self.world)
        self.ice_pos = self._physical_to_lab(
            self.config.initial_target_y_cm,
            self.config.initial_target_z_cm,
        )
        self._last_physical_pos = Vec2(
            self.config.initial_target_y_cm,
            self.config.initial_target_z_cm,
        )
        self._last_result: LabEscapeControllerResult | None = None
        self._ik = None
        self._ik_error: str | None = None
        self._init_ik()

    def reset(self) -> None:
        self.state_machine = ArmStateMachine(self.world)
        self.ice_pos = self._physical_to_lab(
            self.config.initial_target_y_cm,
            self.config.initial_target_z_cm,
        )
        self._last_physical_pos = Vec2(
            self.config.initial_target_y_cm,
            self.config.initial_target_z_cm,
        )
        self._last_result = None
        if self._ik is not None:
            self._ik.reset()

    def compute_targets(
        self,
        *,
        hand: Hand,
        motor_state: dict[str, Any],
        dt: float,
    ) -> LabEscapeControllerResult:
        current_targets = self._current_motor_targets(motor_state)
        hand_pos = self._hand_to_lab(hand)

        algo_result = self.state_machine.update(
            ice_pos=self.ice_pos,
            hand_pos=hand_pos,
            dt=max(float(dt), 1e-3),
            hand_sample_fresh=True,
        )

        physical_pos = self._lab_to_physical(algo_result.pos)
        physical_pos = move_toward(
            current=self._last_physical_pos,
            target=physical_pos,
            max_distance=self.config.max_target_step_cm,
        )
        physical_pos = self._clamp_physical(physical_pos)

        self._last_physical_pos = physical_pos
        self.ice_pos = self._physical_to_lab(physical_pos.x, physical_pos.y)

        if self._ik is None:
            result = LabEscapeControllerResult(
                targets=current_targets,
                status=f"{algo_result.status}; IK unavailable: {self._ik_error}",
                lab_state=self.state_machine.state.name,
                transition_reason=self.state_machine.transition_reason,
                hand_lab_pos=self._vec_tuple(hand_pos),
                target_y_cm=physical_pos.x,
                target_z_cm=physical_pos.y,
                ik_reachable=False,
                ik_error_cm=None,
            )
            self._last_result = result
            return result

        try:
            ik_result = self._ik.solve_yz(
                target_y_cm=physical_pos.x,
                target_z_cm=physical_pos.y,
                reachable_error_threshold_cm=self.config.reachable_error_threshold_cm,
            )
            targets = self._map_ik_targets(ik_result.motor_targets())
            targets = self._clamp_to_motor_state(targets, motor_state)
            status = algo_result.status
            ik_reachable = bool(ik_result.reachable)
            ik_error_cm = float(ik_result.error_cm)
        except Exception as exc:
            logger.exception("Lab escape controller IK step failed")
            targets = current_targets
            status = f"{algo_result.status}; IK failed: {exc}"
            ik_reachable = False
            ik_error_cm = None

        result = LabEscapeControllerResult(
            targets=targets,
            status=status,
            lab_state=self.state_machine.state.name,
            transition_reason=self.state_machine.transition_reason,
            hand_lab_pos=self._vec_tuple(hand_pos),
            target_y_cm=physical_pos.x,
            target_z_cm=physical_pos.y,
            ik_reachable=ik_reachable,
            ik_error_cm=ik_error_cm,
        )
        self._last_result = result
        return result

    @property
    def last_result(self) -> LabEscapeControllerResult | None:
        return self._last_result

    def _init_ik(self) -> None:
        try:
            from core.app.kinematics_helper import ArmIKConfig, ArmIKHelper, JointLimit
        except Exception as exc:
            self._ik_error = str(exc)
            logger.warning(
                "IK helper unavailable; controller will hold motor targets: %s",
                exc,
            )
            return

        joint_limits = {
            joint: JointLimit(
                min_deg=calibration.min_deg,
                max_deg=calibration.max_deg,
                sign=calibration.sign,
                zero_deg=calibration.zero_deg,
            )
            for joint, calibration in self.config.joint_calibration.items()
        }
        self._ik = ArmIKHelper(
            ArmIKConfig(
                max_extra_pitch_deg=12.0,
                max_extra_yaw_deg=10.0,
                joint_limits=joint_limits,
                max_iterations=80,
                stop_error_cm=0.05,
                step_scale=0.4,
            )
        )

    def _build_world_config(self, config: LabEscapeControllerConfig) -> WorldConfig:
        width = int(round(config.vision_width_px))
        height = int(round(config.vision_height_px))
        return WorldConfig(
            width=width,
            height=height,
            control_period=0.10,
            hand_radius=55.0,
            ice_radius=22.0,
            escape_trigger_distance=160.0,
            close_distance=45.0,
            boundary_enter_margin=80.0,
            boundary_exit_margin=115.0,
            min_safe_distance_extra=15.0,
            blind_zone=Rect(width * 0.4, height * 0.35, width * 0.2, height * 0.3),
        )

    def _hand_to_lab(self, hand: Hand) -> Vec2 | None:
        if not hand.detected or hand.x is None or hand.y is None:
            return None

        confidence = hand.confidence
        if confidence is not None and confidence < self.config.min_hand_confidence:
            return None

        x = clamp(float(hand.x), 0.0, self.config.vision_width_px)
        y = clamp(float(hand.y), 0.0, self.config.vision_height_px)

        lab_x = x / self.config.vision_width_px * self.world.width
        lab_y = y / self.config.vision_height_px * self.world.height
        return Vec2(lab_x, lab_y)

    def _lab_to_physical(self, pos: Vec2) -> Vec2:
        x_ratio = clamp(pos.x / self.world.width, 0.0, 1.0)
        y_ratio = clamp(pos.y / self.world.height, 0.0, 1.0)

        target_y = self._lerp(
            self.config.target_y_min_cm,
            self.config.target_y_max_cm,
            x_ratio,
        )
        target_z = self._lerp(
            self.config.target_z_max_cm,
            self.config.target_z_min_cm,
            y_ratio,
        )
        return Vec2(target_y, target_z)

    def _physical_to_lab(self, target_y_cm: float, target_z_cm: float) -> Vec2:
        target_y_cm = clamp(
            target_y_cm,
            self.config.target_y_min_cm,
            self.config.target_y_max_cm,
        )
        target_z_cm = clamp(
            target_z_cm,
            self.config.target_z_min_cm,
            self.config.target_z_max_cm,
        )

        x_ratio = self._unlerp(
            self.config.target_y_min_cm,
            self.config.target_y_max_cm,
            target_y_cm,
        )
        y_ratio = self._unlerp(
            self.config.target_z_max_cm,
            self.config.target_z_min_cm,
            target_z_cm,
        )

        return clamp_position(
            Vec2(x_ratio * self.world.width, y_ratio * self.world.height),
            self.world.ice_radius,
            self.world.width,
            self.world.height,
        )

    def _clamp_physical(self, pos: Vec2) -> Vec2:
        return Vec2(
            clamp(pos.x, self.config.target_y_min_cm, self.config.target_y_max_cm),
            clamp(pos.y, self.config.target_z_min_cm, self.config.target_z_max_cm),
        )

    def _map_ik_targets(self, joint_targets: dict[str, float]) -> dict[str, float]:
        targets: dict[str, float] = {}
        for joint, calibration in self.config.joint_calibration.items():
            if joint in joint_targets:
                targets[calibration.motor_id] = float(joint_targets[joint])
        return targets

    def _clamp_to_motor_state(
        self,
        targets: dict[str, float],
        motor_state: dict[str, Any],
    ) -> dict[str, float]:
        output = self._current_motor_targets(motor_state)
        motors = motor_state.get("motors", {})

        for motor_id, target in targets.items():
            state = motors.get(motor_id)
            if state is None:
                continue

            min_angle = float(state.get("min_angle", -180.0))
            max_angle = float(state.get("max_angle", 180.0))
            output[motor_id] = clamp(float(target), min_angle, max_angle)

        return output

    def _current_motor_targets(self, motor_state: dict[str, Any]) -> dict[str, float]:
        motors = motor_state.get("motors", {})
        return {
            motor_id: float(state.get("target_angle", state.get("current_angle", 0.0)))
            for motor_id, state in motors.items()
        }

    @staticmethod
    def _lerp(start: float, end: float, ratio: float) -> float:
        return start + (end - start) * ratio

    @staticmethod
    def _unlerp(start: float, end: float, value: float) -> float:
        if abs(end - start) < 1e-9:
            return 0.0
        return clamp((value - start) / (end - start), 0.0, 1.0)

    @staticmethod
    def _vec_tuple(pos: Vec2 | None) -> tuple[float, float] | None:
        if pos is None:
            return None
        return (float(pos.x), float(pos.y))
