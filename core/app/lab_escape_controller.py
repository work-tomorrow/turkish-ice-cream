from __future__ import annotations

from dataclasses import dataclass

from core.app.lab_escape.common import (
    Rect,
    Vec2,
    WorldConfig,
    clamp,
    clamp_position,
    move_toward,
)
from core.app.lab_escape.state_machine import ArmStateMachine


@dataclass(frozen=True)
class XYZPosition:
    x_cm: float
    y_cm: float
    z_cm: float


@dataclass(frozen=True)
class YZPosition:
    y_cm: float
    z_cm: float


@dataclass(frozen=True)
class LabEscapeControllerConfig:
    vision_width_px: float = 640.0
    vision_height_px: float = 480.0

    # The lab escape algorithm is 2D. Its x axis maps to robot Y; its y axis
    # maps to robot Z.
    target_y_min_cm: float = -18.0
    target_y_max_cm: float = 18.0
    target_z_min_cm: float = -10.0
    target_z_max_cm: float = 14.0

    max_xyz_step_cm: float = 1.8


@dataclass(frozen=True)
class LabEscapeControllerResult:
    next_xyz_cm: XYZPosition
    status: str
    lab_state: str
    transition_reason: str
    hand_lab_pos: tuple[float, float] | None


class LabEscapeController:
    """
    Adapter around the lab escape state machine.

    The canonical XYZ frame is the robot frame:
    x = forward/back, y = left/right, z = up/down.

    This class only converts robot-frame Y/Z to the 2D lab world and back.
    It does not know motor ids, motor angles, FK, or IK.
    """

    def __init__(self, config: LabEscapeControllerConfig | None = None) -> None:
        self.config = config or LabEscapeControllerConfig()
        self.world = self._build_world_config(self.config)
        self.state_machine = ArmStateMachine(self.world)
        self._last_result: LabEscapeControllerResult | None = None

    def reset(self) -> None:
        self.state_machine = ArmStateMachine(self.world)
        self._last_result = None

    def compute_next_xyz(
        self,
        *,
        hand_robot_yz: YZPosition | None,
        current_xyz_cm: XYZPosition,
        dt: float,
    ) -> LabEscapeControllerResult:
        current_lab_pos = self._robot_yz_to_lab(
            YZPosition(current_xyz_cm.y_cm, current_xyz_cm.z_cm)
        )
        hand_lab_pos = (
            self._robot_yz_to_lab(hand_robot_yz)
            if hand_robot_yz is not None
            else None
        )

        algo_result = self.state_machine.update(
            ice_pos=current_lab_pos,
            hand_pos=hand_lab_pos,
            dt=max(float(dt), 1e-3),
            hand_sample_fresh=True,
        )

        next_yz = self._lab_to_robot_yz(algo_result.pos)
        next_yz = self._move_yz_toward(
            current=YZPosition(current_xyz_cm.y_cm, current_xyz_cm.z_cm),
            target=next_yz,
            max_distance=self.config.max_xyz_step_cm,
        )
        next_yz = self._clamp_robot_yz(next_yz)
        next_xyz = XYZPosition(
            x_cm=float(current_xyz_cm.x_cm),
            y_cm=next_yz.y_cm,
            z_cm=next_yz.z_cm,
        )

        result = LabEscapeControllerResult(
            next_xyz_cm=next_xyz,
            status=algo_result.status,
            lab_state=self.state_machine.state.name,
            transition_reason=self.state_machine.transition_reason,
            hand_lab_pos=self._vec_tuple(hand_lab_pos),
        )
        self._last_result = result
        return result

    @property
    def last_result(self) -> LabEscapeControllerResult | None:
        return self._last_result

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

    def _robot_yz_to_lab(self, position: YZPosition) -> Vec2:
        y_cm = clamp(
            position.y_cm,
            self.config.target_y_min_cm,
            self.config.target_y_max_cm,
        )
        z_cm = clamp(
            position.z_cm,
            self.config.target_z_min_cm,
            self.config.target_z_max_cm,
        )

        x_ratio = self._unlerp(
            self.config.target_y_min_cm,
            self.config.target_y_max_cm,
            y_cm,
        )
        y_ratio = self._unlerp(
            self.config.target_z_max_cm,
            self.config.target_z_min_cm,
            z_cm,
        )

        return clamp_position(
            Vec2(x_ratio * self.world.width, y_ratio * self.world.height),
            self.world.ice_radius,
            self.world.width,
            self.world.height,
        )

    def _lab_to_robot_yz(self, pos: Vec2) -> YZPosition:
        x_ratio = clamp(pos.x / self.world.width, 0.0, 1.0)
        y_ratio = clamp(pos.y / self.world.height, 0.0, 1.0)

        return YZPosition(
            y_cm=self._lerp(
                self.config.target_y_min_cm,
                self.config.target_y_max_cm,
                x_ratio,
            ),
            z_cm=self._lerp(
                self.config.target_z_max_cm,
                self.config.target_z_min_cm,
                y_ratio,
            ),
        )

    def _clamp_robot_yz(self, position: YZPosition) -> YZPosition:
        return YZPosition(
            y_cm=clamp(
                position.y_cm,
                self.config.target_y_min_cm,
                self.config.target_y_max_cm,
            ),
            z_cm=clamp(
                position.z_cm,
                self.config.target_z_min_cm,
                self.config.target_z_max_cm,
            ),
        )

    @staticmethod
    def _move_yz_toward(
        *,
        current: YZPosition,
        target: YZPosition,
        max_distance: float,
    ) -> YZPosition:
        next_pos = move_toward(
            current=Vec2(current.y_cm, current.z_cm),
            target=Vec2(target.y_cm, target.z_cm),
            max_distance=max_distance,
        )
        return YZPosition(y_cm=next_pos.x, z_cm=next_pos.y)

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
