from __future__ import annotations

from dataclasses import dataclass
import math

from core.app.lab_escape.common import (
    Vec2,
    WorldConfig,
    clamp,
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
    pixels_per_cm: float = 32.0
    boundary_size_cm: float = 25.0
    origin_to_left_boundary_cm: float = 12.5
    origin_to_top_boundary_cm: float = 12.5

    max_xyz_step_cm: float = 1.8
    target_radius_cm: float = 0.0

    @property
    def vision_width_px(self) -> float:
        return self.boundary_size_cm * self.pixels_per_cm

    @property
    def vision_height_px(self) -> float:
        return self.boundary_size_cm * self.pixels_per_cm

    @property
    def target_y_min_cm(self) -> float:
        return -self.origin_to_left_boundary_cm

    @property
    def target_y_max_cm(self) -> float:
        return self.boundary_size_cm - self.origin_to_left_boundary_cm

    @property
    def target_z_min_cm(self) -> float:
        return -(self.boundary_size_cm - self.origin_to_top_boundary_cm)

    @property
    def target_z_max_cm(self) -> float:
        return self.origin_to_top_boundary_cm


@dataclass(frozen=True)
class LabEscapeControllerResult:
    next_xyz_cm: XYZPosition
    status: str
    lab_state: str
    transition_reason: str
    hand_lab_pos: tuple[float, float] | None
    visible: bool
    planned_landing_xyz_cm: XYZPosition | None = None


class LabEscapeController:
    """
    Adapter around the lab escape state machine.

    The canonical XYZ frame is the robot frame:
    x = forward/back, y = left/right, z = up/down.

    The state machine operates in the same YZ plane: Vec2.x is y_cm, Vec2.y is z_cm.
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
        current_lab_pos = self._robot_yz_to_lab_vec(
            YZPosition(current_xyz_cm.y_cm, current_xyz_cm.z_cm)
        )
        hand_lab_pos = (
            self._robot_yz_to_lab_vec(hand_robot_yz)
            if hand_robot_yz is not None
            else None
        )

        algo_result = self.state_machine.update(
            ice_pos=current_lab_pos,
            hand_pos=hand_lab_pos,
            dt=max(float(dt), 1e-3),
        )

        next_yz = self._lab_vec_to_robot_yz(algo_result.pos)
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
        planned_landing_xyz = (
            self._lab_vec_to_xyz(
                algo_result.planned_landing,
                current_x_cm=current_xyz_cm.x_cm,
            )
            if algo_result.planned_landing is not None
            else None
        )

        result = LabEscapeControllerResult(
            next_xyz_cm=next_xyz,
            status=algo_result.status,
            lab_state=self.state_machine.state.name,
            transition_reason=self.state_machine.transition_reason,
            hand_lab_pos=self._vec_tuple(hand_lab_pos),
            visible=bool(algo_result.visible),
            planned_landing_xyz_cm=planned_landing_xyz,
        )
        self._last_result = result
        return result

    @property
    def last_result(self) -> LabEscapeControllerResult | None:
        return self._last_result

    def _build_world_config(self, config: LabEscapeControllerConfig) -> WorldConfig:
        return WorldConfig(
            boundary_size_cm=config.boundary_size_cm,
            origin_to_left_boundary_cm=config.origin_to_left_boundary_cm,
            origin_to_top_boundary_cm=config.origin_to_top_boundary_cm,
        )

    def _robot_yz_to_lab_vec(self, position: YZPosition) -> Vec2:
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
        return Vec2(y_cm, z_cm)

    def _lab_vec_to_xyz(self, pos: Vec2, *, current_x_cm: float) -> XYZPosition:
        yz = self._lab_vec_to_robot_yz(pos)
        return XYZPosition(
            x_cm=float(current_x_cm),
            y_cm=yz.y_cm,
            z_cm=yz.z_cm,
        )

    def _lab_vec_to_robot_yz(self, pos: Vec2) -> YZPosition:
        return self._clamp_robot_yz(YZPosition(y_cm=pos.x, z_cm=pos.y))

    def _clamp_robot_yz(self, position: YZPosition) -> YZPosition:
        square_clamped = YZPosition(
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

        radius = float(self.config.target_radius_cm)
        distance_from_origin = math.hypot(
            square_clamped.y_cm,
            square_clamped.z_cm,
        )
        if radius <= 0.0 or distance_from_origin <= radius:
            return square_clamped

        scale = radius / distance_from_origin
        return YZPosition(
            y_cm=square_clamped.y_cm * scale,
            z_cm=square_clamped.z_cm * scale,
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
    def _vec_tuple(pos: Vec2 | None) -> tuple[float, float] | None:
        if pos is None:
            return None
        return (float(pos.x), float(pos.y))
