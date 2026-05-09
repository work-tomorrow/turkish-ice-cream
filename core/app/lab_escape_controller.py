from __future__ import annotations

from dataclasses import dataclass

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
    pixels_per_cm: float = 10.0
    boundary_size_cm: float = 80.0
    origin_to_left_boundary_cm: float = 40.0
    origin_to_top_boundary_cm: float = 40.0

    max_xyz_step_cm: float = 1.8

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
    Adapter around the kosu Turkish escape state machine.

    The canonical XYZ frame is the robot frame:
    x = forward/back, y = left/right, z = up/down.

    The kosu algorithm is a 2D square workspace. Its x axis maps to robot Y,
    and its y axis maps inversely to robot Z.
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
        planned_landing_xyz = (
            self._lab_to_xyz(
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
            pixels_per_cm=config.pixels_per_cm,
            boundary_size_cm=config.boundary_size_cm,
            origin_to_left_boundary_cm=config.origin_to_left_boundary_cm,
            origin_to_top_boundary_cm=config.origin_to_top_boundary_cm,
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
        origin = self.world.origin()
        return Vec2(
            origin.x + y_cm * self.world.pixels_per_cm,
            origin.y - z_cm * self.world.pixels_per_cm,
        )

    def _lab_to_xyz(self, pos: Vec2, *, current_x_cm: float) -> XYZPosition:
        yz = self._lab_to_robot_yz(pos)
        return XYZPosition(
            x_cm=float(current_x_cm),
            y_cm=yz.y_cm,
            z_cm=yz.z_cm,
        )

    def _lab_to_robot_yz(self, pos: Vec2) -> YZPosition:
        origin = self.world.origin()
        return self._clamp_robot_yz(
            YZPosition(
                y_cm=(pos.x - origin.x) / self.world.pixels_per_cm,
                z_cm=(origin.y - pos.y) / self.world.pixels_per_cm,
            )
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
    def _vec_tuple(pos: Vec2 | None) -> tuple[float, float] | None:
        if pos is None:
            return None
        return (float(pos.x), float(pos.y))
