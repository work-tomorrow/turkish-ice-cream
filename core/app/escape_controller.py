from __future__ import annotations

from dataclasses import dataclass

from core.app.lab_escape.common import LabEscapeConfig, Vec2, clamp, move_toward
from core.app.lab_escape.machine import EscapeStateMachine


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
class EscapeStep:
    next_xyz_cm: XYZPosition
    status: str
    escape_state: str
    transition_reason: str
    hand_yz_cm: tuple[float, float] | None
    visible: bool
    planned_landing_xyz_cm: XYZPosition | None = None


class EscapeController:
    """Adapter around the smooth-only lab escape state machine.

    Robot frame:
    - x = forward/back, unchanged by this controller
    - y = left/right
    - z = up/down
    """

    def __init__(self, config: LabEscapeConfig | None = None) -> None:
        self.config = config or LabEscapeConfig()
        self.machine = EscapeStateMachine(self.config)
        self._last_step: EscapeStep | None = None

    def reset(self) -> None:
        self.machine = EscapeStateMachine(self.config)
        self._last_step = None

    def step(
        self,
        *,
        hand_robot_yz: YZPosition | None,
        current_xyz_cm: XYZPosition,
        dt: float,
    ) -> EscapeStep:
        current_lab_pos = self._robot_yz_to_lab_vec(YZPosition(current_xyz_cm.y_cm, current_xyz_cm.z_cm))
        hand_vec = self._robot_yz_to_lab_vec(hand_robot_yz) if hand_robot_yz is not None else None

        algo_result = self.machine.update(
            ice_pos=current_lab_pos,
            hand_pos=hand_vec,
            dt=max(float(dt), 1e-3),
        )

        next_yz = self._lab_vec_to_robot_yz(algo_result.pos)
        next_yz = self._move_yz_toward(
            current=YZPosition(current_xyz_cm.y_cm, current_xyz_cm.z_cm),
            target=next_yz,
            max_distance=self.config.max_command_step_cm,
        )
        next_yz = self._clamp_robot_yz(next_yz)

        next_xyz = XYZPosition(
            x_cm=float(current_xyz_cm.x_cm),
            y_cm=next_yz.y_cm,
            z_cm=next_yz.z_cm,
        )

        planned_landing_xyz = (
            self._lab_vec_to_xyz(algo_result.planned_landing, current_x_cm=current_xyz_cm.x_cm)
            if algo_result.planned_landing is not None
            else None
        )

        result = EscapeStep(
            next_xyz_cm=next_xyz,
            status=algo_result.status,
            escape_state=self.machine.state.name,
            transition_reason=self.machine.transition_reason,
            hand_yz_cm=self._vec_tuple(hand_vec),
            visible=bool(algo_result.visible),
            planned_landing_xyz_cm=planned_landing_xyz,
        )
        self._last_step = result
        return result

    @property
    def last_step(self) -> EscapeStep | None:
        return self._last_step

    def _robot_yz_to_lab_vec(self, position: YZPosition) -> Vec2:
        y_cm = clamp(position.y_cm, self.config.target_y_min_cm, self.config.target_y_max_cm)
        z_cm = clamp(position.z_cm, self.config.target_z_min_cm, self.config.target_z_max_cm)
        return Vec2(y_cm, z_cm)

    def _lab_vec_to_xyz(self, pos: Vec2, *, current_x_cm: float) -> XYZPosition:
        yz = self._lab_vec_to_robot_yz(pos)
        return XYZPosition(x_cm=float(current_x_cm), y_cm=yz.y_cm, z_cm=yz.z_cm)

    def _lab_vec_to_robot_yz(self, pos: Vec2) -> YZPosition:
        return self._clamp_robot_yz(YZPosition(y_cm=pos.x, z_cm=pos.y))

    def _clamp_robot_yz(self, position: YZPosition) -> YZPosition:
        return YZPosition(
            y_cm=clamp(position.y_cm, self.config.target_y_min_cm, self.config.target_y_max_cm),
            z_cm=clamp(position.z_cm, self.config.target_z_min_cm, self.config.target_z_max_cm),
        )

    @staticmethod
    def _move_yz_toward(*, current: YZPosition, target: YZPosition, max_distance: float) -> YZPosition:
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
