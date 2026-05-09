from __future__ import annotations

import math
from enum import Enum, auto

from core.app.lab_escape.base import BaseAlgo
from core.app.lab_escape.common import (
    ACTIVE_BORDER_COLOR,
    ACTIVE_COLOR,
    BLIND_BORDER_COLOR,
    BLIND_COLOR,
    BOUNDARY_BORDER_COLOR,
    BOUNDARY_COLOR,
    IDLE_BORDER_COLOR,
    IDLE_COLOR,
    RETREAT_BORDER_COLOR,
    RETREAT_COLOR,
    AlgoResult,
    RobotState,
    Vec2,
    WorldConfig,
    clamp,
    clamp_position_to_square,
    clamp_position_with_config,
    distance,
    distance_from_nearest_wall_with_config,
    move_toward,
    workspace_center,
)


def _zero() -> Vec2:
    return Vec2(0.0, 0.0)


def _safe_normalized(v: Vec2) -> Vec2:
    if v.length() < 1e-6:
        return _zero()
    return v.normalized()


def _lerp_vec(a: Vec2, b: Vec2, t: float) -> Vec2:
    return a * (1.0 - t) + b * t


def _center(config: WorldConfig) -> Vec2:
    return workspace_center(config)


def _center_pull(pos: Vec2, config: WorldConfig) -> Vec2:
    return _safe_normalized(_center(config) - pos)


def _wall_repulsion(pos: Vec2, config: WorldConfig, margin: float | None = None) -> Vec2:
    m = config.boundary_exit_margin_cm if margin is None else margin
    force = Vec2(0.0, 0.0)

    left = pos.x - config.ice_radius_cm - config.y_min_cm
    right = config.y_max_cm - (pos.x + config.ice_radius_cm)
    bottom = pos.y - config.ice_radius_cm - config.z_min_cm
    top = config.z_max_cm - (pos.y + config.ice_radius_cm)

    if left < m:
        force += Vec2((m - left) / m, 0.0)
    if right < m:
        force += Vec2(-(m - right) / m, 0.0)
    if bottom < m:
        force += Vec2(0.0, (m - bottom) / m)
    if top < m:
        force += Vec2(0.0, -(m - top) / m)

    return force


def _pick_safe_landing(danger: Vec2 | None, config: WorldConfig) -> Vec2:
    center = _center(config)
    margin = config.ice_radius_cm + config.boundary_exit_margin_cm

    candidates = [
        center,
        Vec2(config.y_min_cm + margin, center.y),
        Vec2(config.y_max_cm - margin, center.y),
        Vec2(center.x, config.z_min_cm + margin),
        Vec2(center.x, config.z_max_cm - margin),
        Vec2(config.y_min_cm + margin, config.z_min_cm + margin),
        Vec2(config.y_max_cm - margin, config.z_min_cm + margin),
        Vec2(config.y_min_cm + margin, config.z_max_cm - margin),
        Vec2(config.y_max_cm - margin, config.z_max_cm - margin),
    ]

    best = center
    best_score = -float("inf")
    for candidate in candidates:
        p = clamp_position_to_square(
            candidate,
            config.ice_radius_cm,
            config.y_min_cm,
            config.y_max_cm,
            config.z_min_cm,
            config.z_max_cm,
            margin=1.0,
        )
        wall_dist = distance_from_nearest_wall_with_config(p, config.ice_radius_cm, config)
        score = wall_dist * 1.2 - distance(p, center) * 0.15
        if danger is not None:
            score += distance(p, danger) * 2.0
        if score > best_score:
            best_score = score
            best = p
    return best


class SmoothIdleAlgo(BaseAlgo):
    """No threat: return gently toward center and make a tiny smooth hover."""

    def __init__(self) -> None:
        self.t = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config) -> None:
        self.t = 0.0

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        self.t += dt
        center = _center(config)
        phase = 2.0 * math.pi * 0.8 * self.t
        target = center + Vec2(math.sin(phase) * 1.0, math.cos(phase) * 0.5)
        target = clamp_position_with_config(target, config.ice_radius_cm, config)
        next_pos = move_toward(ice_pos, target, config.idle_speed_cm_s * dt)
        return AlgoResult(next_pos, IDLE_COLOR, IDLE_BORDER_COLOR, "IDLE / smooth center hover")


class SmoothMomentumEscapeAlgo(BaseAlgo):
    """Threat present: continuous vector-field escape, no eight-direction logic."""

    def __init__(self) -> None:
        self.direction = Vec2(1.0, 0.0)
        self.prev_hand_pos: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config) -> None:
        self.direction = _safe_normalized(ice_pos - hand_pos) if hand_pos else Vec2(1.0, 0.0)
        self.prev_hand_pos = hand_pos

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if hand_pos is None:
            return AlgoResult(ice_pos, ACTIVE_COLOR, ACTIVE_BORDER_COLOR, "NORMAL / no hand")

        away = _safe_normalized(ice_pos - hand_pos)
        wall = _wall_repulsion(ice_pos, config, margin=12.0)
        center = _center_pull(ice_pos, config)

        prediction = _zero()
        if self.prev_hand_pos is not None:
            hand_motion = _safe_normalized(hand_pos - self.prev_hand_pos)
            prediction = _safe_normalized(away - hand_motion)

        desired = away * 1.7 + wall * 1.4 + center * 0.35 + prediction * 0.5
        desired_dir = _safe_normalized(desired)
        if desired_dir.length() < 1e-6:
            desired_dir = self.direction

        self.direction = _safe_normalized(_lerp_vec(self.direction, desired_dir, 0.22))
        next_pos = ice_pos + self.direction * config.normal_speed_cm_s * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)
        self.prev_hand_pos = hand_pos

        return AlgoResult(next_pos, ACTIVE_COLOR, ACTIVE_BORDER_COLOR, "NORMAL / smooth momentum escape")


class SmoothBoundaryCenterPullAlgo(BaseAlgo):
    """Near wall: prioritize pulling back into workspace."""

    def __init__(self) -> None:
        self.direction = Vec2(0.0, 1.0)

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config) -> None:
        self.direction = _center_pull(ice_pos, config)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        away = _safe_normalized(ice_pos - hand_pos) if hand_pos is not None else _zero()
        desired = _center_pull(ice_pos, config) * 1.6 + _wall_repulsion(ice_pos, config) * 2.2 + away * 0.4
        desired_dir = _safe_normalized(desired)
        if desired_dir.length() < 1e-6:
            desired_dir = self.direction
        self.direction = _safe_normalized(_lerp_vec(self.direction, desired_dir, 0.30))
        next_pos = ice_pos + self.direction * config.boundary_speed_cm_s * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)
        return AlgoResult(next_pos, BOUNDARY_COLOR, BOUNDARY_BORDER_COLOR, "BOUNDARY / smooth center pull")


class RetreatPhase(Enum):
    UP = auto()
    WAIT = auto()
    DOWN = auto()


class SmoothCloseRetreatAlgo(BaseAlgo):
    """Very close contact: pull upward, pause briefly, then land at a safe point."""

    def __init__(self) -> None:
        self.is_active = False
        self.phase = RetreatPhase.UP
        self.timer = 0.0
        self.up_target: Vec2 | None = None
        self.down_target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config) -> None:
        self.is_active = True
        self.phase = RetreatPhase.UP
        self.timer = 0.0
        self.up_target = Vec2(
            clamp(ice_pos.x, config.y_min_cm + config.ice_radius_cm, config.y_max_cm - config.ice_radius_cm),
            config.z_max_cm - config.ice_radius_cm,
        )
        self.down_target = None

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if not self.is_active:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        if self.phase == RetreatPhase.UP:
            assert self.up_target is not None
            next_pos = move_toward(ice_pos, self.up_target, config.retreat_speed_cm_s * dt)
            if distance(next_pos, self.up_target) < 1.0:
                next_pos = self.up_target
                self.phase = RetreatPhase.WAIT
                self.timer = 0.18
            return AlgoResult(next_pos, RETREAT_COLOR, RETREAT_BORDER_COLOR, "CLOSE / smooth up")

        if self.phase == RetreatPhase.WAIT:
            self.timer -= dt
            if self.timer <= 0.0:
                self.down_target = _pick_safe_landing(hand_pos or last_hand_pos, config)
                self.phase = RetreatPhase.DOWN
            return AlgoResult(ice_pos, RETREAT_COLOR, RETREAT_BORDER_COLOR, f"CLOSE / wait {max(self.timer, 0.0):.2f}s")

        assert self.down_target is not None
        next_pos = move_toward(ice_pos, self.down_target, config.retreat_speed_cm_s * dt)
        if distance(next_pos, self.down_target) < 1.0:
            next_pos = self.down_target
            self.is_active = False
            return AlgoResult(next_pos, ACTIVE_COLOR, ACTIVE_BORDER_COLOR, "CLOSE / finished")
        return AlgoResult(next_pos, RETREAT_COLOR, RETREAT_BORDER_COLOR, "CLOSE / smooth down")


class SmoothBlindVectorEscapeAlgo(BaseAlgo):
    """Hand disappeared near blind zone: keep moving away from the last known hand point briefly."""

    def __init__(self) -> None:
        self.direction = Vec2(1.0, 0.0)

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config) -> None:
        danger = last_hand_pos or config.blind_zone.center()
        self.direction = _safe_normalized(ice_pos - danger)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        danger = last_hand_pos or config.blind_zone.center()
        desired = _safe_normalized(ice_pos - danger) * 1.4 + _wall_repulsion(ice_pos, config, margin=10.0) + _center_pull(ice_pos, config) * 0.25
        desired_dir = _safe_normalized(desired)
        if desired_dir.length() < 1e-6:
            desired_dir = self.direction
        self.direction = _safe_normalized(_lerp_vec(self.direction, desired_dir, 0.25))
        next_pos = ice_pos + self.direction * config.normal_speed_cm_s * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)
        return AlgoResult(next_pos, BLIND_COLOR, BLIND_BORDER_COLOR, "BLIND / smooth vector escape")


def create_algorithms() -> dict[RobotState, BaseAlgo]:
    """Single smooth-only algorithm set. No profiles, no eight-direction variants."""
    return {
        RobotState.IDLE: SmoothIdleAlgo(),
        RobotState.NORMAL_ESCAPE: SmoothMomentumEscapeAlgo(),
        RobotState.BOUNDARY_ESCAPE: SmoothBoundaryCenterPullAlgo(),
        RobotState.CLOSE_RETREAT: SmoothCloseRetreatAlgo(),
        RobotState.BLIND_SPOT_ESCAPE: SmoothBlindVectorEscapeAlgo(),
    }
