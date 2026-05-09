from __future__ import annotations

import math
import random
from dataclasses import dataclass
from enum import Enum, auto

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
    distance_to_square_boundary_along_direction,
    is_inside_square_boundary,
    move_toward,
    workspace_center,
)

from core.app.lab_escape.base import BaseAlgo
from core.app.lab_escape.eight_direction_motion import DIRECTIONS_8


# ============================================================
# Helper functions
# ============================================================

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
    top = pos.y - config.ice_radius_cm - config.z_min_cm
    bottom = config.z_max_cm - (pos.y + config.ice_radius_cm)

    if left < m:
        force += Vec2((m - left) / m, 0.0)

    if right < m:
        force += Vec2(-(m - right) / m, 0.0)

    if top < m:
        force += Vec2(0.0, (m - top) / m)

    if bottom < m:
        force += Vec2(0.0, -(m - bottom) / m)

    return force


def _random_far_position(
    hand_pos: Vec2 | None,
    config: WorldConfig,
    *,
    top_margin: float = 4.0,
    min_dist_extra: float = 8.0,
) -> Vec2:
    if hand_pos is None:
        return _center(config)

    min_dist = (
        config.hand_radius_cm + config.ice_radius_cm + min_dist_extra
    )

    for _ in range(120):
        p = Vec2(
            random.uniform(
                config.y_min_cm + config.ice_radius_cm,
                config.y_max_cm - config.ice_radius_cm,
            ),
            random.uniform(
                config.z_min_cm + config.ice_radius_cm + top_margin,
                config.z_max_cm - config.ice_radius_cm,
            ),
        )

        if distance(p, hand_pos) >= min_dist:
            return p

    corners = [
        Vec2(
            config.y_min_cm + config.ice_radius_cm,
            config.z_min_cm + config.ice_radius_cm + top_margin,
        ),
        Vec2(
            config.y_max_cm - config.ice_radius_cm,
            config.z_min_cm + config.ice_radius_cm + top_margin,
        ),
        Vec2(
            config.y_min_cm + config.ice_radius_cm,
            config.z_max_cm - config.ice_radius_cm,
        ),
        Vec2(
            config.y_max_cm - config.ice_radius_cm,
            config.z_max_cm - config.ice_radius_cm,
        ),
    ]

    return max(corners, key=lambda p: distance(p, hand_pos))


def _pick_safe_anchor(
    ice_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    *,
    prefer_center: bool,
) -> Vec2:
    margin = 2.5
    span_y = config.y_max_cm - config.y_min_cm
    span_z = config.z_max_cm - config.z_min_cm

    candidates: list[Vec2] = []

    xs = [
        config.y_min_cm + margin,
        config.y_min_cm + span_y * 0.25,
        config.y_min_cm + span_y * 0.5,
        config.y_min_cm + span_y * 0.75,
        config.y_max_cm - margin,
    ]

    zs = [
        config.z_min_cm + margin + 1.5,
        config.z_min_cm + span_z * 0.35,
        config.z_min_cm + span_z * 0.55,
        config.z_max_cm - margin,
    ]

    for x in xs:
        for z in zs:
            candidates.append(
                clamp_position_with_config(
                    Vec2(x, z),
                    config.ice_radius_cm,
                    config,
                )
            )

    for _ in range(10):
        candidates.append(
            Vec2(
                random.uniform(config.y_min_cm + margin, config.y_max_cm - margin),
                random.uniform(
                    config.z_min_cm + margin + 1.5,
                    config.z_max_cm - margin,
                ),
            )
        )

    best = candidates[0]
    best_score = -float("inf")
    center = _center(config)

    for p in candidates:
        wall_dist = distance_from_nearest_wall_with_config(
            p,
            config.ice_radius_cm,
            config,
        )

        score = 0.0
        score += wall_dist * 2.0
        score += min(distance(p, ice_pos), 26.0) * 0.4

        if danger_pos is not None:
            score += distance(p, danger_pos) * 2.0

        if prefer_center:
            score -= distance(p, center) * 0.2

        score += random.uniform(-2.0, 2.0)

        if score > best_score:
            best_score = score
            best = p

    return best


def _inside_workspace(pos: Vec2, config: WorldConfig) -> bool:
    return is_inside_square_boundary(
        pos,
        config.ice_radius_cm,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
    )


def _flat_escape_directions(
    current: Vec2,
    danger: Vec2,
    config: WorldConfig,
    step: float,
) -> tuple[list[Vec2], list[Vec2]]:
    toward_danger = _safe_normalized(danger - current)
    min_angle = config.flat_escape_min_angle_deg
    if distance(current, danger) < config.close_flat_escape_distance_cm:
        min_angle = config.close_flat_escape_min_angle_deg

    min_dot = math.cos(math.radians(min_angle))
    away_dirs: list[Vec2] = []
    valid_dirs: list[Vec2] = []

    for direction in DIRECTIONS_8:
        if direction.dot(toward_danger) < min_dot:
            away_dirs.append(direction)
            if _direction_has_escape_room(current, direction, config, step):
                valid_dirs.append(direction)

    return away_dirs, valid_dirs


def _direction_has_escape_room(
    current: Vec2,
    direction: Vec2,
    config: WorldConfig,
    step: float,
) -> bool:
    boundary_distance = distance_to_square_boundary_along_direction(
        current,
        direction,
        config.ice_radius_cm,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
    )

    return (
        boundary_distance > config.boundary_escape_distance_cm
        and _inside_workspace(current + direction * step, config)
    )


def _nearest_8dir(direction: Vec2) -> Vec2:
    return max(DIRECTIONS_8, key=lambda candidate: candidate.dot(direction))


def _random_far_landing(
    danger: Vec2 | None,
    config: WorldConfig,
) -> Vec2:
    if danger is None:
        return _center(config)

    low_x = config.y_min_cm + config.ice_radius_cm + config.landing_boundary_margin_cm
    high_x = config.y_max_cm - config.ice_radius_cm - config.landing_boundary_margin_cm
    low_y = config.z_min_cm + config.ice_radius_cm + config.landing_boundary_margin_cm
    high_y = config.z_max_cm - config.ice_radius_cm - config.landing_boundary_margin_cm

    if low_x > high_x or low_y > high_y:
        return _center(config)

    for _ in range(200):
        p = Vec2(
            random.uniform(low_x, high_x),
            random.uniform(low_y, high_y),
        )

        if distance(p, danger) > config.landing_min_distance_cm:
            return p

    candidates = [
        Vec2(low_x, low_y),
        Vec2(high_x, low_y),
        Vec2(low_x, high_y),
        Vec2(high_x, high_y),
    ]
    return max(candidates, key=lambda p: distance(p, danger))


def _blind_center_distance_anchor(ice_pos: Vec2, config: WorldConfig) -> Vec2:
    center = _center(config)
    direction = _safe_normalized(ice_pos - center)

    if direction.length() < 1e-6:
        direction = Vec2(1.0, 0.0)

    target = center + direction * config.blind_escape_center_distance_cm

    return clamp_position_to_square(
        target,
        config.ice_radius_cm,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
    )


# ============================================================
# IDLE variants
# ============================================================

class SmoothIdleTeaseAlgo(BaseAlgo):
    """
    smooth profile 的 IDLE：
    小幅 figure-eight 擺動。
    穩定、低調，不會太突然。
    """

    def __init__(
        self,
        amplitude_x: float = 3.5,
        amplitude_y: float = 1.2,
        frequency: float = 1.6,
        follow_speed: float = 75.0,
    ):
        self.amplitude_x = amplitude_x
        self.amplitude_y = amplitude_y
        self.frequency = frequency
        self.follow_speed = follow_speed

        self.anchor: Vec2 | None = None
        self.t = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.anchor = ice_pos
        self.t = 0.0

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.anchor is None:
            self.anchor = ice_pos

        self.t += dt
        phase = 2.0 * math.pi * self.frequency * self.t

        target = self.anchor + Vec2(
            math.sin(phase) * self.amplitude_x,
            math.sin(phase * 2.0) * self.amplitude_y,
        )

        target = clamp_position_with_config(target, config.ice_radius_cm, config)

        next_pos = move_toward(
            current=ice_pos,
            target=target,
            max_distance=self.follow_speed * dt,
        )

        return AlgoResult(
            pos=next_pos,
            color=IDLE_COLOR,
            border_color=IDLE_BORDER_COLOR,
            status="IDLE / smooth figure-eight tease",
        )


class PerformerIdleTeaseAlgo(BaseAlgo):
    """
    performer profile 的 IDLE：
    平時小晃動，但偶爾做一個小 fake dart。
    比較有表演感。
    """

    def __init__(
        self,
        wiggle_amp: float = 2.8,
        wiggle_freq: float = 2.2,
        dart_distance: float = 8.0,
        dart_speed: float = 120.0,
        dart_interval_min: float = 0.8,
        dart_interval_max: float = 1.8,
    ):
        self.wiggle_amp = wiggle_amp
        self.wiggle_freq = wiggle_freq
        self.dart_distance = dart_distance
        self.dart_speed = dart_speed
        self.dart_interval_min = dart_interval_min
        self.dart_interval_max = dart_interval_max

        self.anchor: Vec2 | None = None
        self.t = 0.0
        self.dart_timer = 0.0
        self.dart_target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.anchor = ice_pos
        self.t = 0.0
        self.dart_timer = random.uniform(self.dart_interval_min, self.dart_interval_max)
        self.dart_target = None

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.anchor is None:
            self.anchor = ice_pos

        self.t += dt
        self.dart_timer -= dt

        if self.dart_timer <= 0.0 and self.dart_target is None:
            angle = random.uniform(0.0, math.tau)
            self.dart_target = self.anchor + Vec2(
                math.cos(angle) * self.dart_distance,
                math.sin(angle) * self.dart_distance * 0.45,
            )
            self.dart_target = clamp_position_with_config(
                self.dart_target,
                config.ice_radius_cm,
                config,
            )

        if self.dart_target is not None:
            next_pos = move_toward(
                current=ice_pos,
                target=self.dart_target,
                max_distance=self.dart_speed * dt,
            )

            if distance(next_pos, self.dart_target) < 4.0:
                self.dart_target = None
                self.dart_timer = random.uniform(
                    self.dart_interval_min,
                    self.dart_interval_max,
                )

            return AlgoResult(
                pos=next_pos,
                color=IDLE_COLOR,
                border_color=IDLE_BORDER_COLOR,
                status="IDLE / performer fake dart",
            )

        phase = 2.0 * math.pi * self.wiggle_freq * self.t

        target = self.anchor + Vec2(
            math.sin(phase) * self.wiggle_amp,
            math.cos(phase * 1.5) * self.wiggle_amp * 0.25,
        )

        target = clamp_position_with_config(target, config.ice_radius_cm, config)

        next_pos = move_toward(
            current=ice_pos,
            target=target,
            max_distance=70.0 * dt,
        )

        return AlgoResult(
            pos=next_pos,
            color=IDLE_COLOR,
            border_color=IDLE_BORDER_COLOR,
            status="IDLE / performer wiggle",
        )


class SafeIdleGuardAlgo(BaseAlgo):
    """
    safe profile 的 IDLE：
    慢慢回到中間附近，做很小幅 hover。
    不追求挑釁，追求穩定與安全。
    """

    def __init__(
        self,
        hover_radius: float = 1.8,
        frequency: float = 1.0,
        move_speed: float = 42.0,
    ):
        self.hover_radius = hover_radius
        self.frequency = frequency
        self.move_speed = move_speed
        self.t = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.t = 0.0

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        self.t += dt

        center = _center(config)
        phase = 2.0 * math.pi * self.frequency * self.t

        target = center + Vec2(
            math.cos(phase) * self.hover_radius,
            math.sin(phase) * self.hover_radius * 0.6,
        )

        next_pos = move_toward(
            current=ice_pos,
            target=target,
            max_distance=self.move_speed * dt,
        )

        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        return AlgoResult(
            pos=next_pos,
            color=IDLE_COLOR,
            border_color=IDLE_BORDER_COLOR,
            status="IDLE / safe center hover",
        )


class TurkishStillIdleAlgo(BaseAlgo):
    """
    使用者指定版 IDLE：
    當目標手不動時，機械手臂完全停住。
    """

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        return AlgoResult(
            pos=ice_pos,
            color=IDLE_COLOR,
            border_color=IDLE_BORDER_COLOR,
            status="IDLE / hand stationary / hold position",
        )


# ============================================================
# NORMAL_ESCAPE variants
# ============================================================

class SmoothMomentumEscapeAlgo(BaseAlgo):
    """
    smooth profile 的 NORMAL：
    連續向量場逃離，不用 random 8-dir。
    優點是穩、不抖。
    """

    def __init__(
        self,
        move_speed: float = 46.0,
        smoothing: float = 0.18,
    ):
        self.move_speed = move_speed
        self.smoothing = smoothing
        self.direction = Vec2(1.0, 0.0)
        self.prev_hand_pos: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.direction = _safe_normalized(ice_pos - hand_pos) if hand_pos else Vec2(1.0, 0.0)
        self.prev_hand_pos = hand_pos

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if hand_pos is None:
            return AlgoResult(ice_pos, ACTIVE_COLOR, ACTIVE_BORDER_COLOR, "NORMAL / no hand")

        away = _safe_normalized(ice_pos - hand_pos)
        wall = _wall_repulsion(ice_pos, config, margin=14.0)
        center = _center_pull(ice_pos, config)

        prediction = _zero()
        if self.prev_hand_pos is not None:
            hand_motion = _safe_normalized(hand_pos - self.prev_hand_pos)
            prediction = _safe_normalized(away - hand_motion)

        desired = away * 1.8 + wall * 1.3 + center * 0.35 + prediction * 0.6
        desired_dir = _safe_normalized(desired)

        if desired_dir.length() < 1e-6:
            desired_dir = self.direction

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        self.prev_hand_pos = hand_pos

        return AlgoResult(
            pos=next_pos,
            color=ACTIVE_COLOR,
            border_color=ACTIVE_BORDER_COLOR,
            status="NORMAL / smooth momentum escape",
        )


class PerformerOrbitEscapeAlgo(BaseAlgo):
    """
    performer profile 的 NORMAL：
    混合遠離手 + tangent 繞手。
    看起來比較像在逗人，而不是只會後退。
    """

    def __init__(
        self,
        move_speed: float = 47.0,
        smoothing: float = 0.22,
        flip_interval: float = 1.2,
    ):
        self.move_speed = move_speed
        self.smoothing = smoothing
        self.flip_interval = flip_interval

        self.direction = Vec2(1.0, 0.0)
        self.orbit_sign = 1.0
        self.flip_timer = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.direction = Vec2(1.0, 0.0)
        self.orbit_sign = random.choice([-1.0, 1.0])
        self.flip_timer = random.uniform(self.flip_interval * 0.6, self.flip_interval * 1.3)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if hand_pos is None:
            return AlgoResult(ice_pos, ACTIVE_COLOR, ACTIVE_BORDER_COLOR, "NORMAL / no hand")

        self.flip_timer -= dt
        if self.flip_timer <= 0.0:
            self.orbit_sign *= -1.0
            self.flip_timer = random.uniform(self.flip_interval * 0.7, self.flip_interval * 1.4)

        away = _safe_normalized(ice_pos - hand_pos)
        tangent = Vec2(-away.y * self.orbit_sign, away.x * self.orbit_sign)

        wall_dist = distance_from_nearest_wall_with_config(
            ice_pos,
            config.ice_radius_cm,
            config,
        )

        wall_factor = min(1.0, max(0.0, wall_dist / config.boundary_exit_margin_cm))

        desired = (
            away * 0.9
            + tangent * 1.35 * wall_factor
            + _wall_repulsion(ice_pos, config, margin=15.0) * 1.5
            + _center_pull(ice_pos, config) * 0.45
        )

        desired_dir = _safe_normalized(desired)

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        return AlgoResult(
            pos=next_pos,
            color=ACTIVE_COLOR,
            border_color=ACTIVE_BORDER_COLOR,
            status=f"NORMAL / performer orbit / sign={self.orbit_sign:+.0f}",
        )


class SafeAnchorEscapeAlgo(BaseAlgo):
    """
    safe profile 的 NORMAL：
    選一個安全 anchor，穩定移動過去。
    最適合未來接真機，因為 target 很明確。
    """

    def __init__(
        self,
        move_speed: float = 54.0,
        reach_radius: float = 3.0,
        retarget_time: float = 0.8,
    ):
        self.move_speed = move_speed
        self.reach_radius = reach_radius
        self.retarget_time = retarget_time

        self.target: Vec2 | None = None
        self.timer = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.target = None
        self.timer = 0.0

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if hand_pos is None:
            return AlgoResult(ice_pos, ACTIVE_COLOR, ACTIVE_BORDER_COLOR, "NORMAL / no hand")

        self.timer -= dt

        need_new = (
            self.target is None
            or distance(ice_pos, self.target) < self.reach_radius
            or self.timer <= 0.0
            or distance(self.target, hand_pos)
            < config.hand_radius_cm + config.ice_radius_cm + 12.0
        )

        if need_new:
            self.target = _pick_safe_anchor(
                ice_pos=ice_pos,
                danger_pos=hand_pos,
                config=config,
                prefer_center=True,
            )
            self.timer = self.retarget_time

        next_pos = move_toward(
            current=ice_pos,
            target=self.target,
            max_distance=self.move_speed * dt,
        )

        return AlgoResult(
            pos=next_pos,
            color=ACTIVE_COLOR,
            border_color=ACTIVE_BORDER_COLOR,
            status=f"NORMAL / safe anchor target=({self.target.x:.0f}, {self.target.y:.0f})",
        )


class TurkishFlatEscapeAlgo(BaseAlgo):
    """
    平閃：
    每 0.2 秒用八方向重新判斷一次。
    合格方向為與「自己 -> 目標」連線夾角大於 90 度者。
    若原方向仍合格且不出界，維持原方向；否則從合格方向中隨機挑選。
    """

    def __init__(self, move_speed: float | None = None):
        self.move_speed = move_speed
        self.current_direction: Vec2 | None = None
        self.forced_directions: list[Vec2] = []
        self.preferred_directions: list[Vec2] = []
        self.decision_timer = 0.0
        self.last_valid_count = 0

    def set_forced_direction(self, direction: Vec2 | None):
        if direction is None:
            self.forced_directions = []
            return

        self.forced_directions = [_nearest_8dir(direction.normalized())]

    def set_forced_directions(self, directions: list[Vec2]):
        self.forced_directions = [
            _nearest_8dir(direction.normalized())
            for direction in directions
            if direction.length() > 1e-6
        ]

    def set_preferred_directions(self, directions: list[Vec2]):
        self.preferred_directions = [
            _nearest_8dir(direction.normalized())
            for direction in directions
            if direction.length() > 1e-6
        ]

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.current_direction = None
        self.decision_timer = 0.0
        self.last_valid_count = 0

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if hand_pos is None:
            return AlgoResult(
                ice_pos,
                ACTIVE_COLOR,
                ACTIVE_BORDER_COLOR,
                "NORMAL / no observed hand",
            )

        move_speed = self.move_speed or config.flat_escape_speed_cm_s
        step = move_speed * dt
        self.decision_timer -= dt

        if self.current_direction is None or self.decision_timer <= 0.0:
            away_dirs, valid_dirs = _flat_escape_directions(
                current=ice_pos,
                danger=hand_pos,
                config=config,
                step=move_speed * config.decision_interval_s,
            )
            self.last_valid_count = len(valid_dirs)

            if self.forced_directions:
                forced_valid_dirs = [
                    direction
                    for direction in self.forced_directions
                    if _direction_has_escape_room(
                        ice_pos,
                        direction,
                        config,
                        move_speed * config.decision_interval_s,
                    )
                ]

                if not forced_valid_dirs:
                    self.current_direction = None
                    return AlgoResult(
                        ice_pos,
                        ACTIVE_COLOR,
                        ACTIVE_BORDER_COLOR,
                        "NORMAL / direct approach blocked",
                    )

                best_distance = max(
                    distance(ice_pos + direction * step, hand_pos)
                    for direction in forced_valid_dirs
                )
                best_forced_dirs = [
                    direction
                    for direction in forced_valid_dirs
                    if abs(distance(ice_pos + direction * step, hand_pos) - best_distance)
                    < 1e-6
                ]

                if self.current_direction in best_forced_dirs:
                    chosen = self.current_direction
                else:
                    chosen = random.choice(best_forced_dirs)
            elif self.preferred_directions:
                preferred_valid_dirs = [
                    direction
                    for direction in self.preferred_directions
                    if direction in valid_dirs
                ]

                if self.current_direction in preferred_valid_dirs:
                    chosen = self.current_direction
                elif preferred_valid_dirs:
                    chosen = random.choice(preferred_valid_dirs)
                elif not valid_dirs:
                    self.current_direction = None
                    return AlgoResult(
                        ice_pos,
                        ACTIVE_COLOR,
                        ACTIVE_BORDER_COLOR,
                        f"NORMAL / flat escape blocked / away={len(away_dirs)}",
                    )
                elif self.current_direction in valid_dirs:
                    chosen = self.current_direction
                else:
                    chosen = random.choice(valid_dirs)
            elif not valid_dirs:
                self.current_direction = None
                return AlgoResult(
                    ice_pos,
                    ACTIVE_COLOR,
                    ACTIVE_BORDER_COLOR,
                    f"NORMAL / flat escape blocked / away={len(away_dirs)}",
                )
            elif self.current_direction in valid_dirs:
                chosen = self.current_direction
            else:
                chosen = random.choice(valid_dirs)

            self.current_direction = chosen
            self.decision_timer = config.decision_interval_s

        assert self.current_direction is not None
        next_pos = ice_pos + self.current_direction * step
        next_pos = clamp_position_to_square(
            next_pos,
            config.ice_radius_cm,
            config.y_min_cm,
            config.y_max_cm,
            config.z_min_cm,
            config.z_max_cm,
        )

        return AlgoResult(
            next_pos,
            ACTIVE_COLOR,
            ACTIVE_BORDER_COLOR,
            (
                f"NORMAL / flat 8-dir / valid={self.last_valid_count} / "
                f"forced={bool(self.forced_directions)} / "
                f"preferred={bool(self.preferred_directions)} / "
                f"next decision={self.decision_timer:.2f}s"
            ),
        )


# ============================================================
# BOUNDARY_ESCAPE variants
# ============================================================

class SmoothBoundaryCenterPullAlgo(BaseAlgo):
    """
    smooth profile 的 BOUNDARY：
    平滑往中心拉回，同時遠離手。
    """

    def __init__(self, move_speed: float = 52.0, smoothing: float = 0.25):
        self.move_speed = move_speed
        self.smoothing = smoothing
        self.direction = Vec2(0.0, 1.0)

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.direction = _center_pull(ice_pos, config)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        center_dir = _center_pull(ice_pos, config)
        wall = _wall_repulsion(ice_pos, config, margin=18.0)

        away = _zero()
        if hand_pos is not None:
            away = _safe_normalized(ice_pos - hand_pos)

        desired = center_dir * 1.5 + wall * 2.0 + away * 0.6
        desired_dir = _safe_normalized(desired)

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        return AlgoResult(
            pos=next_pos,
            color=BOUNDARY_COLOR,
            border_color=BOUNDARY_BORDER_COLOR,
            status="BOUNDARY / smooth center pull",
        )


class PerformerWallSlideAlgo(BaseAlgo):
    """
    performer profile 的 BOUNDARY：
    沿著牆邊滑一下再往內切，動作比較有表演感。
    """

    def __init__(self, move_speed: float = 56.0, slide_weight: float = 1.0):
        self.move_speed = move_speed
        self.slide_weight = slide_weight
        self.slide_sign = 1.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.slide_sign = random.choice([-1.0, 1.0])

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        left = ice_pos.x - config.ice_radius_cm - config.y_min_cm
        right = config.y_max_cm - (ice_pos.x + config.ice_radius_cm)
        top = ice_pos.y - config.ice_radius_cm - config.z_min_cm
        bottom = config.z_max_cm - (ice_pos.y + config.ice_radius_cm)

        nearest = min(left, right, top, bottom)

        # 根據最近的牆決定 slide direction。
        if nearest == left:
            slide = Vec2(0.0, self.slide_sign)
            inward = Vec2(1.0, 0.0)
        elif nearest == right:
            slide = Vec2(0.0, self.slide_sign)
            inward = Vec2(-1.0, 0.0)
        elif nearest == top:
            slide = Vec2(self.slide_sign, 0.0)
            inward = Vec2(0.0, 1.0)
        else:
            slide = Vec2(self.slide_sign, 0.0)
            inward = Vec2(0.0, -1.0)

        away = _zero()
        if hand_pos is not None:
            away = _safe_normalized(ice_pos - hand_pos)

        desired = inward * 1.7 + slide * self.slide_weight + away * 0.6
        direction = _safe_normalized(desired)

        next_pos = ice_pos + direction * self.move_speed * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        return AlgoResult(
            pos=next_pos,
            color=BOUNDARY_COLOR,
            border_color=BOUNDARY_BORDER_COLOR,
            status="BOUNDARY / performer wall slide",
        )


class SafeBoundaryAnchorAlgo(BaseAlgo):
    """
    safe profile 的 BOUNDARY：
    不做花式動作，直接找安全 anchor，穩定離開邊界。
    """

    def __init__(self, move_speed: float = 62.0):
        self.move_speed = move_speed
        self.target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.target = None

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.target is None or distance(ice_pos, self.target) < 2.5:
            self.target = _pick_safe_anchor(
                ice_pos=ice_pos,
                danger_pos=hand_pos,
                config=config,
                prefer_center=True,
            )

        next_pos = move_toward(
            current=ice_pos,
            target=self.target,
            max_distance=self.move_speed * dt,
        )

        return AlgoResult(
            pos=next_pos,
            color=BOUNDARY_COLOR,
            border_color=BOUNDARY_BORDER_COLOR,
            status=f"BOUNDARY / safe anchor target=({self.target.x:.0f}, {self.target.y:.0f})",
        )


class TurkishBoundaryVanishLandingAlgo(BaseAlgo):
    """
    邊界閃：
    平閃的合格方向太靠近邊界時，先離開平面，
    再降落到距離目標大於設定安全距離的點。
    """

    def __init__(self, vanish_time: float = 0.25):
        self.vanish_time = vanish_time
        self.timer = 0.0
        self.target: Vec2 | None = None
        self.is_active = False

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.timer = self.vanish_time
        self.target = _random_far_landing(hand_pos or last_hand_pos, config)
        self.is_active = True

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.target is None:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        self.timer -= dt
        if self.timer > 0.0:
            return AlgoResult(
                ice_pos,
                BOUNDARY_COLOR,
                BOUNDARY_BORDER_COLOR,
                f"BOUNDARY / vanished / landing in {self.timer:.2f}s",
                visible=False,
                planned_landing=self.target,
            )

        self.is_active = False
        assert self.target is not None

        return AlgoResult(
            self.target,
            BOUNDARY_COLOR,
            BOUNDARY_BORDER_COLOR,
            f"BOUNDARY / landed >{config.landing_min_distance_cm:.0f}cm from hand",
        )


# ============================================================
# CLOSE_RETREAT variants
# ============================================================

class RetreatPhase(Enum):
    UP = auto()
    WAIT = auto()
    DOWN = auto()


class SmoothVerticalRetreatAlgo(BaseAlgo):
    """
    smooth profile 的 CLOSE_RETREAT：
    快速垂直上抽，停一下，再垂直/斜向放下。
    """

    def __init__(
        self,
        up_speed: float = 180.0,
        down_speed: float = 150.0,
        wait_time: float = 0.25,
    ):
        self.up_speed = up_speed
        self.down_speed = down_speed
        self.wait_time = wait_time

        self.is_active = False
        self.phase = RetreatPhase.UP
        self.timer = 0.0
        self.up_target: Vec2 | None = None
        self.down_target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.is_active = True
        self.phase = RetreatPhase.UP
        self.timer = 0.0
        self.up_target = Vec2(
            clamp(
                ice_pos.x,
                config.y_min_cm + config.ice_radius_cm,
                config.y_max_cm - config.ice_radius_cm,
            ),
            config.z_max_cm - config.ice_radius_cm,
        )
        self.down_target = None

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if not self.is_active:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        if self.phase == RetreatPhase.UP:
            assert self.up_target is not None

            next_pos = move_toward(ice_pos, self.up_target, self.up_speed * dt)

            if distance(next_pos, self.up_target) < 3.0:
                next_pos = self.up_target
                self.phase = RetreatPhase.WAIT
                self.timer = self.wait_time

            return AlgoResult(next_pos, RETREAT_COLOR, RETREAT_BORDER_COLOR, "CLOSE / smooth up")

        if self.phase == RetreatPhase.WAIT:
            self.timer -= dt

            if self.timer <= 0.0:
                self.down_target = _random_far_position(
                    hand_pos or last_hand_pos,
                    config,
                    top_margin=4.0,
                    min_dist_extra=8.0,
                )
                self.phase = RetreatPhase.DOWN

            return AlgoResult(
                ice_pos,
                RETREAT_COLOR,
                RETREAT_BORDER_COLOR,
                f"CLOSE / smooth wait {max(self.timer, 0.0):.2f}s",
            )

        assert self.down_target is not None

        next_pos = move_toward(ice_pos, self.down_target, self.down_speed * dt)

        if distance(next_pos, self.down_target) < 3.0:
            next_pos = self.down_target
            self.is_active = False

            return AlgoResult(
                next_pos,
                ACTIVE_COLOR,
                ACTIVE_BORDER_COLOR,
                "CLOSE / smooth finished",
            )

        return AlgoResult(next_pos, RETREAT_COLOR, RETREAT_BORDER_COLOR, "CLOSE / smooth down")


class PerformerFakeRetreatAlgo(SmoothVerticalRetreatAlgo):
    """
    performer profile 的 CLOSE_RETREAT：
    上抽時不是直上，而是斜上方 fake out。
    放下時也選比較遠的位置，看起來比較像被快速抽走再放回。
    """

    def __init__(self):
        super().__init__(
            up_speed=220.0,
            down_speed=180.0,
            wait_time=0.18,
        )
        self.side_offset = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.is_active = True
        self.phase = RetreatPhase.UP
        self.timer = 0.0

        self.side_offset = random.choice([-1.0, 1.0]) * random.uniform(2.0, 4.0)

        x = clamp(
            ice_pos.x + self.side_offset,
            config.y_min_cm + config.ice_radius_cm,
            config.y_max_cm - config.ice_radius_cm,
        )

        self.up_target = Vec2(x, config.z_max_cm - config.ice_radius_cm)
        self.down_target = None

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if not self.is_active:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        if self.phase == RetreatPhase.WAIT and self.down_target is None:
            pass

        result = super().update(ice_pos, hand_pos, last_hand_pos, config, dt)

        if self.phase == RetreatPhase.DOWN and self.down_target is not None:
            result.status = "CLOSE / performer fake down"

        if self.phase == RetreatPhase.UP:
            result.status = "CLOSE / performer diagonal fake up"

        return result


class SafeRetreatToAnchorAlgo(SmoothVerticalRetreatAlgo):
    """
    safe profile 的 CLOSE_RETREAT：
    上抽後固定往安全 anchor 放回，不追求花式。
    """

    def __init__(self):
        super().__init__(
            up_speed=190.0,
            down_speed=140.0,
            wait_time=0.3,
        )

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if not self.is_active:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        if self.phase == RetreatPhase.UP:
            return super().update(ice_pos, hand_pos, last_hand_pos, config, dt)

        if self.phase == RetreatPhase.WAIT:
            self.timer -= dt

            if self.timer <= 0.0:
                self.down_target = _pick_safe_anchor(
                    ice_pos=ice_pos,
                    danger_pos=hand_pos or last_hand_pos,
                    config=config,
                    prefer_center=True,
                )
                self.phase = RetreatPhase.DOWN

            return AlgoResult(
                ice_pos,
                RETREAT_COLOR,
                RETREAT_BORDER_COLOR,
                f"CLOSE / safe wait {max(self.timer, 0.0):.2f}s",
            )

        return super().update(ice_pos, hand_pos, last_hand_pos, config, dt)


# ============================================================
# BLIND_SPOT_ESCAPE variants
# ============================================================

class SmoothBlindVectorEscapeAlgo(BaseAlgo):
    """
    smooth profile 的 BLIND：
    根據 last_hand_pos 或 blind zone center 連續遠離。
    """

    def __init__(self, move_speed: float = 43.0, smoothing: float = 0.2):
        self.move_speed = move_speed
        self.smoothing = smoothing
        self.direction = Vec2(1.0, 0.0)

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        danger = last_hand_pos or config.blind_zone.center()
        self.direction = _safe_normalized(ice_pos - danger)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        danger = last_hand_pos or config.blind_zone.center()

        desired = (
            _safe_normalized(ice_pos - danger) * 1.5
            + _wall_repulsion(ice_pos, config, margin=14.0) * 1.0
            + _center_pull(ice_pos, config) * 0.2
        )

        desired_dir = _safe_normalized(desired)

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        return AlgoResult(
            next_pos,
            BLIND_COLOR,
            BLIND_BORDER_COLOR,
            "BLIND / smooth vector escape",
        )


class PerformerBlindBaitAlgo(BaseAlgo):
    """
    performer profile 的 BLIND：
    手進入盲區後，不只是逃，而是往盲區旁邊切線滑開。
    像是故意從盲區邊緣閃掉。
    """

    def __init__(self, move_speed: float = 50.0):
        self.move_speed = move_speed
        self.sign = 1.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.sign = random.choice([-1.0, 1.0])

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        danger = last_hand_pos or config.blind_zone.center()
        away = _safe_normalized(ice_pos - danger)
        tangent = Vec2(-away.y * self.sign, away.x * self.sign)

        desired = (
            away * 0.9
            + tangent * 1.2
            + _wall_repulsion(ice_pos, config, margin=15.0) * 1.2
        )

        direction = _safe_normalized(desired)
        next_pos = ice_pos + direction * self.move_speed * dt
        next_pos = clamp_position_with_config(next_pos, config.ice_radius_cm, config)

        return AlgoResult(
            next_pos,
            BLIND_COLOR,
            BLIND_BORDER_COLOR,
            "BLIND / performer blind-zone bait",
        )


class SafeBlindAnchorAlgo(BaseAlgo):
    """
    safe profile 的 BLIND：
    直接移動到遠離 blind zone 的安全 anchor。
    """

    def __init__(self, move_speed: float = 52.0):
        self.move_speed = move_speed
        self.target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        danger = last_hand_pos or config.blind_zone.center()
        self.target = _pick_safe_anchor(
            ice_pos=ice_pos,
            danger_pos=danger,
            config=config,
            prefer_center=True,
        )

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.target is None:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        assert self.target is not None

        next_pos = move_toward(
            current=ice_pos,
            target=self.target,
            max_distance=self.move_speed * dt,
        )

        return AlgoResult(
            next_pos,
            BLIND_COLOR,
            BLIND_BORDER_COLOR,
            f"BLIND / safe anchor target=({self.target.x:.0f}, {self.target.y:.0f})",
        )


class TurkishBlindEdgeAlgo(BaseAlgo):
    """
    盲區：
    偵測到目標進入盲區後，根據當下位置遠離圓心，
    主動移到離工作區中心指定距離的位置。
    """

    def __init__(self, move_speed: float | None = None):
        self.move_speed = move_speed
        self.target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.target = _blind_center_distance_anchor(ice_pos, config)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.target is None:
            self.on_enter(ice_pos, hand_pos, last_hand_pos, config)

        assert self.target is not None

        move_speed = self.move_speed or config.flat_escape_speed_cm_s

        next_pos = move_toward(
            current=ice_pos,
            target=self.target,
            max_distance=move_speed * dt,
        )

        return AlgoResult(
            next_pos,
            BLIND_COLOR,
            BLIND_BORDER_COLOR,
            (
                f"BLIND / move {config.blind_escape_center_distance_cm:.0f}cm "
                f"from center ({self.target.x:.0f}, {self.target.y:.0f})"
            ),
        )


# ============================================================
# Profile factory
# ============================================================

def create_algorithm_profile(profile: str):
    """
    回傳完整 state -> algorithm 對應。

    可選 profile：
    - "turkish_spec"
    - "smooth"
    - "performer"
    - "safe"
    """

    if profile == "turkish_spec":
        return {
            RobotState.IDLE: TurkishStillIdleAlgo(),
            RobotState.NORMAL_ESCAPE: TurkishFlatEscapeAlgo(),
            RobotState.BOUNDARY_ESCAPE: TurkishBoundaryVanishLandingAlgo(),
            RobotState.CLOSE_RETREAT: SafeRetreatToAnchorAlgo(),
            RobotState.BLIND_SPOT_ESCAPE: TurkishBlindEdgeAlgo(),
        }

    if profile == "smooth":
        return {
            RobotState.IDLE: SmoothIdleTeaseAlgo(),
            RobotState.NORMAL_ESCAPE: SmoothMomentumEscapeAlgo(),
            RobotState.BOUNDARY_ESCAPE: SmoothBoundaryCenterPullAlgo(),
            RobotState.CLOSE_RETREAT: SmoothVerticalRetreatAlgo(),
            RobotState.BLIND_SPOT_ESCAPE: SmoothBlindVectorEscapeAlgo(),
        }

    if profile == "performer":
        return {
            RobotState.IDLE: PerformerIdleTeaseAlgo(),
            RobotState.NORMAL_ESCAPE: PerformerOrbitEscapeAlgo(),
            RobotState.BOUNDARY_ESCAPE: PerformerWallSlideAlgo(),
            RobotState.CLOSE_RETREAT: PerformerFakeRetreatAlgo(),
            RobotState.BLIND_SPOT_ESCAPE: PerformerBlindBaitAlgo(),
        }

    if profile == "safe":
        return {
            RobotState.IDLE: SafeIdleGuardAlgo(),
            RobotState.NORMAL_ESCAPE: SafeAnchorEscapeAlgo(),
            RobotState.BOUNDARY_ESCAPE: SafeBoundaryAnchorAlgo(),
            RobotState.CLOSE_RETREAT: SafeRetreatToAnchorAlgo(),
            RobotState.BLIND_SPOT_ESCAPE: SafeBlindAnchorAlgo(),
        }

    raise ValueError(f"Unknown algorithm profile: {profile}")
