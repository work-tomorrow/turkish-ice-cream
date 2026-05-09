from __future__ import annotations

import math
import random
from dataclasses import dataclass
from enum import Enum, auto

from common import (
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
    clamp_position,
    distance,
    distance_from_nearest_wall,
    move_toward,
)

from algorithms import BaseAlgo


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
    return Vec2(config.width / 2.0, config.height / 2.0)


def _center_pull(pos: Vec2, config: WorldConfig) -> Vec2:
    return _safe_normalized(_center(config) - pos)


def _wall_repulsion(pos: Vec2, config: WorldConfig, margin: float = 130.0) -> Vec2:
    force = Vec2(0.0, 0.0)

    left = pos.x - config.ice_radius
    right = config.width - (pos.x + config.ice_radius)
    top = pos.y - config.ice_radius
    bottom = config.height - (pos.y + config.ice_radius)

    if left < margin:
        force += Vec2((margin - left) / margin, 0.0)

    if right < margin:
        force += Vec2(-(margin - right) / margin, 0.0)

    if top < margin:
        force += Vec2(0.0, (margin - top) / margin)

    if bottom < margin:
        force += Vec2(0.0, -(margin - bottom) / margin)

    return force


def _perp(v: Vec2, sign: float = 1.0) -> Vec2:
    return Vec2(-v.y * sign, v.x * sign)


def _clamp_length(v: Vec2, max_length: float) -> Vec2:
    length = v.length()
    if length <= max_length or length < 1e-6:
        return v
    return v.normalized() * max_length


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


def _move_angle_toward(current: float, target: float, max_delta: float) -> float:
    delta = _wrap_angle(target - current)
    if abs(delta) <= max_delta:
        return _wrap_angle(target)
    return _wrap_angle(current + max_delta * (1.0 if delta > 0.0 else -1.0))


def _angle_to_vec(angle: float) -> Vec2:
    return Vec2(math.cos(angle), math.sin(angle))


def _distance_point_to_segment(p: Vec2, a: Vec2, b: Vec2) -> float:
    ab = b - a
    ab_len_sq = ab.dot(ab)
    if ab_len_sq < 1e-6:
        return distance(p, a)

    t = clamp((p - a).dot(ab) / ab_len_sq, 0.0, 1.0)
    closest = a + ab * t
    return distance(p, closest)


def _point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool:
    d1 = _triangle_sign(p, a, b)
    d2 = _triangle_sign(p, b, c)
    d3 = _triangle_sign(p, c, a)

    has_neg = d1 < 0.0 or d2 < 0.0 or d3 < 0.0
    has_pos = d1 > 0.0 or d2 > 0.0 or d3 > 0.0

    return not (has_neg and has_pos)


def _triangle_sign(p1: Vec2, p2: Vec2, p3: Vec2) -> float:
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)


def _cone_points(center: Vec2, angle: float, config: WorldConfig) -> tuple[Vec2, Vec2, Vec2]:
    outward = _angle_to_vec(angle)
    tangent = Vec2(-outward.y, outward.x)

    base_center = center + outward * config.ice_radius
    half_width = config.cone_base_width * 0.5

    base_a = base_center + tangent * half_width
    base_b = base_center - tangent * half_width
    tip = center + outward * (config.ice_radius + config.cone_length)

    return base_a, base_b, tip


def _random_far_position(
    hand_pos: Vec2 | None,
    config: WorldConfig,
    *,
    top_margin: float = 90.0,
    min_dist_extra: float = 200.0,
) -> Vec2:
    if hand_pos is None:
        return _center(config)

    min_dist = config.hand_radius + config.ice_radius + min_dist_extra

    for _ in range(120):
        p = Vec2(
            random.uniform(config.ice_radius, config.width - config.ice_radius),
            random.uniform(
                config.ice_radius + top_margin,
                config.height - config.ice_radius,
            ),
        )

        if distance(p, hand_pos) >= min_dist:
            return p

    corners = [
        Vec2(config.ice_radius, config.ice_radius + top_margin),
        Vec2(config.width - config.ice_radius, config.ice_radius + top_margin),
        Vec2(config.ice_radius, config.height - config.ice_radius),
        Vec2(config.width - config.ice_radius, config.height - config.ice_radius),
    ]

    return max(corners, key=lambda p: distance(p, hand_pos))


def _pick_safe_anchor(
    ice_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    *,
    prefer_center: bool,
) -> Vec2:
    margin = 95.0

    candidates: list[Vec2] = []

    xs = [
        margin,
        config.width * 0.25,
        config.width * 0.5,
        config.width * 0.75,
        config.width - margin,
    ]

    ys = [
        margin + 40.0,
        config.height * 0.35,
        config.height * 0.55,
        config.height - margin,
    ]

    for x in xs:
        for y in ys:
            candidates.append(
                clamp_position(
                    Vec2(x, y),
                    config.ice_radius,
                    config.width,
                    config.height,
                )
            )

    for _ in range(10):
        candidates.append(
            Vec2(
                random.uniform(margin, config.width - margin),
                random.uniform(margin + 40.0, config.height - margin),
            )
        )

    best = candidates[0]
    best_score = -float("inf")
    center = _center(config)

    for p in candidates:
        wall_dist = distance_from_nearest_wall(
            p,
            config.ice_radius,
            config.width,
            config.height,
        )

        score = 0.0
        score += wall_dist * 2.0
        score += min(distance(p, ice_pos), 260.0) * 0.4

        if danger_pos is not None:
            score += distance(p, danger_pos) * 2.0

        if prefer_center:
            score -= distance(p, center) * 0.2

        score += random.uniform(-20.0, 20.0)

        if score > best_score:
            best_score = score
            best = p

    return best


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
        amplitude_x: float = 35.0,
        amplitude_y: float = 12.0,
        frequency: float = 1.6,
        follow_speed: float = 750.0,
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

        target = clamp_position(target, config.ice_radius, config.width, config.height)

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
        wiggle_amp: float = 28.0,
        wiggle_freq: float = 2.2,
        dart_distance: float = 80.0,
        dart_speed: float = 1200.0,
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
            self.dart_target = clamp_position(
                self.dart_target,
                config.ice_radius,
                config.width,
                config.height,
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

        target = clamp_position(target, config.ice_radius, config.width, config.height)

        next_pos = move_toward(
            current=ice_pos,
            target=target,
            max_distance=700.0 * dt,
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
        hover_radius: float = 18.0,
        frequency: float = 1.0,
        move_speed: float = 420.0,
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

        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

        return AlgoResult(
            pos=next_pos,
            color=IDLE_COLOR,
            border_color=IDLE_BORDER_COLOR,
            status="IDLE / safe center hover",
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
        move_speed: float = 460.0,
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
        wall = _wall_repulsion(ice_pos, config, margin=140.0)
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
        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

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
        move_speed: float = 470.0,
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

        wall_dist = distance_from_nearest_wall(
            ice_pos,
            config.ice_radius,
            config.width,
            config.height,
        )

        wall_factor = min(1.0, max(0.0, wall_dist / config.boundary_exit_margin))

        desired = (
            away * 0.9
            + tangent * 1.35 * wall_factor
            + _wall_repulsion(ice_pos, config, margin=150.0) * 1.5
            + _center_pull(ice_pos, config) * 0.45
        )

        desired_dir = _safe_normalized(desired)

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

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
        move_speed: float = 540.0,
        reach_radius: float = 30.0,
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
            or distance(self.target, hand_pos) < config.hand_radius + config.ice_radius + 120.0
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


# ============================================================
# BOUNDARY_ESCAPE variants
# ============================================================

class SmoothBoundaryCenterPullAlgo(BaseAlgo):
    """
    smooth profile 的 BOUNDARY：
    平滑往中心拉回，同時遠離手。
    """

    def __init__(self, move_speed: float = 520.0, smoothing: float = 0.25):
        self.move_speed = move_speed
        self.smoothing = smoothing
        self.direction = Vec2(0.0, 1.0)

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.direction = _center_pull(ice_pos, config)

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        center_dir = _center_pull(ice_pos, config)
        wall = _wall_repulsion(ice_pos, config, margin=180.0)

        away = _zero()
        if hand_pos is not None:
            away = _safe_normalized(ice_pos - hand_pos)

        desired = center_dir * 1.5 + wall * 2.0 + away * 0.6
        desired_dir = _safe_normalized(desired)

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

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

    def __init__(self, move_speed: float = 560.0, slide_weight: float = 1.0):
        self.move_speed = move_speed
        self.slide_weight = slide_weight
        self.slide_sign = 1.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.slide_sign = random.choice([-1.0, 1.0])

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        left = ice_pos.x - config.ice_radius
        right = config.width - (ice_pos.x + config.ice_radius)
        top = ice_pos.y - config.ice_radius
        bottom = config.height - (ice_pos.y + config.ice_radius)

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
        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

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

    def __init__(self, move_speed: float = 620.0):
        self.move_speed = move_speed
        self.target: Vec2 | None = None

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.target = None

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        if self.target is None or distance(ice_pos, self.target) < 25.0:
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
        up_speed: float = 1800.0,
        down_speed: float = 1500.0,
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
            clamp(ice_pos.x, config.ice_radius, config.width - config.ice_radius),
            config.ice_radius,
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
                    top_margin=90.0,
                    min_dist_extra=210.0,
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
            up_speed=2200.0,
            down_speed=1800.0,
            wait_time=0.18,
        )
        self.side_offset = 0.0

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        self.is_active = True
        self.phase = RetreatPhase.UP
        self.timer = 0.0

        self.side_offset = random.choice([-1.0, 1.0]) * random.uniform(80.0, 160.0)

        x = clamp(
            ice_pos.x + self.side_offset,
            config.ice_radius,
            config.width - config.ice_radius,
        )

        self.up_target = Vec2(x, config.ice_radius)
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
            up_speed=1900.0,
            down_speed=1400.0,
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

    def __init__(self, move_speed: float = 430.0, smoothing: float = 0.2):
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
            + _wall_repulsion(ice_pos, config, margin=140.0) * 1.0
            + _center_pull(ice_pos, config) * 0.2
        )

        desired_dir = _safe_normalized(desired)

        self.direction = _safe_normalized(
            _lerp_vec(self.direction, desired_dir, self.smoothing)
        )

        next_pos = ice_pos + self.direction * self.move_speed * dt
        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

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

    def __init__(self, move_speed: float = 500.0):
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
            + _wall_repulsion(ice_pos, config, margin=150.0) * 1.2
        )

        direction = _safe_normalized(desired)
        next_pos = ice_pos + direction * self.move_speed * dt
        next_pos = clamp_position(next_pos, config.ice_radius, config.width, config.height)

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

    def __init__(self, move_speed: float = 520.0):
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


# ============================================================
# PREDICTIVE_SAFE profile
# ============================================================

class PredictiveSafetyPlanner:
    """
    Shared low-level planner for the predictive_safe profile.

    It keeps one motor direction command across state changes.  Every control
    tick it scores short predicted paths, but can change only the horizontal
    or the vertical axis of the command.  The motion speed is treated as a
    near-constant motor property with a small per-command delta.
    """

    def __init__(
        self,
        hand_velocity_smoothing: float = 0.35,
        boundary_guard: float = 10.0,
    ):
        self.hand_velocity_smoothing = hand_velocity_smoothing
        self.boundary_guard = boundary_guard

        self.velocity = Vec2(0.0, 0.0)
        self.command_direction = Vec2(0.0, 0.0)
        self.actual_speed = 0.0
        self.prev_hand_pos: Vec2 | None = None
        self.hand_velocity = Vec2(0.0, 0.0)
        self.hand_sample_elapsed = 0.0
        self.orbit_sign = 1.0
        self.command_timer = 0.0
        self.command_mode: str | None = None
        self.cone_angle = -math.pi / 2.0
        self.cone_target_angle = self.cone_angle
        self.cone_actual_speed = 0.0

    def step(
        self,
        *,
        mode: str,
        ice_pos: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
        idle_target: Vec2 | None = None,
    ) -> tuple[Vec2, str]:
        frame_dt = max(1.0 / 240.0, min(dt, 1.0 / 20.0))
        control_dt = max(config.control_period, frame_dt)

        self._update_hand_velocity(danger_pos, frame_dt, config)
        self.command_timer -= frame_dt

        mode_changed = self.command_mode != mode
        should_replan = self.command_timer <= 0.0 or mode_changed

        if should_replan:
            self.cone_target_angle = self._plan_cone_angle(
                ice_pos=ice_pos,
                danger_pos=danger_pos,
                config=config,
            )
            self.cone_actual_speed = self._sample_cone_rotation_speed(config)

            self.command_direction = self._plan_direction(
                mode=mode,
                ice_pos=ice_pos,
                danger_pos=danger_pos,
                config=config,
                plan_dt=control_dt,
                idle_target=idle_target,
            )
            self.actual_speed = self._sample_motor_speed(config)
            self.velocity = self.command_direction * self.actual_speed

            if mode_changed:
                self.command_timer = config.control_period
            else:
                self.command_timer += config.control_period
                while self.command_timer <= 0.0:
                    self.command_timer += config.control_period
            self.command_mode = mode

        self.cone_angle = _move_angle_toward(
            self.cone_angle,
            self.cone_target_angle,
            self.cone_actual_speed * frame_dt,
        )

        next_pos = ice_pos + self.velocity * frame_dt
        next_pos = clamp_position(
            next_pos,
            config.ice_radius + self.boundary_guard,
            config.width,
            config.height,
        )
        self._remove_wall_push(next_pos, config)

        clearance = self._min_predicted_clearance(ice_pos, self.velocity, danger_pos, config, control_dt)
        wall_dist = distance_from_nearest_wall(
            next_pos,
            config.ice_radius,
            config.width,
            config.height,
        )

        status = (
            f"motor cmd=({self._motor_sign(self.command_direction.x):+d},"
            f"{self._motor_sign(self.command_direction.y):+d}) "
            f"speed={self.velocity.length():.0f} "
            f"cone={math.degrees(self.cone_angle):.0f}deg "
            f"clearance={clearance:.0f} wall={wall_dist:.0f} "
            f"next_cmd={max(self.command_timer, 0.0):.2f}s"
        )

        return next_pos, status

    def _plan_direction(
        self,
        *,
        mode: str,
        ice_pos: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
        plan_dt: float,
        idle_target: Vec2 | None,
    ) -> Vec2:
        desired = self._desired_vector(mode, ice_pos, danger_pos, config, idle_target)

        if desired.length() < 1e-6:
            desired = _center_pull(ice_pos, config)

        desired_dir = _safe_normalized(desired)
        risk = self._risk_level(mode, ice_pos, danger_pos, config)

        best_command = self.command_direction
        best_score = -float("inf")

        for candidate_command in self._candidate_motor_commands(mode):
            candidate_velocity = candidate_command * config.motor_nominal_speed
            score = self._score_velocity(
                mode=mode,
                ice_pos=ice_pos,
                velocity=candidate_velocity,
                desired_dir=desired_dir,
                danger_pos=danger_pos,
                config=config,
                dt=plan_dt,
                idle_target=idle_target,
            )
            score += self._score_cone_clearance(
                ice_pos=ice_pos,
                velocity=candidate_velocity,
                cone_angle=self.cone_angle,
                danger_pos=danger_pos,
                config=config,
                dt=plan_dt,
            )
            score += self._direction_change_score(candidate_command, risk, config)

            if score > best_score:
                best_score = score
                best_command = candidate_command

        return best_command

    def _update_hand_velocity(
        self,
        danger_pos: Vec2 | None,
        dt: float,
        config: WorldConfig,
    ):
        self.hand_sample_elapsed += dt

        if danger_pos is None:
            self.prev_hand_pos = None
            self.hand_velocity = self.hand_velocity * 0.82
            self.hand_sample_elapsed = 0.0
            return

        if self.prev_hand_pos is None:
            self.prev_hand_pos = danger_pos
            self.hand_sample_elapsed = 0.0
            return

        sample_moved = distance(danger_pos, self.prev_hand_pos) > 1e-3
        stale_sample = self.hand_sample_elapsed >= (1.5 / config.camera_fps)

        if sample_moved:
            sample_dt = max(self.hand_sample_elapsed, 1e-6)
            raw_velocity = (danger_pos - self.prev_hand_pos) * (1.0 / sample_dt)
            raw_velocity = _clamp_length(raw_velocity, 1400.0)
            a = self.hand_velocity_smoothing
            self.hand_velocity = self.hand_velocity * (1.0 - a) + raw_velocity * a
            self.prev_hand_pos = danger_pos
            self.hand_sample_elapsed = 0.0
        elif stale_sample:
            self.hand_velocity = self.hand_velocity * 0.65
            self.hand_sample_elapsed = 0.0

    def _sample_motor_speed(self, config: WorldConfig) -> float:
        delta = config.motor_speed_delta_ratio
        return config.motor_nominal_speed * random.uniform(1.0 - delta, 1.0 + delta)

    def _sample_cone_rotation_speed(self, config: WorldConfig) -> float:
        delta = config.cone_rotation_delta_ratio
        return config.cone_rotation_speed * random.uniform(1.0 - delta, 1.0 + delta)

    def _plan_cone_angle(
        self,
        *,
        ice_pos: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
    ) -> float:
        if danger_pos is None:
            return self.cone_angle

        predicted_hand = danger_pos + self.hand_velocity * config.control_period
        away = _safe_normalized(ice_pos - predicted_hand)

        if away.length() < 1e-6:
            away = _safe_normalized(ice_pos - danger_pos)

        if away.length() < 1e-6:
            return self.cone_angle

        desired_angle = math.atan2(away.y, away.x)
        max_step = config.cone_rotation_speed * config.control_period

        candidates = [
            self.cone_angle,
            _move_angle_toward(self.cone_angle, desired_angle, max_step),
            _move_angle_toward(self.cone_angle, desired_angle, max_step * 0.55),
            _wrap_angle(desired_angle),
            _wrap_angle(desired_angle + 0.35),
            _wrap_angle(desired_angle - 0.35),
        ]

        best_angle = self.cone_angle
        best_score = -float("inf")

        for candidate in candidates:
            score = self._cone_angle_score(
                ice_pos=ice_pos,
                cone_angle=candidate,
                danger_pos=danger_pos,
                config=config,
            )
            score -= abs(_wrap_angle(candidate - self.cone_angle)) * 25.0

            if score > best_score:
                best_score = score
                best_angle = candidate

        return _wrap_angle(best_angle)

    def _risk_level(
        self,
        mode: str,
        ice_pos: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
    ) -> float:
        if danger_pos is None:
            return 0.0 if mode == "idle" else 0.35

        trigger = config.hand_radius + config.ice_radius + config.escape_trigger_distance
        close = config.hand_radius + config.ice_radius + config.close_distance
        d = distance(ice_pos, danger_pos)

        if d <= close:
            return 1.0

        return clamp(1.0 - (d - close) / max(1.0, trigger - close), 0.15, 1.0)

    def _desired_vector(
        self,
        mode: str,
        ice_pos: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
        idle_target: Vec2 | None,
    ) -> Vec2:
        wall = _wall_repulsion(ice_pos, config, margin=185.0)
        center = _center_pull(ice_pos, config)
        target_pull = _zero()

        if idle_target is not None:
            target_pull = _safe_normalized(idle_target - ice_pos)

        away = _zero()
        tangent = _zero()

        if danger_pos is not None:
            predicted_hand = danger_pos + self.hand_velocity * 0.22
            away = _safe_normalized(ice_pos - predicted_hand)

            if away.length() < 1e-6:
                away = _safe_normalized(ice_pos - danger_pos)

            tangent = _perp(away, self.orbit_sign)

            if (
                self.command_direction.length() > 0.0
                and tangent.dot(self.command_direction) < 0.0
            ):
                tangent = tangent * -1.0
                self.orbit_sign *= -1.0

        if mode == "idle":
            return target_pull * 1.4 + wall * 2.0 + center * 0.4

        if mode == "boundary":
            return wall * 4.2 + center * 1.5 + away * 1.1 + tangent * 0.25

        if mode == "close":
            return away * 4.0 + wall * 3.2 + tangent * 0.5 + center * 0.25

        if mode == "blind":
            return away * 2.3 + wall * 1.8 + center * 0.6 + tangent * 0.35

        return away * 2.7 + wall * 1.9 + center * 0.45 + tangent * 0.55

    def _candidate_motor_commands(self, mode: str) -> list[Vec2]:
        if self.command_direction.length() < 1e-6:
            raw_commands = [
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]
        else:
            current_x = self._motor_sign(self.command_direction.x)
            current_y = self._motor_sign(self.command_direction.y)

            raw_commands = [(current_x, current_y)]
            raw_commands.extend((x, current_y) for x in (-1, 0, 1))
            raw_commands.extend((current_x, y) for y in (-1, 0, 1))

            if mode == "idle":
                raw_commands.append((0, 0))

        unique: list[tuple[int, int]] = []
        for command in raw_commands:
            if command not in unique:
                unique.append(command)

        return [self._command_to_direction(x, y) for x, y in unique]

    def _direction_change_score(
        self,
        candidate_command: Vec2,
        risk: float,
        config: WorldConfig,
    ) -> float:
        current_x = self._motor_sign(self.command_direction.x)
        current_y = self._motor_sign(self.command_direction.y)
        next_x = self._motor_sign(candidate_command.x)
        next_y = self._motor_sign(candidate_command.y)

        changed_axes = int(current_x != next_x) + int(current_y != next_y)
        score = -changed_axes * config.direction_axis_change_penalty * (1.15 - risk * 0.45)

        if changed_axes == 0:
            score += config.direction_keep_bonus * (1.25 - risk * 0.55)

        sign_flips = int(current_x * next_x < 0) + int(current_y * next_y < 0)
        score -= sign_flips * config.direction_axis_change_penalty * (1.0 - risk * 0.35)

        if candidate_command.length() < 1e-6 and risk > 0.2:
            score -= 1200.0

        return score

    def _command_to_direction(self, x: int, y: int) -> Vec2:
        if x == 0 and y == 0:
            return Vec2(0.0, 0.0)
        return Vec2(float(x), float(y)).normalized()

    def _motor_sign(self, value: float) -> int:
        if value > 0.1:
            return 1
        if value < -0.1:
            return -1
        return 0

    def _score_velocity(
        self,
        *,
        mode: str,
        ice_pos: Vec2,
        velocity: Vec2,
        desired_dir: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
        idle_target: Vec2 | None,
    ) -> float:
        next_pos = ice_pos + velocity * dt

        guarded_radius = config.ice_radius + self.boundary_guard

        if not (
            guarded_radius <= next_pos.x <= config.width - guarded_radius
            and guarded_radius <= next_pos.y <= config.height - guarded_radius
        ):
            return -1e9

        speed = velocity.length()
        direction = _safe_normalized(velocity)
        score = 0.0

        clearance = self._min_predicted_clearance(ice_pos, velocity, danger_pos, config, dt)
        required = config.min_safe_distance_extra + self._clearance_buffer(mode)

        if clearance < required:
            score -= (required - clearance) ** 2 * 5.5
        else:
            score += min(clearance, 320.0) * 3.2

        wall_dist = distance_from_nearest_wall(
            next_pos,
            config.ice_radius,
            config.width,
            config.height,
        )
        score += min(wall_dist, 180.0) * 2.1

        cone_wall_dist = self._cone_wall_distance(next_pos, self.cone_angle, config)
        score += min(cone_wall_dist, 140.0) * 4.0

        if cone_wall_dist < config.cone_clearance_extra:
            score -= (config.cone_clearance_extra - cone_wall_dist) ** 2 * 7.0

        if wall_dist < 55.0:
            score -= (55.0 - wall_dist) ** 2 * 3.0
            score += direction.dot(_center_pull(next_pos, config)) * 220.0

        if speed > 1e-6:
            score += direction.dot(desired_dir) * 180.0

        current_speed = self.velocity.length()
        if current_speed > 12.0 and speed > 12.0:
            current_dir = self.velocity.normalized()
            continuity = direction.dot(current_dir)
            score += continuity * 150.0

            if continuity < -0.15:
                score -= (abs(continuity) + 0.2) * 500.0

        score -= abs(speed - current_speed) * 0.10

        if mode == "idle" and idle_target is not None:
            score -= distance(next_pos, idle_target) * 1.2

        if mode == "boundary" and wall_dist < config.boundary_exit_margin:
            score += direction.dot(_center_pull(next_pos, config)) * 260.0

        if danger_pos is not None:
            future_hand = danger_pos + self.hand_velocity * 0.18
            away = _safe_normalized(next_pos - future_hand)
            score += direction.dot(away) * 120.0

        return score

    def _score_cone_clearance(
        self,
        *,
        ice_pos: Vec2,
        velocity: Vec2,
        cone_angle: float,
        danger_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> float:
        if danger_pos is None:
            return 0.0

        clearance = self._min_predicted_cone_clearance(
            ice_pos=ice_pos,
            velocity=velocity,
            cone_angle=cone_angle,
            danger_pos=danger_pos,
            config=config,
            dt=dt,
        )

        required = config.cone_clearance_extra

        if clearance < required:
            return -((required - clearance) ** 2) * config.cone_avoidance_weight

        return min(clearance, 220.0) * config.cone_avoidance_weight

    def _cone_angle_score(
        self,
        *,
        ice_pos: Vec2,
        cone_angle: float,
        danger_pos: Vec2,
        config: WorldConfig,
    ) -> float:
        clearance = self._min_predicted_cone_clearance(
            ice_pos=ice_pos,
            velocity=Vec2(0.0, 0.0),
            cone_angle=cone_angle,
            danger_pos=danger_pos,
            config=config,
            dt=config.control_period,
        )

        wall_dist = self._cone_wall_distance(ice_pos, cone_angle, config)
        score = clearance * 8.0 + min(wall_dist, 120.0) * 7.0

        if wall_dist < config.cone_clearance_extra:
            score -= (config.cone_clearance_extra - wall_dist) ** 2 * 12.0

        return score

    def _min_predicted_cone_clearance(
        self,
        *,
        ice_pos: Vec2,
        velocity: Vec2,
        cone_angle: float,
        danger_pos: Vec2,
        config: WorldConfig,
        dt: float,
    ) -> float:
        min_clearance = float("inf")

        for horizon in (0.0, dt, 0.16, 0.32):
            future_ice = ice_pos + velocity * horizon
            future_hand = danger_pos + self.hand_velocity * horizon
            min_clearance = min(
                min_clearance,
                self._cone_clearance_to_hand(
                    center=future_ice,
                    cone_angle=cone_angle,
                    hand_pos=future_hand,
                    config=config,
                ),
            )

        return min_clearance

    def _cone_clearance_to_hand(
        self,
        *,
        center: Vec2,
        cone_angle: float,
        hand_pos: Vec2,
        config: WorldConfig,
    ) -> float:
        base_a, base_b, tip = _cone_points(center, cone_angle, config)

        if _point_in_triangle(hand_pos, base_a, base_b, tip):
            return -config.hand_radius

        d = min(
            _distance_point_to_segment(hand_pos, base_a, base_b),
            _distance_point_to_segment(hand_pos, base_a, tip),
            _distance_point_to_segment(hand_pos, base_b, tip),
        )

        return d - config.hand_radius

    def _cone_wall_distance(
        self,
        center: Vec2,
        cone_angle: float,
        config: WorldConfig,
    ) -> float:
        points = _cone_points(center, cone_angle, config)
        return min(
            min(
                p.x,
                config.width - p.x,
                p.y,
                config.height - p.y,
            )
            for p in points
        )

    def _clearance_buffer(self, mode: str) -> float:
        if mode == "close":
            return 95.0
        if mode == "blind":
            return 75.0
        if mode == "boundary":
            return 50.0
        if mode == "idle":
            return 25.0
        return 60.0

    def _min_predicted_clearance(
        self,
        ice_pos: Vec2,
        velocity: Vec2,
        danger_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> float:
        if danger_pos is None:
            return 999.0

        body_radius = config.hand_radius + config.ice_radius
        min_clearance = float("inf")

        for horizon in (0.0, dt, 0.12, 0.24, 0.40):
            future_ice = ice_pos + velocity * horizon
            future_hand = danger_pos + self.hand_velocity * horizon
            min_clearance = min(min_clearance, distance(future_ice, future_hand) - body_radius)

        return min_clearance

    def _remove_wall_push(self, pos: Vec2, config: WorldConfig):
        r = config.ice_radius + self.boundary_guard + 1.0
        clipped = False

        if pos.x <= r and self.velocity.x < 0.0:
            self.velocity = Vec2(0.0, self.velocity.y)
            clipped = True
        elif pos.x >= config.width - r and self.velocity.x > 0.0:
            self.velocity = Vec2(0.0, self.velocity.y)
            clipped = True

        if pos.y <= r and self.velocity.y < 0.0:
            self.velocity = Vec2(self.velocity.x, 0.0)
            clipped = True
        elif pos.y >= config.height - r and self.velocity.y > 0.0:
            self.velocity = Vec2(self.velocity.x, 0.0)
            clipped = True

        if clipped:
            self.command_direction = _safe_normalized(self.velocity)


class PredictiveSafetyAlgo(BaseAlgo):
    def __init__(self, role: RobotState, planner: PredictiveSafetyPlanner):
        self.role = role
        self.planner = planner
        self.t = 0.0
        self.close_timer = 0.0
        self.is_active = False

    def on_enter(self, ice_pos, hand_pos, last_hand_pos, config):
        if self.role == RobotState.CLOSE_RETREAT:
            self.is_active = True
            self.close_timer = 0.35

    def update(self, ice_pos, hand_pos, last_hand_pos, config, dt):
        self.t += dt

        if self.role == RobotState.IDLE:
            phase = 2.0 * math.pi * 0.55 * self.t
            idle_target = _center(config) + Vec2(math.cos(phase) * 16.0, math.sin(phase) * 9.0)
            next_pos, status = self.planner.step(
                mode="idle",
                ice_pos=ice_pos,
                danger_pos=None,
                config=config,
                dt=dt,
                idle_target=idle_target,
            )
            return AlgoResult(
                next_pos,
                IDLE_COLOR,
                IDLE_BORDER_COLOR,
                f"IDLE / {status}",
                cone_angle=self.planner.cone_angle,
            )

        if self.role == RobotState.BOUNDARY_ESCAPE:
            next_pos, status = self.planner.step(
                mode="boundary",
                ice_pos=ice_pos,
                danger_pos=hand_pos or last_hand_pos,
                config=config,
                dt=dt,
            )
            return AlgoResult(
                next_pos,
                BOUNDARY_COLOR,
                BOUNDARY_BORDER_COLOR,
                f"BOUNDARY / {status}",
                cone_angle=self.planner.cone_angle,
            )

        if self.role == RobotState.CLOSE_RETREAT:
            danger = hand_pos or last_hand_pos
            next_pos, status = self.planner.step(
                mode="close",
                ice_pos=ice_pos,
                danger_pos=danger,
                config=config,
                dt=dt,
            )

            self.close_timer = max(0.0, self.close_timer - dt)
            safe_release_dist = config.hand_radius + config.ice_radius + config.escape_trigger_distance + 35.0

            if self.close_timer <= 0.0 and (
                danger is None or distance(next_pos, danger) >= safe_release_dist
            ):
                self.is_active = False

            return AlgoResult(
                next_pos,
                RETREAT_COLOR,
                RETREAT_BORDER_COLOR,
                f"CLOSE / {status}",
                cone_angle=self.planner.cone_angle,
            )

        if self.role == RobotState.BLIND_SPOT_ESCAPE:
            next_pos, status = self.planner.step(
                mode="blind",
                ice_pos=ice_pos,
                danger_pos=last_hand_pos or config.blind_zone.center(),
                config=config,
                dt=dt,
            )
            return AlgoResult(
                next_pos,
                BLIND_COLOR,
                BLIND_BORDER_COLOR,
                f"BLIND / {status}",
                cone_angle=self.planner.cone_angle,
            )

        next_pos, status = self.planner.step(
            mode="normal",
            ice_pos=ice_pos,
            danger_pos=hand_pos,
            config=config,
            dt=dt,
        )
        return AlgoResult(
            next_pos,
            ACTIVE_COLOR,
            ACTIVE_BORDER_COLOR,
            f"NORMAL / {status}",
            cone_angle=self.planner.cone_angle,
        )


# ============================================================
# Profile factory
# ============================================================

def create_algorithm_profile(profile: str):
    """
    回傳完整 state -> algorithm 對應。

    可選 profile：
    - "smooth"
    - "performer"
    - "safe"
    - "predictive_safe"
    """

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

    if profile == "predictive_safe":
        planner = PredictiveSafetyPlanner()

        return {
            RobotState.IDLE: PredictiveSafetyAlgo(RobotState.IDLE, planner),
            RobotState.NORMAL_ESCAPE: PredictiveSafetyAlgo(RobotState.NORMAL_ESCAPE, planner),
            RobotState.BOUNDARY_ESCAPE: PredictiveSafetyAlgo(RobotState.BOUNDARY_ESCAPE, planner),
            RobotState.CLOSE_RETREAT: PredictiveSafetyAlgo(RobotState.CLOSE_RETREAT, planner),
            RobotState.BLIND_SPOT_ESCAPE: PredictiveSafetyAlgo(RobotState.BLIND_SPOT_ESCAPE, planner),
        }

    raise ValueError(f"Unknown algorithm profile: {profile}")
