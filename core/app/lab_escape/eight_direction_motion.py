from __future__ import annotations

import random
from dataclasses import dataclass
from enum import Enum, auto

from core.app.lab_escape.common import (
    Vec2,
    WorldConfig,
    clamp_position_with_config,
    distance,
    distance_from_nearest_wall_with_config,
    workspace_center,
)


class EightDirMode(Enum):
    RANDOM_VALID = auto()
    WEIGHTED_ESCAPE = auto()
    BOUNDARY_AWARE = auto()
    ESCAPE_FROM_POINT = auto()


@dataclass
class EightDirCommand:
    next_pos: Vec2
    direction: Vec2
    speed: float
    valid_count: int
    reason: str


DIRECTIONS_8 = [
    Vec2(1, 0),
    Vec2(1, 1).normalized(),
    Vec2(0, 1),
    Vec2(-1, 1).normalized(),
    Vec2(-1, 0),
    Vec2(-1, -1).normalized(),
    Vec2(0, -1),
    Vec2(1, -1).normalized(),
]


def run_eight_direction_motion(
    *,
    current_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    dt: float,
    move_speed: float,
    mode: EightDirMode,
    locked_direction: Vec2 | None = None,
) -> EightDirCommand:
    """
    八方向 motion primitive。

    locked_direction:
        用來降低 NORMAL_ESCAPE 抖動。
        如果上一個方向仍然 valid，高階 algo 可以要求這次繼續沿用舊方向。
    """

    if danger_pos is None and mode != EightDirMode.BOUNDARY_AWARE:
        return EightDirCommand(
            next_pos=current_pos,
            direction=Vec2(0, 0),
            speed=0.0,
            valid_count=0,
            reason="eight-dir / no danger position",
        )

    step = move_speed * dt
    valid_dirs: list[Vec2] = []

    require_farther_from_danger = mode in {
        EightDirMode.RANDOM_VALID,
        EightDirMode.WEIGHTED_ESCAPE,
        EightDirMode.ESCAPE_FROM_POINT,
    }

    for direction in DIRECTIONS_8:
        candidate = current_pos + direction * step

        if _is_valid_direction(
            candidate=candidate,
            current_pos=current_pos,
            danger_pos=danger_pos,
            config=config,
            require_farther_from_danger=require_farther_from_danger,
        ):
            valid_dirs.append(direction)

    if not valid_dirs:
        return EightDirCommand(
            next_pos=current_pos,
            direction=Vec2(0, 0),
            speed=0.0,
            valid_count=0,
            reason="eight-dir / no valid direction",
        )

    if locked_direction is not None and locked_direction in valid_dirs:
        chosen_dir = locked_direction
        reason = f"eight-dir / keep locked direction / valid={len(valid_dirs)}"

    elif mode == EightDirMode.RANDOM_VALID:
        chosen_dir = random.choice(valid_dirs)
        reason = f"eight-dir / random valid / valid={len(valid_dirs)}"

    elif mode == EightDirMode.WEIGHTED_ESCAPE:
        chosen_dir = _choose_weighted_escape_direction(
            directions=valid_dirs,
            current_pos=current_pos,
            danger_pos=danger_pos,
            config=config,
            step=step,
        )
        reason = f"eight-dir / weighted escape / valid={len(valid_dirs)}"

    elif mode == EightDirMode.BOUNDARY_AWARE:
        chosen_dir = _choose_boundary_aware_direction(
            directions=valid_dirs,
            current_pos=current_pos,
            danger_pos=danger_pos,
            config=config,
            step=step,
        )
        reason = f"eight-dir / boundary aware / valid={len(valid_dirs)}"

    elif mode == EightDirMode.ESCAPE_FROM_POINT:
        chosen_dir = _choose_escape_from_point_direction(
            directions=valid_dirs,
            current_pos=current_pos,
            danger_pos=danger_pos,
            config=config,
            step=step,
        )
        reason = f"eight-dir / escape from point / valid={len(valid_dirs)}"

    else:
        raise ValueError(f"Unknown EightDirMode: {mode}")

    next_pos = current_pos + chosen_dir * step
    next_pos = clamp_position_with_config(
        next_pos,
        config.ice_radius_cm,
        config,
    )

    return EightDirCommand(
        next_pos=next_pos,
        direction=chosen_dir,
        speed=move_speed,
        valid_count=len(valid_dirs),
        reason=reason,
    )


def _is_valid_direction(
    *,
    candidate: Vec2,
    current_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    require_farther_from_danger: bool,
) -> bool:
    inside_window = (
        config.y_min_cm + config.ice_radius_cm
        <= candidate.x
        <= config.y_max_cm - config.ice_radius_cm
        and config.z_min_cm + config.ice_radius_cm
        <= candidate.y
        <= config.z_max_cm - config.ice_radius_cm
    )

    if not inside_window:
        return False

    if danger_pos is None:
        return True

    current_dist = distance(current_pos, danger_pos)
    candidate_dist = distance(candidate, danger_pos)

    min_safe_distance = (
        config.hand_radius_cm
        + config.ice_radius_cm
        + config.min_safe_distance_extra_cm
    )

    if candidate_dist < min_safe_distance:
        return False

    if require_farther_from_danger and candidate_dist <= current_dist:
        return False

    return True


def _choose_weighted_escape_direction(
    *,
    directions: list[Vec2],
    current_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    step: float,
) -> Vec2:
    if danger_pos is None:
        return random.choice(directions)

    best_dir = directions[0]
    best_score = -float("inf")

    away_from_danger = (current_pos - danger_pos).normalized()

    for direction in directions:
        candidate = current_pos + direction * step

        candidate_dist = distance(candidate, danger_pos)
        direction_alignment = direction.dot(away_from_danger)
        wall_penalty = _wall_penalty(candidate, config)
        noise = random.uniform(-0.5, 0.5)

        score = 0.0
        score += candidate_dist * 1.0
        score += direction_alignment * 9.0
        score -= wall_penalty * 1.5
        score += noise

        if score > best_score:
            best_score = score
            best_dir = direction

    return best_dir


def _choose_boundary_aware_direction(
    *,
    directions: list[Vec2],
    current_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    step: float,
) -> Vec2:
    best_dir = directions[0]
    best_score = -float("inf")

    center = workspace_center(config)
    toward_center = (center - current_pos).normalized()

    away_from_danger = None
    if danger_pos is not None:
        away_from_danger = (current_pos - danger_pos).normalized()

    for direction in directions:
        candidate = current_pos + direction * step

        center_alignment = direction.dot(toward_center)
        wall_distance = distance_from_nearest_wall_with_config(
            candidate,
            config.ice_radius_cm,
            config,
        )

        score = 0.0
        score += center_alignment * 18.0
        score += wall_distance * 1.8

        if danger_pos is not None and away_from_danger is not None:
            candidate_dist = distance(candidate, danger_pos)
            escape_alignment = direction.dot(away_from_danger)

            score += escape_alignment * 7.0
            score += candidate_dist * 0.4

        score += random.uniform(-0.5, 0.5)

        if score > best_score:
            best_score = score
            best_dir = direction

    return best_dir


def _choose_escape_from_point_direction(
    *,
    directions: list[Vec2],
    current_pos: Vec2,
    danger_pos: Vec2 | None,
    config: WorldConfig,
    step: float,
) -> Vec2:
    if danger_pos is None:
        return random.choice(directions)

    best_dir = directions[0]
    best_score = -float("inf")

    away_from_danger = (current_pos - danger_pos).normalized()

    for direction in directions:
        candidate = current_pos + direction * step

        candidate_dist = distance(candidate, danger_pos)
        escape_alignment = direction.dot(away_from_danger)
        wall_penalty = _wall_penalty(candidate, config)

        score = 0.0
        score += escape_alignment * 12.0
        score += candidate_dist * 1.0
        score -= wall_penalty * 1.0
        score += random.uniform(-0.5, 0.5)

        if score > best_score:
            best_score = score
            best_dir = direction

    return best_dir


def _wall_penalty(pos: Vec2, config: WorldConfig) -> float:
    nearest_wall_distance = distance_from_nearest_wall_with_config(
        pos,
        config.ice_radius_cm,
        config,
    )

    if nearest_wall_distance >= config.boundary_enter_margin_cm:
        return 0.0

    return config.boundary_enter_margin_cm - nearest_wall_distance
