from __future__ import annotations

"""Minimal smooth lab escape geometry in robot YZ centimeters.

Vec2.x is robot Y (left/right), Vec2.y is robot Z (up/down).
The controller keeps robot X unchanged and only asks this module for the next YZ target.
"""

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import TypeAlias

ImagePointPx: TypeAlias = tuple[float, float]


class EscapeState(Enum):
    IDLE = auto()
    NORMAL_ESCAPE = auto()
    BOUNDARY_ESCAPE = auto()
    CLOSE_RETREAT = auto()
    BLIND_SPOT_ESCAPE = auto()


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, k: float) -> "Vec2":
        return Vec2(self.x * k, self.y * k)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> "Vec2":
        length = self.length()
        if length < 1e-6:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / length, self.y / length)


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle in YZ plane. x/y means min-y/min-z corner."""

    x: float
    y: float
    w: float
    h: float

    def contains(self, p: Vec2) -> bool:
        return self.x <= p.x <= self.x + self.w and self.y <= p.y <= self.y + self.h

    def center(self) -> Vec2:
        return Vec2(self.x + self.w / 2.0, self.y + self.h / 2.0)

    def expanded(self, margin: float) -> "Rect":
        return Rect(
            self.x - margin,
            self.y - margin,
            self.w + margin * 2.0,
            self.h + margin * 2.0,
        )


@dataclass(frozen=True)
class AlgoResult:
    pos: Vec2
    color: tuple[int, int, int]
    border_color: tuple[int, int, int]
    status: str
    visible: bool = True
    planned_landing: Vec2 | None = None


@dataclass(frozen=True)
class StateDecision:
    next_state: EscapeState
    reason: str


@dataclass
class LabEscapeConfig:
    """Single tuning surface for the smooth-only escape controller.

    The lab escape layer works in robot-frame Y/Z centimeters. Camera pixels are
    converted into the same Y/Z workspace before the state machine runs.
    """

    # Workspace geometry: y in [-25, 25], z in [-5, 25].
    boundary_size_cm: float = 50.0
    boundary_height_cm: float = 30.0
    origin_to_left_boundary_cm: float = 25.0
    origin_to_top_boundary_cm: float = 25.0

    # Camera mapping. Calibrate these four image points to the physical
    # workspace corners, in this order:
    # top-left, top-right, bottom-right, bottom-left.
    pixels_per_cm: float = 10.0
    image_workspace_points_px: tuple[
        ImagePointPx,
        ImagePointPx,
        ImagePointPx,
        ImagePointPx,
    ] = (
        (170.0, 165.0),
        (450.0, 200.0),
        (450.0, 350.0),
        (160.0, 350.0),
    )
    min_hand_confidence: float = 0.2

    # Max physical YZ displacement per core control update.
    max_command_step_cm: float = 20.0

    # Object sizes.
    hand_radius_cm: float = 3.0
    ice_radius_cm: float = 3.0

    # State thresholds. These are the main knobs you are expected to tune.
    escape_trigger_distance_cm: float = -1.0
    close_retreat_distance_cm: float = -1.0
    boundary_enter_margin_cm: float = 1.0
    boundary_exit_margin_cm: float = 2.0
    enable_close_retreat: bool = True

    # Internal stability knobs. Usually do not touch these first.
    hand_stationary_speed_cm_s: float = 4.0
    hand_lost_blind_timeout_s: float = 0.45
    blind_zone_detection_margin_cm: float = 2.5

    # Motion speeds in the lab YZ plane.
    normal_speed_cm_s: float = 40.0
    boundary_speed_cm_s: float = 40.0
    retreat_speed_cm_s: float = 40.0
    idle_speed_cm_s: float = 15.0

    blind_zone: Rect | None = None

    def __post_init__(self) -> None:
        if self.blind_zone is None:
            self.blind_zone = Rect(-3, -2, 6, 3)

    @property
    def y_min_cm(self) -> float:
        return -self.origin_to_left_boundary_cm

    @property
    def origin_to_right_boundary_cm(self) -> float:
        return self.boundary_size_cm - self.origin_to_left_boundary_cm

    @property
    def y_max_cm(self) -> float:
        return self.origin_to_right_boundary_cm

    @property
    def origin_to_bottom_boundary_cm(self) -> float:
        return self.boundary_height_cm - self.origin_to_top_boundary_cm

    @property
    def z_min_cm(self) -> float:
        return -self.origin_to_bottom_boundary_cm

    @property
    def z_max_cm(self) -> float:
        return self.origin_to_top_boundary_cm

    @property
    def vision_width_px(self) -> float:
        return self.boundary_size_cm * self.pixels_per_cm

    @property
    def vision_height_px(self) -> float:
        return self.boundary_height_cm * self.pixels_per_cm

    @property
    def target_y_min_cm(self) -> float:
        return self.y_min_cm

    @property
    def target_y_max_cm(self) -> float:
        return self.y_max_cm

    @property
    def target_z_min_cm(self) -> float:
        return self.z_min_cm

    @property
    def target_z_max_cm(self) -> float:
        return self.z_max_cm


WorldConfig = LabEscapeConfig


ACTIVE_COLOR = (120, 190, 255)
ACTIVE_BORDER_COLOR = (40, 90, 180)
IDLE_COLOR = (180, 210, 255)
IDLE_BORDER_COLOR = (80, 120, 180)
BOUNDARY_COLOR = (255, 190, 90)
BOUNDARY_BORDER_COLOR = (180, 100, 20)
RETREAT_COLOR = (150, 150, 150)
RETREAT_BORDER_COLOR = (90, 90, 90)
BLIND_COLOR = (170, 120, 230)
BLIND_BORDER_COLOR = (90, 40, 150)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def workspace_center(config: WorldConfig) -> Vec2:
    return Vec2(
        (config.y_min_cm + config.y_max_cm) / 2.0,
        (config.z_min_cm + config.z_max_cm) / 2.0,
    )


def clamp_position_to_square(
    pos: Vec2,
    object_radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    margin: float = 0.0,
) -> Vec2:
    low_x = y_min + object_radius + margin
    high_x = y_max - object_radius - margin
    low_y = z_min + object_radius + margin
    high_y = z_max - object_radius - margin

    if low_x > high_x or low_y > high_y:
        return Vec2((y_min + y_max) / 2.0, (z_min + z_max) / 2.0)

    return Vec2(clamp(pos.x, low_x, high_x), clamp(pos.y, low_y, high_y))


def clamp_position_with_config(pos: Vec2, radius: float, config: WorldConfig) -> Vec2:
    return clamp_position_to_square(
        pos,
        radius,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
    )


def distance_from_nearest_wall_with_config(
    pos: Vec2,
    object_radius: float,
    config: WorldConfig,
) -> float:
    left = pos.x - object_radius - config.y_min_cm
    right = config.y_max_cm - (pos.x + object_radius)
    bottom = pos.y - object_radius - config.z_min_cm
    top = config.z_max_cm - (pos.y + object_radius)
    return min(left, right, bottom, top)


def move_toward(current: Vec2, target: Vec2, max_distance: float) -> Vec2:
    delta = target - current
    length = delta.length()
    if length < 1e-6 or length <= max_distance:
        return target
    return current + delta.normalized() * max_distance
