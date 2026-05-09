from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto


class RobotState(Enum):
    IDLE = auto()
    NORMAL_ESCAPE = auto()
    BOUNDARY_ESCAPE = auto()
    CLOSE_RETREAT = auto()
    BLIND_SPOT_ESCAPE = auto()


@dataclass
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
            return Vec2(0, 0)
        return Vec2(self.x / length, self.y / length)


@dataclass
class Rect:
    x: float
    y: float
    w: float
    h: float

    def contains(self, p: Vec2) -> bool:
        return (
            self.x <= p.x <= self.x + self.w
            and self.y <= p.y <= self.y + self.h
        )

    def center(self) -> Vec2:
        return Vec2(
            self.x + self.w / 2,
            self.y + self.h / 2,
        )

    def expanded(self, margin: float) -> "Rect":
        return Rect(
            self.x - margin,
            self.y - margin,
            self.w + margin * 2,
            self.h + margin * 2,
        )


@dataclass
class AlgoResult:
    pos: Vec2
    color: tuple[int, int, int]
    border_color: tuple[int, int, int]
    status: str
    cone_angle: float | None = None


@dataclass
class StateDecision:
    next_state: RobotState
    reason: str


@dataclass
class WorldConfig:
    width: int = 900
    height: int = 600

    render_fps: int = 60
    camera_fps: float = 20.0
    control_period: float = 0.10

    motor_nominal_speed: float = 860.0
    motor_speed_delta_ratio: float = 0.08
    direction_keep_bonus: float = 220.0
    direction_axis_change_penalty: float = 260.0

    cone_length: float = 86.0
    cone_base_width: float = 64.0
    cone_rotation_speed: float = 5.8
    cone_rotation_delta_ratio: float = 0.10
    cone_clearance_extra: float = 25.0
    cone_avoidance_weight: float = 4.0

    hand_radius: float = 55
    ice_radius: float = 22

    escape_trigger_distance: float = 180
    close_distance: float = 45

    boundary_enter_margin: float = 90
    boundary_exit_margin: float = 130

    hand_lost_blind_timeout: float = 0.8
    blind_zone_detection_margin: float = 70

    min_safe_distance_extra: float = 10

    blind_zone: Rect = field(
        default_factory=lambda: Rect(360, 210, 180, 180)
    )


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


def distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_position(
    pos: Vec2,
    radius: float,
    width: int,
    height: int,
) -> Vec2:
    return Vec2(
        clamp(pos.x, radius, width - radius),
        clamp(pos.y, radius, height - radius),
    )


def distance_from_nearest_wall(
    pos: Vec2,
    radius: float,
    width: int,
    height: int,
) -> float:
    left = pos.x - radius
    right = width - (pos.x + radius)
    top = pos.y - radius
    bottom = height - (pos.y + radius)

    return min(left, right, top, bottom)


def is_near_boundary(
    pos: Vec2,
    radius: float,
    width: int,
    height: int,
    margin: float,
) -> bool:
    return (
        distance_from_nearest_wall(pos, radius, width, height)
        < margin
    )


def move_toward(
    current: Vec2,
    target: Vec2,
    max_distance: float,
) -> Vec2:
    delta = target - current
    dist = delta.length()

    if dist <= max_distance:
        return target

    return current + delta.normalized() * max_distance
