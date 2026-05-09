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
    visible: bool = True
    planned_landing: Vec2 | None = None


@dataclass
class StateDecision:
    next_state: RobotState
    reason: str


@dataclass
class WorldConfig:
    # 80cm x 80cm square workspace, simulated as 10 px/cm.
    width: int = 800
    height: int = 800
    pixels_per_cm: float = 10.0
    boundary_size_cm: float = 80.0
    origin_to_left_boundary_cm: float = 40.0
    origin_to_top_boundary_cm: float = 40.0

    hand_radius: float = 55
    ice_radius: float = 22

    escape_trigger_distance: float = 180
    close_distance: float = 45

    boundary_enter_margin: float = 90
    boundary_exit_margin: float = 130

    hand_lost_blind_timeout: float = 0.8
    blind_zone_detection_margin: float = 70

    min_safe_distance_extra: float = 10

    decision_interval: float = 0.1
    flat_escape_speed: float = 500
    landing_min_distance: float = 280
    landing_boundary_margin: float = 280
    boundary_escape_distance: float = 60
    boundary_escape_max_target_distance: float = 150
    idle_exit_distance: float = 200
    contact_escape_distance: float = 185
    flat_escape_min_angle_deg: float = 100
    close_flat_escape_distance: float = 150
    close_flat_escape_min_angle_deg: float = 120
    blind_escape_center_distance: float = 280
    direct_approach_alignment: float = 0.95
    hand_stationary_speed: float = 15
    enable_close_retreat: bool = False

    blind_zone: Rect | None = None

    def __post_init__(self):
        size = int(self.boundary_size_cm * self.pixels_per_cm)
        self.width = size
        self.height = size

        if self.blind_zone is None:
            self.blind_zone = Rect(
                self.width / 2 - 60,
                self.height / 2 - 90,
                120,
                180,
            )

    def origin(self) -> Vec2:
        return Vec2(
            self.origin_to_left_boundary_cm * self.pixels_per_cm,
            self.origin_to_top_boundary_cm * self.pixels_per_cm,
        )

    def origin_to_right_boundary_cm(self) -> float:
        return self.boundary_size_cm - self.origin_to_left_boundary_cm

    def origin_to_bottom_boundary_cm(self) -> float:
        return self.boundary_size_cm - self.origin_to_top_boundary_cm


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


def boundary_center(width: int, height: int) -> Vec2:
    return Vec2(width / 2, height / 2)


def distance_from_square_boundary(
    pos: Vec2,
    object_radius: float,
    width: int,
    height: int,
) -> float:
    left = pos.x - object_radius
    right = width - (pos.x + object_radius)
    top = pos.y - object_radius
    bottom = height - (pos.y + object_radius)

    return min(left, right, top, bottom)


def is_inside_square_boundary(
    pos: Vec2,
    object_radius: float,
    width: int,
    height: int,
) -> bool:
    return (
        distance_from_square_boundary(
            pos,
            object_radius,
            width,
            height,
        )
        >= 0
    )


def clamp_position_to_square(
    pos: Vec2,
    object_radius: float,
    width: int,
    height: int,
    margin: float = 0.0,
) -> Vec2:
    low_x = object_radius + margin
    high_x = width - object_radius - margin
    low_y = object_radius + margin
    high_y = height - object_radius - margin

    if low_x > high_x or low_y > high_y:
        return boundary_center(width, height)

    return Vec2(
        clamp(pos.x, low_x, high_x),
        clamp(pos.y, low_y, high_y),
    )


def distance_to_square_boundary_along_direction(
    pos: Vec2,
    direction: Vec2,
    object_radius: float,
    width: int,
    height: int,
) -> float:
    unit_dir = direction.normalized()

    if unit_dir.length() < 1e-6:
        return 0.0

    distances: list[float] = []

    if unit_dir.x > 1e-6:
        distances.append((width - object_radius - pos.x) / unit_dir.x)
    elif unit_dir.x < -1e-6:
        distances.append((object_radius - pos.x) / unit_dir.x)

    if unit_dir.y > 1e-6:
        distances.append((height - object_radius - pos.y) / unit_dir.y)
    elif unit_dir.y < -1e-6:
        distances.append((object_radius - pos.y) / unit_dir.y)

    positive_distances = [d for d in distances if d >= 0.0]

    if not positive_distances:
        return 0.0

    return min(positive_distances)


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
