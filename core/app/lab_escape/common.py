from __future__ import annotations

"""Lab escape geometry in robot YZ centimeters.

Vec2 is always (y_robot_cm, z_robot_cm), matching XYZPosition.y_cm / z_cm.
"""

import math
from dataclasses import dataclass
from enum import Enum, auto


class RobotState(Enum):
    IDLE = auto()
    NORMAL_ESCAPE = auto()
    BOUNDARY_ESCAPE = auto()
    CLOSE_RETREAT = auto()
    BLIND_SPOT_ESCAPE = auto()


@dataclass
class Vec2:
    """Robot Y (left/right) and Z (up/down), both in centimeters."""

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
    """Axis-aligned rectangle in (y_cm, z_cm). (x, y) is min corner; w, h spans in y and z."""

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
    """Square YZ workspace in centimeters (robot frame)."""

    boundary_size_cm: float = 25.0
    origin_to_left_boundary_cm: float = 12.5
    origin_to_top_boundary_cm: float = 12.5

    hand_radius_cm: float = 3.0
    ice_radius_cm: float = 2.0

    escape_trigger_distance_cm: float = 4.0
    close_distance_cm: float = 4.5

    boundary_enter_margin_cm: float = 2.5
    boundary_exit_margin_cm: float = 4.0

    hand_lost_blind_timeout_s: float = 0.8
    blind_zone_detection_margin_cm: float = 3.5

    min_safe_distance_extra_cm: float = 1.0

    decision_interval_s: float = 0.1
    flat_escape_speed_cm_s: float = 50.0
    landing_min_distance_cm: float = 8.0
    landing_boundary_margin_cm: float = 3.0
    boundary_escape_distance_cm: float = 2.0
    boundary_escape_max_target_distance_cm: float = 5.0
    idle_exit_distance_cm: float = 15.0
    contact_escape_distance_cm: float = 9.0
    flat_escape_min_angle_deg: float = 100.0
    close_flat_escape_distance_cm: float = 8.0
    close_flat_escape_min_angle_deg: float = 120.0
    blind_escape_center_distance_cm: float = 6.0
    direct_approach_alignment: float = 0.95
    hand_stationary_speed_cm_s: float = 1.5
    enable_close_retreat: bool = False

    blind_zone: Rect | None = None

    def __post_init__(self) -> None:
        if self.blind_zone is None:
            self.blind_zone = Rect(-3.75, -5.625, 7.5, 11.25)

    @property
    def y_min_cm(self) -> float:
        """Minimum robot Y (cm): ``-origin_to_left_boundary_cm``."""
        return -self.origin_to_left_boundary_cm

    @property
    def origin_to_right_boundary_cm(self) -> float:
        """Positive span from workspace origin to the right (+Y) boundary."""
        return self.boundary_size_cm - self.origin_to_left_boundary_cm

    @property
    def y_max_cm(self) -> float:
        """Maximum robot Y (cm): same magnitude as ``origin_to_right_boundary_cm``."""
        return self.origin_to_right_boundary_cm

    @property
    def origin_to_bottom_boundary_cm(self) -> float:
        """Positive span from workspace origin to the lower (-Z) boundary."""
        return self.boundary_size_cm - self.origin_to_top_boundary_cm

    @property
    def z_min_cm(self) -> float:
        """Minimum robot Z (cm): ``-origin_to_bottom_boundary_cm`` (below origin)."""
        return -self.origin_to_bottom_boundary_cm

    @property
    def z_max_cm(self) -> float:
        """Maximum robot Z (cm): ``origin_to_top_boundary_cm`` (above origin)."""
        return self.origin_to_top_boundary_cm



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


def workspace_center(config: WorldConfig) -> Vec2:
    return Vec2(
        (config.y_min_cm + config.y_max_cm) / 2.0,
        (config.z_min_cm + config.z_max_cm) / 2.0,
    )


def distance_from_square_boundary(
    pos: Vec2,
    object_radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> float:
    left = pos.x - object_radius - y_min
    right = y_max - (pos.x + object_radius)
    top = pos.y - object_radius - z_min
    bottom = z_max - (pos.y + object_radius)

    return min(left, right, top, bottom)


def is_inside_square_boundary(
    pos: Vec2,
    object_radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> bool:
    return (
        distance_from_square_boundary(
            pos,
            object_radius,
            y_min,
            y_max,
            z_min,
            z_max,
        )
        >= 0
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

    return Vec2(
        clamp(pos.x, low_x, high_x),
        clamp(pos.y, low_y, high_y),
    )


def distance_to_square_boundary_along_direction(
    pos: Vec2,
    direction: Vec2,
    object_radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> float:
    unit_dir = direction.normalized()

    if unit_dir.length() < 1e-6:
        return 0.0

    distances: list[float] = []

    if unit_dir.x > 1e-6:
        distances.append((y_max - object_radius - pos.x) / unit_dir.x)
    elif unit_dir.x < -1e-6:
        distances.append((y_min + object_radius - pos.x) / unit_dir.x)

    if unit_dir.y > 1e-6:
        distances.append((z_max - object_radius - pos.y) / unit_dir.y)
    elif unit_dir.y < -1e-6:
        distances.append((z_min + object_radius - pos.y) / unit_dir.y)

    positive_distances = [d for d in distances if d >= 0.0]

    if not positive_distances:
        return 0.0

    return min(positive_distances)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_position(
    pos: Vec2,
    radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> Vec2:
    return Vec2(
        clamp(pos.x, y_min + radius, y_max - radius),
        clamp(pos.y, z_min + radius, z_max - radius),
    )


def clamp_position_with_config(pos: Vec2, radius: float, config: WorldConfig) -> Vec2:
    return clamp_position(
        pos,
        radius,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
    )


def distance_from_nearest_wall(
    pos: Vec2,
    radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> float:
    left = pos.x - radius - y_min
    right = y_max - (pos.x + radius)
    top = pos.y - radius - z_min
    bottom = z_max - (pos.y + radius)

    return min(left, right, top, bottom)


def distance_from_nearest_wall_with_config(
    pos: Vec2,
    radius: float,
    config: WorldConfig,
) -> float:
    return distance_from_nearest_wall(
        pos,
        radius,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
    )


def is_near_boundary(
    pos: Vec2,
    radius: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    margin: float,
) -> bool:
    return (
        distance_from_nearest_wall(pos, radius, y_min, y_max, z_min, z_max)
        < margin
    )


def is_near_boundary_with_config(
    pos: Vec2,
    radius: float,
    config: WorldConfig,
    margin: float,
) -> bool:
    return is_near_boundary(
        pos,
        radius,
        config.y_min_cm,
        config.y_max_cm,
        config.z_min_cm,
        config.z_max_cm,
        margin,
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
