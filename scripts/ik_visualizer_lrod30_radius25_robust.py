import csv
import math
from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


# =========================
# 機構參數
# =========================

L_SHORT = 15.0   # 原本短桿，cm
L_ROD = 45.0    # 冰淇淋桿，cm

J2_FORWARD_OFFSET = 5.0
J3_FORWARD_OFFSET = 5.0

# 這個圓只作為「extra 開始啟用」的控制門檻
EXTRA_TRIGGER_RADIUS = 15.0

# 從 15 cm 到 20 cm 之間平滑打開 extra，避免角度不連續
EXTRA_FULL_RADIUS = 20.0

MAX_EXTRA_PITCH_DEG = 60.0
MAX_EXTRA_YAW_DEG = 30.0

# 這版的目標：半徑 25 cm 內盡量都要能穩定解出
TARGET_GOOD_RADIUS = 25.0


# =========================
# 每顆馬達的實體角度範圍與方向
# =========================

@dataclass(frozen=True)
class JointLimit:
    """
    min_deg / max_deg:
        實際送給馬達的角度範圍，單位 degree。

    sign:
        1.0  代表馬達正方向跟 model angle 相同。
        -1.0 代表馬達正方向跟 model angle 相反。

    zero_deg:
        馬達角度 offset。

    轉換公式：
        motor_angle = zero_deg + sign * model_angle
    """
    min_deg: float = -180.0
    max_deg: float = 180.0
    sign: float = 1.0
    zero_deg: float = 0.0

    def model_to_motor(self, model_deg: float) -> float:
        return self.zero_deg + self.sign * model_deg

    def motor_to_model(self, motor_deg: float) -> float:
        return (motor_deg - self.zero_deg) / self.sign

    def clamp_motor(self, motor_deg: float) -> float:
        return clamp(motor_deg, self.min_deg, self.max_deg)

    def clamp_model(self, model_deg: float) -> float:
        motor_deg = self.model_to_motor(model_deg)
        motor_deg = self.clamp_motor(motor_deg)
        return self.motor_to_model(motor_deg)

    def model_bounds_deg(self) -> tuple[float, float]:
        a = self.motor_to_model(self.min_deg)
        b = self.motor_to_model(self.max_deg)
        return min(a, b), max(a, b)


# 你主要調這裡
# min/max 是「實際馬達角度範圍」，不是 model angle。
# J2 這裡先照前面設定：方向反了，且實體最小只能到 -40 度。
JOINT_LIMITS = {
    "J1": JointLimit(min_deg=-45.0, max_deg=45.0, sign=1.0, zero_deg=0.0),
    "J2": JointLimit(min_deg=-40.0, max_deg=80.0, sign=1.0, zero_deg=0.0),
    "J3": JointLimit(min_deg=-60.0, max_deg=60.0, sign=1.0, zero_deg=0.0),
    "J4": JointLimit(min_deg=-60.0, max_deg=60.0, sign=-1.0, zero_deg=0.0),
}


# =========================
# 基本矩陣
# =========================

def rot_y(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def rot_z(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def trans_x(length):
    return np.array([
        [1, 0, 0, length],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=float)


# =========================
# 小工具
# =========================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def smoothstep(edge0, edge1, x):
    """
    平滑從 0 變到 1：
    x <= edge0 -> 0
    x >= edge1 -> 1
    edge0 < x < edge1 -> 平滑過渡
    """
    if edge0 == edge1:
        return 1.0 if x >= edge1 else 0.0

    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)



def approximate_extra_reach_cm():
    """
    粗估 extra_pitch / extra_yaw 在 Y-Z 平面額外能提供的位移。
    """
    max_extra_deg = max(MAX_EXTRA_PITCH_DEG, MAX_EXTRA_YAW_DEG)
    effective_extra_length = L_ROD + J3_FORWARD_OFFSET
    return effective_extra_length * math.sin(math.radians(max_extra_deg))


def suggested_yz_view_limit_cm():
    """
    根據 L_ROD / extra 角度自動估計視覺化範圍。
    """
    approx = EXTRA_TRIGGER_RADIUS + approximate_extra_reach_cm()
    return max(TARGET_GOOD_RADIUS + 5.0, math.ceil((approx + 10.0) / 5.0) * 5.0)


def set_axes_from_points(ax, points, margin=10.0):
    """
    根據實際手臂點位自動設定 3D 顯示範圍，避免 L_ROD 改短後被固定 xlim 裁掉。
    """
    pts = np.asarray(points, dtype=float)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)

    center = (mins + maxs) / 2.0
    span = float(np.max(maxs - mins))
    half = max(span / 2.0 + margin, 25.0)

    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)


def _raw_initial_guess_from_target(target_y, target_z):
    """
    不套 joint limit 的初始猜測，用來判斷 J2 是否會撞到限制。
    """
    r = math.sqrt(target_y * target_y + target_z * target_z)

    if r < 1e-9:
        y = 0.0
        z = 0.0
    else:
        rr = min(r, EXTRA_TRIGGER_RADIUS)
        scale = rr / r
        y = target_y * scale
        z = target_z * scale

    x_offset = math.sqrt(max(0.0, EXTRA_TRIGGER_RADIUS ** 2 - y * y - z * z))

    j1 = math.atan2(y, x_offset)
    j2 = math.asin(clamp(-z / EXTRA_TRIGGER_RADIUS, -1.0, 1.0))

    return np.array([j1, j2, 0.0, 0.0], dtype=float)


def _joint_limit_activation(target_y, target_z):
    """
    原本 activation 只看半徑，導致 target 在半徑 15 內時 extra 被關掉。
    但如果 J2 已經撞到硬體限制，半徑 15 內也可能需要 J3 extra 幫忙。

    這裡用 raw J2 和被 joint limit clamp 後的 J2 差距，平滑打開 extra。
    """
    raw_v = _raw_initial_guess_from_target(target_y, target_z)
    raw_j2_deg = math.degrees(float(raw_v[1]))
    clamped_j2_deg = _clamp_model_angle("J2", raw_j2_deg)

    diff_deg = abs(raw_j2_deg - clamped_j2_deg)

    # 差 0 度不用 extra；差到 8 度以上時 extra 完全打開
    return smoothstep(0.0, 8.0, diff_deg)


def _make_initial_candidates(target_y, target_z, max_extra_pitch, max_extra_yaw, initial_v=None):
    """
    多組 initial guess，避免 numerical IK 卡在局部解或 joint limit 邊界。
    """
    candidates = []

    if initial_v is not None:
        candidates.append(np.array(initial_v, dtype=float))

    base = _initial_guess_from_target(target_y, target_z, max_extra_pitch, max_extra_yaw)
    candidates.append(base)

    # 根據目標方向給一個 extra 初始猜測：
    # extra_pitch 正值通常讓 Z 更往負方向；
    # extra_yaw 正值通常讓 Y 更往正方向。
    effective_len = max(1e-6, L_ROD + J3_FORWARD_OFFSET)
    ep_guess = math.asin(clamp(-target_z / effective_len, -math.sin(max_extra_pitch), math.sin(max_extra_pitch)))
    ey_guess = math.asin(clamp(target_y / effective_len, -math.sin(max_extra_yaw), math.sin(max_extra_yaw)))

    v_guess = base.copy()
    v_guess[2] = ep_guess
    v_guess[3] = ey_guess
    candidates.append(_clamp_v_to_joint_limits(v_guess, max_extra_pitch, max_extra_yaw))

    # 邊界起點：對小 L_ROD 很重要
    for ep in (-max_extra_pitch, 0.0, max_extra_pitch):
        for ey in (-max_extra_yaw, 0.0, max_extra_yaw):
            v = base.copy()
            v[2] = ep
            v[3] = ey
            candidates.append(_clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw))

    # 去除重複
    unique = []
    seen = set()
    for v in candidates:
        key = tuple(round(float(x), 5) for x in v)
        if key not in seen:
            unique.append(v)
            seen.add(key)

    return unique


def _solve_numerical_ik_once(target, v, max_extra_pitch, max_extra_yaw, iterations=120):
    """
    從單一 initial guess 開始解一次 IK。
    """
    v = _clamp_v_to_joint_limits(np.array(v, dtype=float), max_extra_pitch, max_extra_yaw)

    for _ in range(iterations):
        current = _end_yz_from_vars(v)
        err = target - current

        if float(np.linalg.norm(err)) < 0.03:
            break

        J = np.zeros((2, 4), dtype=float)
        eps = 1e-4

        for i in range(4):
            vv = v.copy()
            vv[i] += eps
            vv = _clamp_v_to_joint_limits(vv, max_extra_pitch, max_extra_yaw)
            J[:, i] = (_end_yz_from_vars(vv) - current) / eps

        reg = np.diag([
            0.008,  # J1 step penalty
            0.008,  # J2 step penalty
            0.02,   # extra_pitch step penalty
            0.02,   # extra_yaw step penalty
        ])

        A = J.T @ J + reg + 1e-6 * np.eye(4)
        b = J.T @ err

        try:
            step = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            step = np.linalg.pinv(A) @ b

        v += 0.6 * step

        v[0] = (v[0] + math.pi) % (2.0 * math.pi) - math.pi
        v[1] = (v[1] + math.pi) % (2.0 * math.pi) - math.pi

        v = _clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)

    angles = _angles_from_vars(v)
    _, _, rod_end, _ = forward_kinematics(angles)
    err_cm = math.sqrt((float(rod_end[1]) - target[0]) ** 2 + (float(rod_end[2]) - target[1]) ** 2)

    return err_cm, v, angles, rod_end


def model_to_motor_angles(model_angles):
    return {
        joint: normalize_deg(JOINT_LIMITS[joint].model_to_motor(float(angle)))
        for joint, angle in model_angles.items()
        if joint in JOINT_LIMITS
    }


def motor_targets_from_angles(angles):
    return {
        "J1": angles["J1_motor"],
        "J2": angles["J2_motor"],
        "J3": angles["J3_motor"],
        "J4": angles["J4_motor"],
    }


def _clamp_model_angle(joint, model_deg):
    return JOINT_LIMITS[joint].clamp_model(model_deg)


def _clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw):
    """
    v = [j1, j2, extra_pitch, extra_yaw]，單位 rad。

    J1 = j1
    J2 = j2
    J3 = -j2 + extra_pitch
    J4 = -j1 + extra_yaw

    因此：
        extra_pitch = J3 + J2
        extra_yaw   = J4 + J1
    """
    v = np.array(v, dtype=float)

    # J1 / J2 直接限制
    j1_deg = _clamp_model_angle("J1", math.degrees(float(v[0])))
    j2_deg = _clamp_model_angle("J2", math.degrees(float(v[1])))

    v[0] = math.radians(j1_deg)
    v[1] = math.radians(j2_deg)

    # 先限制 extra 自己的 activation 範圍
    v[2] = clamp(float(v[2]), -max_extra_pitch, max_extra_pitch)
    v[3] = clamp(float(v[3]), -max_extra_yaw, max_extra_yaw)

    # J3 limit: J3 = -J2 + extra_pitch
    j2_rad = float(v[1])
    j3_min_deg, j3_max_deg = JOINT_LIMITS["J3"].model_bounds_deg()
    extra_pitch_min = math.radians(j3_min_deg) + j2_rad
    extra_pitch_max = math.radians(j3_max_deg) + j2_rad

    lo = max(-max_extra_pitch, min(extra_pitch_min, extra_pitch_max))
    hi = min(max_extra_pitch, max(extra_pitch_min, extra_pitch_max))
    if lo <= hi:
        v[2] = clamp(float(v[2]), lo, hi)
    else:
        # 如果 extra 限制和馬達限制沒有交集，優先滿足馬達限制，留下 error。
        v[2] = clamp(
            float(v[2]),
            min(extra_pitch_min, extra_pitch_max),
            max(extra_pitch_min, extra_pitch_max),
        )

    # J4 limit: J4 = -J1 + extra_yaw
    j1_rad = float(v[0])
    j4_min_deg, j4_max_deg = JOINT_LIMITS["J4"].model_bounds_deg()
    extra_yaw_min = math.radians(j4_min_deg) + j1_rad
    extra_yaw_max = math.radians(j4_max_deg) + j1_rad

    lo = max(-max_extra_yaw, min(extra_yaw_min, extra_yaw_max))
    hi = min(max_extra_yaw, max(extra_yaw_min, extra_yaw_max))
    if lo <= hi:
        v[3] = clamp(float(v[3]), lo, hi)
    else:
        # 如果 extra 限制和馬達限制沒有交集，優先滿足馬達限制，留下 error。
        v[3] = clamp(
            float(v[3]),
            min(extra_yaw_min, extra_yaw_max),
            max(extra_yaw_min, extra_yaw_max),
        )

    return v


# =========================
# FK：把 J1~J4 算成末端位置
# =========================

def forward_kinematics(angles):
    """
    機構順序：

    J1 yaw
    J2 pitch
    J2 forward offset
    L_SHORT
    J3 pitch
    J3 forward offset
    J4 yaw
    L_ROD

    注意：
        這裡吃的是 model angles，不是 motor angles。
    """
    j1 = math.radians(angles["J1"])
    j2 = math.radians(angles["J2"])
    j3 = math.radians(angles["J3"])
    j4 = math.radians(angles["J4"])

    T = np.eye(4)

    points = []
    points.append(T[:3, 3].copy())  # base

    # J1 yaw
    T = T @ rot_z(j1)
    points.append(T[:3, 3].copy())

    # J2 pitch
    T = T @ rot_y(j2)

    # J2 馬達軸線往前伸出的 5 cm
    T = T @ trans_x(J2_FORWARD_OFFSET)
    points.append(T[:3, 3].copy())

    # 原本短桿 15 cm
    T = T @ trans_x(L_SHORT)
    points.append(T[:3, 3].copy())

    # J3 pitch
    T = T @ rot_y(j3)

    # J3 馬達軸線往前伸出的 5 cm
    T = T @ trans_x(J3_FORWARD_OFFSET)
    rod_start = T[:3, 3].copy()
    points.append(rod_start)

    # J4 yaw
    T = T @ rot_z(j4)

    # 冰淇淋桿 100 cm
    T = T @ trans_x(L_ROD)
    rod_end = T[:3, 3].copy()
    points.append(rod_end)

    end_forward = T[:3, 0]

    return np.array(points), rod_start, rod_end, end_forward


# =========================
# IK：平滑啟用 J3/J4 extra
# =========================

def _angles_from_vars(v):
    """
    v = [j1, j2, extra_pitch, extra_yaw]
    單位都是 rad。

    原本保持朝前：
        J3 = -J2
        J4 = -J1

    平滑擴展範圍：
        J3 = -J2 + extra_pitch
        J4 = -J1 + extra_yaw
    """
    j1, j2, extra_pitch, extra_yaw = v

    J1 = normalize_deg(math.degrees(j1))
    J2 = normalize_deg(math.degrees(j2))
    J3 = normalize_deg(math.degrees(-j2 + extra_pitch))
    J4 = normalize_deg(math.degrees(-j1 + extra_yaw))

    model_angles = {
        "J1": J1,
        "J2": J2,
        "J3": J3,
        "J4": J4,
    }

    motor_angles = model_to_motor_angles(model_angles)

    return {
        **model_angles,

        "J1_motor": motor_angles["J1"],
        "J2_motor": motor_angles["J2"],
        "J3_motor": motor_angles["J3"],
        "J4_motor": motor_angles["J4"],

        "extra_pitch": math.degrees(extra_pitch),
        "extra_yaw": math.degrees(extra_yaw),
    }


def _end_yz_from_vars(v):
    angles = _angles_from_vars(v)
    _, _, rod_end, _ = forward_kinematics(angles)
    return np.array([rod_end[1], rod_end[2]], dtype=float)


def _initial_guess_from_target(target_y, target_z, max_extra_pitch, max_extra_yaw):
    """
    用原本圓形模型給 numerical IK 一個穩定初始猜測。
    這不是精確答案，只是初始值。
    """
    r = math.sqrt(target_y * target_y + target_z * target_z)

    if r < 1e-9:
        y = 0.0
        z = 0.0
    else:
        rr = min(r, EXTRA_TRIGGER_RADIUS)
        scale = rr / r
        y = target_y * scale
        z = target_z * scale

    x_offset = math.sqrt(max(0.0, EXTRA_TRIGGER_RADIUS ** 2 - y * y - z * z))

    j1 = math.atan2(y, x_offset)
    j2 = math.asin(clamp(-z / EXTRA_TRIGGER_RADIUS, -1.0, 1.0))

    v = np.array([j1, j2, 0.0, 0.0], dtype=float)
    return _clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)


def ik_keep_forward_yz_smooth(target_y, target_z, initial_v=None):
    """
    平滑版 IK。

    這版針對 L_ROD = 30 cm 做了兩個修正：

    1. 如果 J2 撞到硬體限制，即使 target 還在原本 trigger circle 裡，
       也會平滑打開 J3/J4 extra 來補償。

    2. 如果 warm start 解不好，會自動嘗試多組 initial guess，
       避免在某些範圍卡住。
    """
    target_y = float(target_y)
    target_z = float(target_z)
    target = np.array([target_y, target_z], dtype=float)

    r = float(np.linalg.norm(target))

    radial_activation = smoothstep(
        EXTRA_TRIGGER_RADIUS,
        EXTRA_FULL_RADIUS,
        r,
    )

    limit_activation = _joint_limit_activation(target_y, target_z)

    activation = max(radial_activation, limit_activation)

    max_extra_pitch = math.radians(MAX_EXTRA_PITCH_DEG) * activation
    max_extra_yaw = math.radians(MAX_EXTRA_YAW_DEG) * activation

    candidates = _make_initial_candidates(
        target_y,
        target_z,
        max_extra_pitch,
        max_extra_yaw,
        initial_v=initial_v,
    )

    best = None

    for candidate in candidates:
        err_cm, v, candidate_angles, rod_end = _solve_numerical_ik_once(
            target,
            candidate,
            max_extra_pitch,
            max_extra_yaw,
            iterations=120,
        )

        if best is None or err_cm < best[0]:
            best = (err_cm, v, candidate_angles, rod_end)

        if err_cm < 0.03:
            break

    error_cm, v, angles, rod_end = best

    reached_y = float(rod_end[1])
    reached_z = float(rod_end[2])

    angles["target_y"] = target_y
    angles["target_z"] = target_z
    angles["reached_y"] = reached_y
    angles["reached_z"] = reached_z
    angles["error_cm"] = error_cm
    angles["activation"] = activation
    angles["radial_activation"] = radial_activation
    angles["limit_activation"] = limit_activation
    angles["motor_targets"] = motor_targets_from_angles(angles)

    if activation <= 1e-6:
        angles["mode"] = "inside: extra disabled"
    elif limit_activation > radial_activation + 1e-6:
        angles["mode"] = "joint-limit assist: extra enabled"
    elif activation >= 1.0 - 1e-6:
        angles["mode"] = "outside: extra fully enabled"
    else:
        angles["mode"] = "transition: extra smoothly enabled"

    angles["v_rad"] = v.copy()

    return angles


def ik_keep_forward_yz(target_y, target_z, initial_v=None):
    """
    保留舊函數名稱，方便你原本程式不用大改。
    """
    return ik_keep_forward_yz_smooth(target_y, target_z, initial_v=initial_v)


# =========================
# 視覺化
# =========================

def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_mid = np.mean(x_limits)
    y_mid = np.mean(y_limits)
    z_mid = np.mean(z_limits)

    max_range = max(
        abs(x_limits[1] - x_limits[0]),
        abs(y_limits[1] - y_limits[0]),
        abs(z_limits[1] - z_limits[0]),
    )

    ax.set_xlim3d(x_mid - max_range / 2, x_mid + max_range / 2)
    ax.set_ylim3d(y_mid - max_range / 2, y_mid + max_range / 2)
    ax.set_zlim3d(z_mid - max_range / 2, z_mid + max_range / 2)


def _joint_limit_summary():
    parts = []
    for joint in ["J1", "J2", "J3", "J4"]:
        limit = JOINT_LIMITS[joint]
        sign_text = "+" if limit.sign > 0 else "-"
        parts.append(f"{joint}[{limit.min_deg:.0f},{limit.max_deg:.0f}], sign={sign_text}")
    return " | ".join(parts)


def main():
    fig = plt.figure(figsize=(13, 7))

    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2d = fig.add_subplot(1, 2, 2)

    plt.subplots_adjust(bottom=0.25)

    slider_y_ax = plt.axes([0.20, 0.12, 0.60, 0.03])
    slider_z_ax = plt.axes([0.20, 0.07, 0.60, 0.03])

    yz_view_limit = suggested_yz_view_limit_cm()

    slider_y = Slider(slider_y_ax, "target_y cm", -yz_view_limit, yz_view_limit, valinit=0)
    slider_z = Slider(slider_z_ax, "target_z cm", -yz_view_limit, yz_view_limit, valinit=0)

    solver_state = {"v": None}

    def update(_=None):
        target_y = slider_y.val
        target_z = slider_z.val

        angles = ik_keep_forward_yz(
            target_y,
            target_z,
            initial_v=solver_state["v"],
        )
        solver_state["v"] = angles["v_rad"]

        points, rod_start, rod_end, end_forward = forward_kinematics(angles)

        # =========================
        # 3D 圖
        # =========================
        ax3d.clear()

        xs = points[:, 0]
        ys = points[:, 1]
        zs = points[:, 2]

        ax3d.plot(xs, ys, zs, marker="o", linewidth=3)

        labels = [
            "Base",
            "J1",
            "J2 offset",
            "L_SHORT end",
            "Rod start",
            "Ice cream",
        ]

        for p, label in zip(points, labels):
            ax3d.text(p[0], p[1], p[2], label)

        # 畫末端 forward arrow
        d = end_forward / (np.linalg.norm(end_forward) + 1e-9)
        ax3d.quiver(
            rod_end[0], rod_end[1], rod_end[2],
            d[0], d[1], d[2],
            length=20,
            normalize=True,
        )

        # 畫世界座標軸
        ax3d.quiver(0, 0, 0, 30, 0, 0, arrow_length_ratio=0.1)
        ax3d.quiver(0, 0, 0, 0, 30, 0, arrow_length_ratio=0.1)
        ax3d.quiver(0, 0, 0, 0, 0, 30, arrow_length_ratio=0.1)

        ax3d.text(32, 0, 0, "X forward")
        ax3d.text(0, 32, 0, "Y left/right")
        ax3d.text(0, 0, 32, "Z up/down")

        set_axes_from_points(ax3d, points, margin=15.0)
        ax3d.set_xlabel("X cm")
        ax3d.set_ylabel("Y cm")
        ax3d.set_zlabel("Z cm")

        ax3d.set_title(
            "3D Arm IK Viewer with Joint Limits\n"
            f"model: J1={angles['J1']:.1f}, J2={angles['J2']:.1f}, "
            f"J3={angles['J3']:.1f}, J4={angles['J4']:.1f}\n"
            f"motor: J1={angles['J1_motor']:.1f}, J2={angles['J2_motor']:.1f}, "
            f"J3={angles['J3_motor']:.1f}, J4={angles['J4_motor']:.1f}\n"
            f"extra_pitch={angles['extra_pitch']:.1f}, "
            f"extra_yaw={angles['extra_yaw']:.1f}, "
            f"activation={angles['activation']:.2f} "
            f"(radial={angles.get('radial_activation', 0):.2f}, "
            f"limit={angles.get('limit_activation', 0):.2f})\n"
            f"L_ROD={L_ROD:.1f} cm, target_good_radius={TARGET_GOOD_RADIUS:.1f} cm, "
            f"approx_extra_reach={approximate_extra_reach_cm():.1f} cm\n"
            f"End=({rod_end[0]:.1f}, {rod_end[1]:.1f}, {rod_end[2]:.1f}) cm, "
            f"err={angles['error_cm']:.2f} cm\n"
            f"{angles['mode']}\n"
            f"{_joint_limit_summary()}"
        )

        # =========================
        # 2D Y-Z 平面圖
        # =========================
        ax2d.clear()

        trigger_circle = plt.Circle(
            (0, 0),
            EXTRA_TRIGGER_RADIUS,
            fill=False,
            linestyle="--",
            label="extra trigger radius",
        )
        full_circle = plt.Circle(
            (0, 0),
            EXTRA_FULL_RADIUS,
            fill=False,
            linestyle=":",
            label="extra full radius",
        )
        ax2d.add_patch(trigger_circle)
        ax2d.add_patch(full_circle)

        ax2d.scatter([target_y], [target_z], s=80, label="Target y/z")
        ax2d.scatter([rod_end[1]], [rod_end[2]], s=80, label="Reached y/z")

        ax2d.plot(
            [target_y, rod_end[1]],
            [target_z, rod_end[2]],
            linewidth=1,
            label="error",
        )

        ax2d.axhline(0, linewidth=1)
        ax2d.axvline(0, linewidth=1)

        good_circle = plt.Circle(
            (0, 0),
            TARGET_GOOD_RADIUS,
            fill=False,
            linestyle="-.",
            label="target good radius",
        )
        ax2d.add_patch(good_circle)

        ax2d.set_xlim(-yz_view_limit, yz_view_limit)
        ax2d.set_ylim(-yz_view_limit, yz_view_limit)
        ax2d.set_aspect("equal", adjustable="box")
        ax2d.set_xlabel("Y cm, left/right")
        ax2d.set_ylabel("Z cm, up/down")
        ax2d.set_title("Camera-like Y-Z View")
        ax2d.legend(loc="upper right")

        fig.canvas.draw_idle()

    slider_y.on_changed(update)
    slider_z.on_changed(update)

    update()
    plt.show()


# =========================
# 表格產生
# =========================

def generate_ik_angle_table(
    y_min=-40,
    y_max=40,
    z_min=-40,
    z_max=40,
    output_path="ik_angle_table.csv",
    reachable_error_threshold_cm=0.5,
):
    """
    產生 y, z = -40 ~ 40 每一個整數點的 IK 角度表。

    使用 snake scan + warm start：
    讓相鄰格點更容易使用連續的 IK 解。
    """

    rows = []
    last_v = None
    scan_index = 0

    for y in range(y_min, y_max + 1):
        if (y - y_min) % 2 == 0:
            z_values = range(z_min, z_max + 1)
        else:
            z_values = range(z_max, z_min - 1, -1)

        for z in z_values:
            angles = ik_keep_forward_yz(
                float(y),
                float(z),
                initial_v=last_v,
            )
            last_v = angles["v_rad"]

            _, _, rod_end, _ = forward_kinematics(angles)

            reached_y = float(rod_end[1])
            reached_z = float(rod_end[2])

            error_cm = math.sqrt(
                (reached_y - y) ** 2 +
                (reached_z - z) ** 2
            )

            reachable = error_cm <= reachable_error_threshold_cm

            row = {
                "scan_index": scan_index,
                "target_y_cm": y,
                "target_z_cm": z,

                "J1_model_deg": angles["J1"],
                "J2_model_deg": angles["J2"],
                "J3_model_deg": angles["J3"],
                "J4_model_deg": angles["J4"],

                "J1_motor_deg": angles["J1_motor"],
                "J2_motor_deg": angles["J2_motor"],
                "J3_motor_deg": angles["J3_motor"],
                "J4_motor_deg": angles["J4_motor"],

                "extra_pitch_deg": angles.get("extra_pitch", 0.0),
                "extra_yaw_deg": angles.get("extra_yaw", 0.0),
                "activation": angles.get("activation", 0.0),
                "radial_activation": angles.get("radial_activation", 0.0),
                "limit_activation": angles.get("limit_activation", 0.0),

                "reached_y_cm": reached_y,
                "reached_z_cm": reached_z,
                "error_cm": error_cm,

                "reachable": reachable,
                "mode": angles.get("mode", ""),
            }

            rows.append(row)
            scan_index += 1

    fieldnames = [
        "scan_index",
        "target_y_cm",
        "target_z_cm",

        "J1_model_deg",
        "J2_model_deg",
        "J3_model_deg",
        "J4_model_deg",

        "J1_motor_deg",
        "J2_motor_deg",
        "J3_motor_deg",
        "J4_motor_deg",

        "extra_pitch_deg",
        "extra_yaw_deg",
        "activation",
        "radial_activation",
        "limit_activation",
        "reached_y_cm",
        "reached_z_cm",
        "error_cm",
        "reachable",
        "mode",
    ]

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            writer.writerow({
                key: (
                    round(value, 6)
                    if isinstance(value, float)
                    else value
                )
                for key, value in row.items()
            })

    total = len(rows)
    reachable_count = sum(1 for row in rows if row["reachable"])

    print(f"Saved table to: {output_path}")
    print(f"Total points: {total}")
    print(f"Reachable points: {reachable_count}")
    print(f"Unreachable points: {total - reachable_count}")


if __name__ == "__main__":
    # 開視覺化介面
    main()

    # 如果要產生 y,z = -40~40 表格，改成下面這樣：
    #
    # generate_ik_angle_table(
    #     y_min=-40,
    #     y_max=40,
    #     z_min=-40,
    #     z_max=40,
    #     output_path="ik_angle_table.csv",
    # )
    # main()
