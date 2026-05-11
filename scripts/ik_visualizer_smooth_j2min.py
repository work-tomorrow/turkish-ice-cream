import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


# =========================
# 機構參數
# =========================

L_SHORT = 15.0   # 原本短桿，cm
L_ROD = 100.0    # 冰淇淋桿，cm

J2_FORWARD_OFFSET = 5.0
J3_FORWARD_OFFSET = 5.0

# 這個圓只作為「extra 開始啟用」的控制門檻
EXTRA_TRIGGER_RADIUS = 15.0

# 從 15 cm 到 20 cm 之間平滑打開 extra，避免角度不連續
EXTRA_FULL_RADIUS = 20.0

MAX_EXTRA_PITCH_DEG = 15.0
MAX_EXTRA_YAW_DEG = 15.0

# J2 hardware limits
# J2 最小只能到 40 度；若 IK 需要更小角度，solver 會卡在 -40 度並留下 error。
J2_MIN_DEG = -180.0
J2_MAX_DEG = 40.0
J2_MIN_RAD = math.radians(J2_MIN_DEG)
J2_MAX_RAD = math.radians(J2_MAX_DEG)


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
    ])


def rot_z(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def trans_x(length):
    return np.array([
        [1, 0, 0, length],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


# =========================
# 小工具
# =========================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def clamp_j2_rad(j2):
    return clamp(j2, J2_MIN_RAD, J2_MAX_RAD)


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

    J1 = math.degrees(j1)
    J2 = math.degrees(j2)
    J3 = math.degrees(-j2 + extra_pitch)
    J4 = math.degrees(-j1 + extra_yaw)

    return {
        "J1": normalize_deg(J1),
        "J2": normalize_deg(J2),
        "J3": normalize_deg(J3),
        "J4": normalize_deg(J4),
        "extra_pitch": math.degrees(extra_pitch),
        "extra_yaw": math.degrees(extra_yaw),
    }


def _end_yz_from_vars(v):
    angles = _angles_from_vars(v)
    _, _, rod_end, _ = forward_kinematics(angles)
    return np.array([rod_end[1], rod_end[2]], dtype=float)


def _initial_guess_from_target(target_y, target_z):
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
    j2 = clamp_j2_rad(j2)

    return np.array([j1, j2, 0.0, 0.0], dtype=float)


def ik_keep_forward_yz_smooth(target_y, target_z, initial_v=None):
    """
    平滑版 IK。

    target_y, target_z:
        末端在 Y-Z 平面的目標位置，單位 cm。

    initial_v:
        上一次的 [j1, j2, extra_pitch, extra_yaw]，單位 rad。
        視覺化或實機控制時建議傳入，會讓角度更連續。

    行為：
        r <= 15 cm:
            extra_pitch / extra_yaw 會被限制為 0
        15 < r < 20 cm:
            extra 上限從 0 平滑增加到 ±15°
        r >= 20 cm:
            extra 上限為 ±15°
    """
    target_y = float(target_y)
    target_z = float(target_z)
    target = np.array([target_y, target_z], dtype=float)

    r = float(np.linalg.norm(target))

    activation = smoothstep(
        EXTRA_TRIGGER_RADIUS,
        EXTRA_FULL_RADIUS,
        r,
    )

    max_extra_pitch = math.radians(MAX_EXTRA_PITCH_DEG) * activation
    max_extra_yaw = math.radians(MAX_EXTRA_YAW_DEG) * activation

    if initial_v is None:
        v = _initial_guess_from_target(target_y, target_z)
    else:
        v = np.array(initial_v, dtype=float)

    # 先套用 J2 硬體限制
    v[1] = clamp_j2_rad(v[1])

    # 如果 activation 變小，先把 extra 夾回當前允許範圍
    v[2] = clamp(v[2], -max_extra_pitch, max_extra_pitch)
    v[3] = clamp(v[3], -max_extra_yaw, max_extra_yaw)

    # numerical IK
    for _ in range(100):
        current = _end_yz_from_vars(v)
        err = target - current

        if float(np.linalg.norm(err)) < 0.03:
            break

        # numerical Jacobian: 2 x 4
        J = np.zeros((2, 4), dtype=float)
        eps = 1e-4

        for i in range(4):
            vv = v.copy()
            vv[i] += eps
            J[:, i] = (_end_yz_from_vars(vv) - current) / eps

        # 讓 extra 有比較高的使用成本：
        # 能用 J1/J2 靠近時，就不要太早使用 J3/J4 的 extra。
        reg = np.diag([
            0.01,  # J1 step penalty
            0.01,  # J2 step penalty
            0.08,  # extra_pitch step penalty
            0.08,  # extra_yaw step penalty
        ])

        A = J.T @ J + reg + 1e-6 * np.eye(4)
        b = J.T @ err

        step = np.linalg.solve(A, b)

        v += 0.5 * step

        # J2 硬體限制
        v[1] = clamp_j2_rad(v[1])

        # extra 限制；圓內 activation=0，所以這裡會讓 extra=0
        v[2] = clamp(v[2], -max_extra_pitch, max_extra_pitch)
        v[3] = clamp(v[3], -max_extra_yaw, max_extra_yaw)

        # 避免 numerical solver 把 j1/j2 跑到太誇張的分支
        v[0] = (v[0] + math.pi) % (2.0 * math.pi) - math.pi
        v[1] = (v[1] + math.pi) % (2.0 * math.pi) - math.pi
        v[1] = clamp_j2_rad(v[1])

    angles = _angles_from_vars(v)

    _, _, rod_end, _ = forward_kinematics(angles)

    reached_y = float(rod_end[1])
    reached_z = float(rod_end[2])

    error_cm = math.sqrt(
        (reached_y - target_y) ** 2 +
        (reached_z - target_z) ** 2
    )

    angles["target_y"] = target_y
    angles["target_z"] = target_z
    angles["reached_y"] = reached_y
    angles["reached_z"] = reached_z
    angles["error_cm"] = error_cm
    angles["activation"] = activation

    if activation <= 1e-6:
        angles["mode"] = "inside: extra disabled"
    elif activation >= 1.0 - 1e-6:
        angles["mode"] = "outside: extra fully enabled"
    else:
        angles["mode"] = "transition: extra smoothly enabled"

    # 給下一幀 warm start 用
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


def main():
    fig = plt.figure(figsize=(13, 7))

    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2d = fig.add_subplot(1, 2, 2)

    plt.subplots_adjust(bottom=0.25)

    slider_y_ax = plt.axes([0.20, 0.12, 0.60, 0.03])
    slider_z_ax = plt.axes([0.20, 0.07, 0.60, 0.03])

    slider_y = Slider(slider_y_ax, "target_y cm", -40, 40, valinit=0)
    slider_z = Slider(slider_z_ax, "target_z cm", -40, 40, valinit=0)

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

        ax3d.set_xlim(60, 140)
        ax3d.set_ylim(-55, 55)
        ax3d.set_zlim(-55, 55)
        ax3d.set_xlabel("X cm")
        ax3d.set_ylabel("Y cm")
        ax3d.set_zlabel("Z cm")
        set_axes_equal(ax3d)

        ax3d.set_title(
            "3D Arm IK Viewer\n"
            f"J1={angles['J1']:.1f}, J2={angles['J2']:.1f}, "
            f"J3={angles['J3']:.1f}, J4={angles['J4']:.1f}\n"
            f"extra_pitch={angles['extra_pitch']:.1f}, "
            f"extra_yaw={angles['extra_yaw']:.1f}, "
            f"activation={angles['activation']:.2f}\n"
            f"J2 limit=[{J2_MIN_DEG:.0f}, {J2_MAX_DEG:.0f}] deg\\n"
            f"End=({rod_end[0]:.1f}, {rod_end[1]:.1f}, {rod_end[2]:.1f}) cm, "
            f"err={angles['error_cm']:.2f} cm\n"
            f"{angles['mode']}"
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

        ax2d.set_xlim(-50, 50)
        ax2d.set_ylim(-50, 50)
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

                "J1_deg": angles["J1"],
                "J2_deg": angles["J2"],
                "J3_deg": angles["J3"],
                "J4_deg": angles["J4"],

                "extra_pitch_deg": angles.get("extra_pitch", 0.0),
                "extra_yaw_deg": angles.get("extra_yaw", 0.0),
                "activation": angles.get("activation", 0.0),

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
        "J1_deg",
        "J2_deg",
        "J3_deg",
        "J4_deg",
        "extra_pitch_deg",
        "extra_yaw_deg",
        "activation",
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
