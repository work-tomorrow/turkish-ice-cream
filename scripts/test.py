import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


# =========================
# 機構參數
# =========================

L_SHORT = 15.0   # J2 後面的短桿，cm
L_ROD = 100.0    # 冰淇淋桿，cm

J2_FORWARD_OFFSET = 5.0
J3_FORWARD_OFFSET = 5.0

EXTRA_TRIGGER_RADIUS = 15.0  # 仍然用原本那個圓當作是否啟用 extra 的門檻

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
# IK：保持冰淇淋桿朝前
# =========================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


MAX_EXTRA_PITCH_DEG = 15.0
MAX_EXTRA_YAW_DEG = 15.0


def normalize_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _angles_from_vars(v):
    """
    v = [j1, j2, extra_pitch, extra_yaw]
    單位都是 rad
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


def _solve_j1_j2_keep_forward(target_y, target_z):
    """
    圓內使用：
    只調 J1/J2。
    固定 extra_pitch = 0, extra_yaw = 0。
    """

    target = np.array([target_y, target_z], dtype=float)

    # 初始猜測：仍然用舊圓公式給一個接近值
    r = math.sqrt(target_y * target_y + target_z * target_z)
    y = target_y
    z = target_z

    if r > EXTRA_TRIGGER_RADIUS:
        scale = EXTRA_TRIGGER_RADIUS / r
        y *= scale
        z *= scale

    x_offset = math.sqrt(max(0.0, EXTRA_TRIGGER_RADIUS ** 2 - y * y - z * z))

    j1 = math.atan2(y, x_offset)
    j2 = math.asin(clamp(-z / EXTRA_TRIGGER_RADIUS, -1.0, 1.0))

    v = np.array([j1, j2, 0.0, 0.0], dtype=float)

    for _ in range(60):
        current = _end_yz_from_vars(v)
        err = target - current

        if float(np.linalg.norm(err)) < 0.03:
            break

        # numerical Jacobian，只對 j1, j2 做微分
        J = np.zeros((2, 2), dtype=float)
        eps = 1e-4

        for col, idx in enumerate([0, 1]):
            vv = v.copy()
            vv[idx] += eps
            J[:, col] = (_end_yz_from_vars(vv) - current) / eps

        damping = 0.3
        A = J @ J.T + (damping * damping) * np.eye(2)
        step = J.T @ np.linalg.solve(A, err)

        v[0] += 0.7 * step[0]
        v[1] += 0.7 * step[1]

    return v


def _solve_extra_only(target_y, target_z, base_v):
    """
    圓外使用：
    固定 J1/J2，只調 extra_pitch / extra_yaw。
    """

    target = np.array([target_y, target_z], dtype=float)
    v = np.array(base_v, dtype=float)

    max_extra_pitch = math.radians(MAX_EXTRA_PITCH_DEG)
    max_extra_yaw = math.radians(MAX_EXTRA_YAW_DEG)

    for _ in range(80):
        current = _end_yz_from_vars(v)
        err = target - current

        if float(np.linalg.norm(err)) < 0.05:
            break

        # numerical Jacobian，只對 extra_pitch, extra_yaw 做微分
        J = np.zeros((2, 2), dtype=float)
        eps = 1e-4

        for col, idx in enumerate([2, 3]):
            vv = v.copy()
            vv[idx] += eps
            J[:, col] = (_end_yz_from_vars(vv) - current) / eps

        damping = 0.4
        A = J @ J.T + (damping * damping) * np.eye(2)
        step = J.T @ np.linalg.solve(A, err)

        v[2] += 0.7 * step[0]
        v[3] += 0.7 * step[1]

        # 只限制 J3/J4 的額外偏移量
        v[2] = clamp(v[2], -max_extra_pitch, max_extra_pitch)
        v[3] = clamp(v[3], -max_extra_yaw, max_extra_yaw)

    return v


def ik_keep_forward_yz(target_y, target_z):
    """
    有 J2/J3 forward offset 的分段 IK。

    圓內：
        extra_pitch = 0
        extra_yaw = 0
        只調 J1/J2

    圓外：
        先把 J1/J2 解到原本圓邊
        再只調 J3/J4 的 extra_pitch / extra_yaw
    """

    target_y = float(target_y)
    target_z = float(target_z)

    r = math.sqrt(target_y * target_y + target_z * target_z)

    # =========================
    # Case 1: 目標在原本圓內
    # =========================
    if r <= EXTRA_TRIGGER_RADIUS + 1e-9:
        v = _solve_j1_j2_keep_forward(target_y, target_z)
        mode = "inside circle: keep forward"

    # =========================
    # Case 2: 目標超出原本圓
    # =========================
    else:
        # 先取圓邊上的點
        boundary_y = target_y * EXTRA_TRIGGER_RADIUS / r
        boundary_z = target_z * EXTRA_TRIGGER_RADIUS / r

        # J1/J2 只負責到圓邊
        base_v = _solve_j1_j2_keep_forward(boundary_y, boundary_z)

        # 超出圓的部分，只讓 J3/J4 extra 去補
        v = _solve_extra_only(target_y, target_z, base_v)
        mode = "outside circle: use J3/J4 extra"

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
    angles["mode"] = mode

    return angles

# =========================
# FK：把 J1~J4 畫出來
# =========================


def forward_kinematics(angles):
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

    slider_y = Slider(slider_y_ax, "target_y cm", -50, 50, valinit=0)
    slider_z = Slider(slider_z_ax, "target_z cm", -50, 50, valinit=0)

    def update(_=None):
        target_y = slider_y.val
        target_z = slider_z.val

        angles = ik_keep_forward_yz(target_y, target_z)
        points, rod_start, rod_end, end_forward = forward_kinematics(angles)

        # =========================
        # 3D 圖
        # =========================
        ax3d.clear()

        xs = points[:, 0]
        ys = points[:, 1]
        zs = points[:, 2]

        ax3d.plot(xs, ys, zs, marker="o", linewidth=3)

        labels = ["Base", "J1", "J2 end", "J3/J4", "Rod start", "Ice cream"]
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

        ax3d.set_xlim(70, 130)
        ax3d.set_ylim(-40, 40)
        ax3d.set_zlim(-40, 40)
        ax3d.set_xlabel("X cm")
        ax3d.set_ylabel("Y cm")
        ax3d.set_zlabel("Z cm")
        set_axes_equal(ax3d)

        ax3d.set_title(
            "3D Arm IK Viewer\n"
            f"J1={angles['J1']:.1f}, J2={angles['J2']:.1f}, "
            f"J3={angles['J3']:.1f}, J4={angles['J4']:.1f}\n"
            f"End=({rod_end[0]:.1f}, {rod_end[1]:.1f}, {rod_end[2]:.1f}) cm"
        )

        # =========================
        # 2D Y-Z 平面圖
        # =========================
        ax2d.clear()

        circle = plt.Circle((0, 0), L_SHORT, fill=False, linestyle="--")
        ax2d.add_patch(circle)

        ax2d.scatter([target_y], [target_z], s=80, label="Target y/z")
        ax2d.scatter([rod_end[1]], [rod_end[2]], s=80, label="Reached y/z")

        ax2d.axhline(0, linewidth=1)
        ax2d.axvline(0, linewidth=1)

        ax2d.set_xlim(-50, 50)
        ax2d.set_ylim(-50, 50)
        ax2d.set_aspect("equal", adjustable="box")
        ax2d.set_xlabel("Y cm, left/right")
        ax2d.set_ylabel("Z cm, up/down")
        ax2d.set_title("Camera-like Y-Z View")
        ax2d.legend()

        fig.canvas.draw_idle()

    slider_y.on_changed(update)
    slider_z.on_changed(update)

    update()
    plt.show()


    import csv


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

    輸出欄位：
    - target_y, target_z
    - J1, J2, J3, J4
    - extra_pitch, extra_yaw
    - reached_y, reached_z
    - error_cm
    - reachable
    - mode
    """

    rows = []

    for y in range(y_min, y_max + 1):
        for z in range(z_min, z_max + 1):
            angles = ik_keep_forward_yz(float(y), float(z))

            points, rod_start, rod_end, end_forward = forward_kinematics(angles)

            reached_y = float(rod_end[1])
            reached_z = float(rod_end[2])

            error_cm = math.sqrt(
                (reached_y - y) ** 2 +
                (reached_z - z) ** 2
            )

            reachable = error_cm <= reachable_error_threshold_cm

            row = {
                "target_y_cm": y,
                "target_z_cm": z,

                "J1_deg": angles["J1"],
                "J2_deg": angles["J2"],
                "J3_deg": angles["J3"],
                "J4_deg": angles["J4"],

                "extra_pitch_deg": angles.get("extra_pitch", 0.0),
                "extra_yaw_deg": angles.get("extra_yaw", 0.0),

                "reached_y_cm": reached_y,
                "reached_z_cm": reached_z,
                "error_cm": error_cm,

                "reachable": reachable,
                "mode": angles.get("mode", ""),
            }

            rows.append(row)

    fieldnames = [
        "target_y_cm",
        "target_z_cm",
        "J1_deg",
        "J2_deg",
        "J3_deg",
        "J4_deg",
        "extra_pitch_deg",
        "extra_yaw_deg",
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
    #generate_ik_angle_table(
     #   y_min=-40,
     #   y_max=40,
     #   z_min=-40,
     #   z_max=40,
     #   output_path="ik_angle_table.csv",
    #)
    main()
