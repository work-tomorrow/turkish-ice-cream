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


def _angles_from_vars(v):
    """
    v = [j1, j2, extra_pitch, extra_yaw]
    單位都是 rad
    """
    j1, j2, extra_pitch, extra_yaw = v

    return {
        "J1": math.degrees(j1),
        "J2": math.degrees(j2),

        # 原本保持朝前是 J3 = -J2
        # 現在超出圓後允許多一點 extra_pitch
        "J3": math.degrees(-j2 + extra_pitch),

        # 原本保持朝前是 J4 = -J1
        # 現在超出圓後允許多一點 extra_yaw
        "J4": math.degrees(-j1 + extra_yaw),

        "extra_pitch": math.degrees(extra_pitch),
        "extra_yaw": math.degrees(extra_yaw),
    }


def _end_yz_from_vars(v):
    """
    用目前的 forward_kinematics 算末端 y/z。
    """
    angles = _angles_from_vars(v)
    _, _, rod_end, _ = forward_kinematics(angles)
    return np.array([rod_end[1], rod_end[2]], dtype=float)


def ik_keep_forward_yz(target_y, target_z):
    """
    分段 IK：

    1. 如果 target 在半徑 L_SHORT 的圓內：
       - 不啟用 extra_pitch / extra_yaw
       - J3 = -J2
       - J4 = -J1

    2. 如果 target 超出圓：
       - 才允許 J3 多偏 extra_pitch
       - 才允許 J4 多偏 extra_yaw
       - extra_pitch / extra_yaw 都限制在 ±15 度
    """

    target = np.array([target_y, target_z], dtype=float)
    r = float(np.linalg.norm(target))

    max_extra_pitch = math.radians(MAX_EXTRA_PITCH_DEG)
    max_extra_yaw = math.radians(MAX_EXTRA_YAW_DEG)

    # =========================
    # Case 1: target 在原本圓內
    # =========================
    if r <= L_SHORT + 1e-9:
        y = target_y
        z = target_z

        x_offset = math.sqrt(max(0.0, L_SHORT * L_SHORT - y * y - z * z))

        J1 = math.atan2(y, x_offset)
        J2 = math.asin(clamp(-z / L_SHORT, -1.0, 1.0))

        extra_pitch = 0.0
        extra_yaw = 0.0

        angles = _angles_from_vars([J1, J2, extra_pitch, extra_yaw])
        angles["target_y"] = target_y
        angles["target_z"] = target_z
        angles["mode"] = "inside circle: keep forward"

        return angles

    # =========================
    # Case 2: target 超出原本圓
    # =========================

    # 先取原本圓上的邊界點
    boundary = target * (L_SHORT / r)
    by, bz = float(boundary[0]), float(boundary[1])

    bx = math.sqrt(max(0.0, L_SHORT * L_SHORT - by * by - bz * bz))

    J1 = math.atan2(by, bx)
    J2 = math.asin(clamp(-bz / L_SHORT, -1.0, 1.0))

    # 超出圓的那一段，交給 J3/J4 的 extra 去處理
    excess = target - boundary
    excess_y = float(excess[0])
    excess_z = float(excess[1])

    # 初始猜測：
    # J4 extra_yaw 幫忙推 Y
    # J3 extra_pitch 幫忙推 Z
    extra_yaw = math.asin(
        clamp(
            excess_y / L_ROD,
            -math.sin(max_extra_yaw),
            math.sin(max_extra_yaw),
        )
    )

    extra_pitch = math.asin(
        clamp(
            -excess_z / L_ROD,
            -math.sin(max_extra_pitch),
            math.sin(max_extra_pitch),
        )
    )

    v = np.array([J1, J2, extra_pitch, extra_yaw], dtype=float)

    # 用簡單 numerical IK 微調，讓 reached y/z 更接近 target
    for _ in range(40):
        current = _end_yz_from_vars(v)
        err = target - current

        if float(np.linalg.norm(err)) < 0.05:
            break

        # numerical Jacobian: 2 x 4
        J = np.zeros((2, 4), dtype=float)
        eps = 1e-4

        for i in range(4):
            vv = v.copy()
            vv[i] += eps
            J[:, i] = (_end_yz_from_vars(vv) - current) / eps

        # damped least squares
        damping = 0.2
        A = J @ J.T + (damping * damping) * np.eye(2)
        step = J.T @ np.linalg.solve(A, err)

        v += 0.7 * step

        # 只限制 extra_pitch / extra_yaw
        v[2] = clamp(v[2], -max_extra_pitch, max_extra_pitch)
        v[3] = clamp(v[3], -max_extra_yaw, max_extra_yaw)

    angles = _angles_from_vars(v)
    angles["target_y"] = target_y
    angles["target_z"] = target_z
    angles["mode"] = "outside circle: use J3/J4 extra"

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

    # J2 pitch + short link
    T = T @ rot_y(j2)
    T = T @ trans_x(L_SHORT)
    points.append(T[:3, 3].copy())

    # J3 pitch
    T = T @ rot_y(j3)
    points.append(T[:3, 3].copy())

    # J4 yaw + ice cream rod
    T = T @ rot_z(j4)
    rod_start = T[:3, 3].copy()
    T = T @ trans_x(L_ROD)
    rod_end = T[:3, 3].copy()

    points.append(rod_start)
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
    generate_ik_angle_table(
        y_min=-40,
        y_max=40,
        z_min=-40,
        z_max=40,
        output_path="ik_angle_table.csv",
    )
    main()
