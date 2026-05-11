"""
speed_limit_xyz_estimator.py

用途：
    在「每顆馬達最多 25 deg/s」的限制下，估計目前姿態附近
    XYZ 位置差多少以內是合法的。

重點：
    你一定要指定 control loop 的 dt。
    例如 dt_s = 0.1 秒，25 deg/s 代表每次控制迴圈每顆馬達最多只能變：
        25 * 0.1 = 2.5 deg

這份 script 提供三種東西：

1. Jacobian local estimate
   快速估計目前姿態附近，dx/dy/dz 每軸大約可變多少。

2. sampling cloud
   直接枚舉每顆馬達在 ±max_delta_deg 內的變化，算出所有可能的 xyz delta。
   這比較接近真實 FK，但只是 sample，不是連續完整集合。

3. IK target checker
   給一個 target_y / target_z，先用 ArmIKHelper 解 IK，
   再檢查算出的 motor angle delta 是否小於速度限制。

使用方式：
    把這個檔案放在 kinematics_helper.py 同一層。
    然後執行：
        python speed_limit_xyz_estimator.py
"""

from __future__ import annotations

import csv
import math
from itertools import product
from typing import Any

import numpy as np

try:
    from kinematics_helper import ArmIKHelper
except ImportError:
    # 如果你檔名還是我前面給的 robust 版，可以用這個 fallback。
    from kinematics_helper_robust_lrod30 import ArmIKHelper


# =========================
# 你主要調這裡
# =========================

MAX_MOTOR_SPEED_DEG_PER_S = 250

# control loop 週期。
# 如果你的 core loop 是每 100 ms 呼叫一次，就用 0.1。
# 如果是每 50 ms 呼叫一次，就用 0.05。
DT_S = 0.1

# 目前實際馬達角度，單位 degree。
# 這裡應該填「motor angle」，不是 model angle。
CURRENT_MOTOR_ANGLES = {
    "J1": 0.0,
    "J2": 0.0,
    "J3": 0.0,
    "J4": 0.0,
}

# sampling 越大越準，但越慢。
# 9 表示每顆馬達取 9 個點，總共 9^4 = 6561 個 FK。
SAMPLES_PER_JOINT = 9

# 輸出所有合法 motor delta 對應的 xyz delta。
OUTPUT_CSV = "legal_xyz_delta_cloud.csv"

JOINTS = ["J1", "J2", "J3", "J4"]


# =========================
# 基本工具
# =========================

def motor_step_limit_deg(dt_s: float = DT_S) -> float:
    return MAX_MOTOR_SPEED_DEG_PER_S * dt_s


def xyz_from_motor_angles(
    ik: ArmIKHelper,
    motor_angles_deg: dict[str, float],
) -> np.ndarray:
    x, y, z = ik.end_xyz_from_motor_angles(motor_angles_deg)
    return np.array([x, y, z], dtype=float)


def is_motor_angle_inside_limit(
    ik: ArmIKHelper,
    joint: str,
    angle_deg: float,
) -> bool:
    limit = ik.config.joint_limits[joint]
    return limit.min_deg <= angle_deg <= limit.max_deg


def motor_delta_deg(
    target_deg: float,
    current_deg: float,
    *,
    use_shortest_angle: bool = False,
) -> float:
    """
    一般 servo / non-continuous joint 建議 use_shortest_angle=False。
    如果某顆馬達是可連續旋轉的 yaw joint，才考慮 shortest angle。
    """
    delta = target_deg - current_deg

    if use_shortest_angle:
        delta = (delta + 180.0) % 360.0 - 180.0

    return delta


def solver_state_from_motor_angles(
    ik: ArmIKHelper,
    motor_angles_deg: dict[str, float],
) -> np.ndarray:
    """
    將目前 motor angles 轉成 ArmIKHelper 內部 warm start 用的 v：

        v = [j1, j2, extra_pitch, extra_yaw]

    其中：
        J3 = -J2 + extra_pitch  => extra_pitch = J3 + J2
        J4 = -J1 + extra_yaw    => extra_yaw   = J4 + J1
    """
    model = ik.motor_to_model_angles(motor_angles_deg)

    j1 = math.radians(model["J1"])
    j2 = math.radians(model["J2"])

    extra_pitch = math.radians(model["J3"] + model["J2"])
    extra_yaw = math.radians(model["J4"] + model["J1"])

    return np.array([j1, j2, extra_pitch, extra_yaw], dtype=float)


# =========================
# 方法 1：local Jacobian 估計
# =========================

def compute_xyz_jacobian_cm_per_deg(
    ik: ArmIKHelper,
    current_motor_angles_deg: dict[str, float],
    eps_deg: float = 0.1,
) -> np.ndarray:
    """
    回傳 J，shape = (3, 4)

    J[:, i] 的意思：
        第 i 顆馬達角度增加 1 degree 時，
        end effector 的 [x, y, z] 大約改變多少 cm。

    單位：cm / degree
    """
    J = np.zeros((3, 4), dtype=float)

    for i, joint in enumerate(JOINTS):
        plus = dict(current_motor_angles_deg)
        minus = dict(current_motor_angles_deg)

        plus[joint] += eps_deg
        minus[joint] -= eps_deg

        if not is_motor_angle_inside_limit(ik, joint, plus[joint]):
            plus[joint] = current_motor_angles_deg[joint]

        if not is_motor_angle_inside_limit(ik, joint, minus[joint]):
            minus[joint] = current_motor_angles_deg[joint]

        p_plus = xyz_from_motor_angles(ik, plus)
        p_minus = xyz_from_motor_angles(ik, minus)

        denom = plus[joint] - minus[joint]
        if abs(denom) < 1e-9:
            J[:, i] = 0.0
        else:
            J[:, i] = (p_plus - p_minus) / denom

    return J


def local_delta_estimate(
    ik: ArmIKHelper,
    current_motor_angles_deg: dict[str, float],
    dt_s: float = DT_S,
    direction_samples: int = 20000,
) -> dict[str, Any]:
    """
    用 Jacobian 快速估計目前姿態附近的合法 xyz delta。

    回傳：
        axis_max_abs_cm:
            各軸最大可能變化量估計。
            例如 abs(dx) <= axis_max_abs_cm["dx"]。

        conservative_ball_radius_cm:
            近似的保守球半徑。
            如果 ||delta_xyz|| 小於這個數，比較有機會一定合法。
            這是 local linear approximation，不是全域保證。
    """
    J = compute_xyz_jacobian_cm_per_deg(ik, current_motor_angles_deg)

    max_step_deg = motor_step_limit_deg(dt_s)

    # 對 motor delta box: |dq_i| <= max_step_deg
    # 每軸最大值約為 sum_i |J_axis_i| * max_step_deg
    axis_max = np.sum(np.abs(J) * max_step_deg, axis=1)

    # 估計 local reachable zonoid 的 inradius:
    # support(u) = sum_i max_step_deg * abs(u^T J_i)
    rng = np.random.default_rng(0)
    directions = rng.normal(size=(direction_samples, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True) + 1e-12

    supports = []
    for u in directions:
        support = float(np.sum(np.abs(u @ J) * max_step_deg))
        supports.append(support)

    conservative_radius = float(min(supports))

    return {
        "dt_s": dt_s,
        "max_motor_speed_deg_per_s": MAX_MOTOR_SPEED_DEG_PER_S,
        "max_motor_step_deg": max_step_deg,
        "jacobian_cm_per_deg": J,
        "axis_max_abs_cm": {
            "dx": float(axis_max[0]),
            "dy": float(axis_max[1]),
            "dz": float(axis_max[2]),
        },
        "conservative_ball_radius_cm": conservative_radius,
    }


# =========================
# 方法 2：sampling motor delta cloud
# =========================

def sample_legal_xyz_delta_cloud(
    ik: ArmIKHelper,
    current_motor_angles_deg: dict[str, float],
    dt_s: float = DT_S,
    samples_per_joint: int = SAMPLES_PER_JOINT,
    output_csv: str | None = OUTPUT_CSV,
) -> dict[str, Any]:
    """
    枚舉每顆馬達在一個 control step 內可能的 angle delta，
    並用 FK 算出所有對應的 xyz delta。

    這直接回答：
        在目前姿態、速度限制、dt_s 下，一步內可能到達哪些 xyz 差值？

    注意：
        samples_per_joint 是離散 sample。
        它不代表完整連續集合，但很適合拿來看範圍。
    """
    max_step_deg = motor_step_limit_deg(dt_s)

    current_xyz = xyz_from_motor_angles(ik, current_motor_angles_deg)

    delta_values = np.linspace(-max_step_deg, max_step_deg, samples_per_joint)

    rows: list[dict[str, float]] = []
    deltas_xyz: list[np.ndarray] = []

    for dJ1, dJ2, dJ3, dJ4 in product(delta_values, repeat=4):
        motor_candidate = {
            "J1": current_motor_angles_deg["J1"] + float(dJ1),
            "J2": current_motor_angles_deg["J2"] + float(dJ2),
            "J3": current_motor_angles_deg["J3"] + float(dJ3),
            "J4": current_motor_angles_deg["J4"] + float(dJ4),
        }

        # 超出馬達實體角度範圍就跳過。
        if not all(
            is_motor_angle_inside_limit(ik, joint, motor_candidate[joint])
            for joint in JOINTS
        ):
            continue

        xyz = xyz_from_motor_angles(ik, motor_candidate)
        delta_xyz = xyz - current_xyz
        deltas_xyz.append(delta_xyz)

        rows.append(
            {
                "dJ1_deg": float(dJ1),
                "dJ2_deg": float(dJ2),
                "dJ3_deg": float(dJ3),
                "dJ4_deg": float(dJ4),
                "x_cm": float(xyz[0]),
                "y_cm": float(xyz[1]),
                "z_cm": float(xyz[2]),
                "dx_cm": float(delta_xyz[0]),
                "dy_cm": float(delta_xyz[1]),
                "dz_cm": float(delta_xyz[2]),
                "delta_norm_cm": float(np.linalg.norm(delta_xyz)),
            }
        )

    if not deltas_xyz:
        raise RuntimeError("No valid motor samples. Check current_motor_angles and joint limits.")

    delta_arr = np.vstack(deltas_xyz)

    min_delta = np.min(delta_arr, axis=0)
    max_delta = np.max(delta_arr, axis=0)
    abs_max_delta = np.max(np.abs(delta_arr), axis=0)

    norms = np.linalg.norm(delta_arr, axis=1)

    if output_csv is not None:
        with open(output_csv, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "dJ1_deg",
                "dJ2_deg",
                "dJ3_deg",
                "dJ4_deg",
                "x_cm",
                "y_cm",
                "z_cm",
                "dx_cm",
                "dy_cm",
                "dz_cm",
                "delta_norm_cm",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    return {
        "dt_s": dt_s,
        "max_motor_speed_deg_per_s": MAX_MOTOR_SPEED_DEG_PER_S,
        "max_motor_step_deg": max_step_deg,
        "current_xyz_cm": {
            "x": float(current_xyz[0]),
            "y": float(current_xyz[1]),
            "z": float(current_xyz[2]),
        },
        "sample_count": len(rows),
        "delta_min_cm": {
            "dx": float(min_delta[0]),
            "dy": float(min_delta[1]),
            "dz": float(min_delta[2]),
        },
        "delta_max_cm": {
            "dx": float(max_delta[0]),
            "dy": float(max_delta[1]),
            "dz": float(max_delta[2]),
        },
        "delta_abs_max_cm": {
            "dx": float(abs_max_delta[0]),
            "dy": float(abs_max_delta[1]),
            "dz": float(abs_max_delta[2]),
        },
        "max_delta_norm_cm": float(np.max(norms)),
        "mean_delta_norm_cm": float(np.mean(norms)),
        "output_csv": output_csv,
    }


# =========================
# 方法 3：檢查某個 IK target 是否符合速度限制
# =========================

def check_ik_yz_target_speed_legal(
    ik: ArmIKHelper,
    current_motor_angles_deg: dict[str, float],
    target_y_cm: float,
    target_z_cm: float,
    dt_s: float = DT_S,
    use_shortest_angle: bool = False,
) -> dict[str, Any]:
    """
    給一個 target_y / target_z：
        1. 用目前姿態當 warm start 解 IK
        2. 算 target motor angles
        3. 檢查每顆馬達是否都能在 dt_s 內到達
    """
    max_step_deg = motor_step_limit_deg(dt_s)

    current_v = solver_state_from_motor_angles(ik, current_motor_angles_deg)

    # 暫時指定 warm start，不改 ik 內部狀態。
    old_v = ik._last_v
    ik._last_v = current_v.copy()

    try:
        result = ik.solve_yz(
            target_y_cm=target_y_cm,
            target_z_cm=target_z_cm,
            warm_start=True,
            update_state=False,
        )
    finally:
        ik._last_v = old_v

    target_motor = result.motor_targets()

    motor_deltas = {
        joint: motor_delta_deg(
            target_motor[joint],
            current_motor_angles_deg[joint],
            use_shortest_angle=use_shortest_angle,
        )
        for joint in JOINTS
    }

    legal_by_joint = {
        joint: abs(delta) <= max_step_deg
        for joint, delta in motor_deltas.items()
    }

    legal = all(legal_by_joint.values())

    current_xyz = xyz_from_motor_angles(ik, current_motor_angles_deg)
    target_xyz = np.array(
        [
            result.reached_x_cm,
            result.reached_y_cm,
            result.reached_z_cm,
        ],
        dtype=float,
    )

    delta_xyz = target_xyz - current_xyz

    return {
        "legal": legal,
        "dt_s": dt_s,
        "max_motor_step_deg": max_step_deg,
        "current_motor_angles_deg": dict(current_motor_angles_deg),
        "target_motor_angles_deg": target_motor,
        "motor_deltas_deg": motor_deltas,
        "legal_by_joint": legal_by_joint,
        "current_xyz_cm": {
            "x": float(current_xyz[0]),
            "y": float(current_xyz[1]),
            "z": float(current_xyz[2]),
        },
        "target_reached_xyz_cm": {
            "x": float(target_xyz[0]),
            "y": float(target_xyz[1]),
            "z": float(target_xyz[2]),
        },
        "delta_xyz_cm": {
            "dx": float(delta_xyz[0]),
            "dy": float(delta_xyz[1]),
            "dz": float(delta_xyz[2]),
            "norm": float(np.linalg.norm(delta_xyz)),
        },
        "ik_error_cm": result.error_cm,
        "ik_reachable": result.reachable,
        "ik_mode": result.mode,
    }


# =========================
# Demo
# =========================

def main() -> None:
    ik = ArmIKHelper()

    print("=== Config ===")
    print(f"max speed = {MAX_MOTOR_SPEED_DEG_PER_S:.2f} deg/s")
    print(f"dt_s = {DT_S:.3f} s")
    print(f"max motor step = {motor_step_limit_deg(DT_S):.3f} deg")
    print(f"current motor angles = {CURRENT_MOTOR_ANGLES}")
    print()

    print("=== Local Jacobian estimate ===")
    local = local_delta_estimate(
        ik,
        CURRENT_MOTOR_ANGLES,
        dt_s=DT_S,
    )

    print("axis max abs delta, cm:")
    for k, v in local["axis_max_abs_cm"].items():
        print(f"  {k}: {v:.4f}")

    print(
        "conservative local ball radius ≈ "
        f"{local['conservative_ball_radius_cm']:.4f} cm"
    )
    print()

    print("=== Sampling legal XYZ delta cloud ===")
    cloud = sample_legal_xyz_delta_cloud(
        ik,
        CURRENT_MOTOR_ANGLES,
        dt_s=DT_S,
        samples_per_joint=SAMPLES_PER_JOINT,
        output_csv=OUTPUT_CSV,
    )

    print(f"sample count = {cloud['sample_count']}")
    print(f"current xyz = {cloud['current_xyz_cm']}")
    print("delta min cm =", cloud["delta_min_cm"])
    print("delta max cm =", cloud["delta_max_cm"])
    print("delta abs max cm =", cloud["delta_abs_max_cm"])
    print(f"max delta norm = {cloud['max_delta_norm_cm']:.4f} cm")
    print(f"csv saved to: {cloud['output_csv']}")
    print()

    print("=== Example target check ===")
    example = check_ik_yz_target_speed_legal(
        ik,
        CURRENT_MOTOR_ANGLES,
        target_y_cm=2.0,
        target_z_cm=-1.0,
        dt_s=DT_S,
    )
    print(example)


if __name__ == "__main__":
    main()
