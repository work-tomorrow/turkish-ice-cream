"""
kinematics_helper.py

Core API 用的 IK helper。

座標定義：
    這份 helper 的 XYZ 都是 robot frame，不是 camera frame，也不是
    lab escape algorithm 的 2D screen frame。

    x: 前後
    y: 左右
    z: 上下

這份檔案是從 ik_visualizer_smooth.py 抽出來的「純計算版本」：
- 沒有 matplotlib
- 沒有 Slider
- 沒有 CSV
- 可以直接被 FastAPI / core control loop import
- 內建 warm start，連續呼叫時角度比較不會跳

機構假設：
    J1 yaw
    J2 pitch
    J2 forward offset
    L_SHORT
    J3 pitch
    J3 forward offset
    J4 yaw
    L_ROD

保持朝前的基本約束：
    J3 = -J2 + extra_pitch
    J4 = -J1 + extra_yaw

這版已整合 robust IK：
- L_ROD 預設 30 cm
- 目標是半徑 25 cm 圓內盡量穩定解出
- 如果 J2 撞到硬體限制，即使還在原本 trigger radius 內，也會啟用 extra
- 使用 multi-start numerical IK，避免某些區域卡在局部解
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np


# =========================
# Data models
# =========================


@dataclass(frozen=True)
class JointLimit:
    """
    單一馬達的設定。

    model angle:
        IK / FK 內部使用的角度。

    motor angle:
        實際送給 MotorManager / 馬達的角度。

    轉換公式：
        motor = zero_deg + sign * model

    sign:
        1.0 代表馬達正方向跟 model 相同。
        -1.0 代表馬達正方向跟 model 相反。

    min_deg / max_deg:
        馬達實際允許角度範圍，單位 degree。
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


@dataclass(frozen=True)
class ArmIKConfig:
    # 單位：cm
    l_short: float = 15.0
    l_rod: float = 30.0
    j2_forward_offset: float = 5.0
    j3_forward_offset: float = 5.0

    # 這是希望穩定可用的 Y-Z 半徑，不是硬限制。
    target_good_radius: float = 25.0

    # extra 啟用範圍，單位：cm
    extra_trigger_radius: float = 15.0
    extra_full_radius: float = 20.0

    # extra 最大角度，單位：degree
    # L_ROD 改成 30 cm 後，30 度比 15 度更容易保證半徑 25 cm 內穩定。
    max_extra_pitch_deg: float = 30.0
    max_extra_yaw_deg: float = 30.0

    # 每顆馬達的實體角度範圍與方向
    # 這裡的 min/max 是「實際送給馬達」的角度，不是 model angle。
    # 如果某顆馬達方向反了，把 sign 設成 -1.0。
    joint_limits: dict[str, JointLimit] = field(
        default_factory=lambda: {
            "J1": JointLimit(min_deg=-180.0, max_deg=180.0, sign=1.0, zero_deg=0.0),
            # J2 實體方向反了；目前依你提供的檔案保留 motor J2 min=-30。
            # 若硬體實測是 -40，改成 min_deg=-40.0 即可。
            "J2": JointLimit(min_deg=-30.0, max_deg=180.0, sign=-1.0, zero_deg=0.0),
            "J3": JointLimit(min_deg=-180.0, max_deg=180.0, sign=1.0, zero_deg=0.0),
            "J4": JointLimit(min_deg=-180.0, max_deg=180.0, sign=1.0, zero_deg=0.0),
        }
    )

    # solver 參數
    max_iterations: int = 120
    stop_error_cm: float = 0.03
    numerical_eps: float = 1e-4
    step_scale: float = 0.6

    # multi-start 設定
    use_multistart: bool = True

    # regularization；越大代表越不想用該變數動
    j1_penalty: float = 0.008
    j2_penalty: float = 0.008
    extra_pitch_penalty: float = 0.02
    extra_yaw_penalty: float = 0.02

    # 如果 J2 因為 joint limit 被 clamp，extra 會被打開。
    # raw J2 和 clamped J2 差到這個角度以上時，limit activation = 1。
    joint_limit_activation_full_deg: float = 8.0


@dataclass(frozen=True)
class IKResult:
    # target
    target_y_cm: float
    target_z_cm: float

    # model angles，單位：degree，用於 FK / debug
    angles_deg: dict[str, float]

    # motor angles，單位：degree，實際送給 MotorManager
    motor_angles_deg: dict[str, float]

    # debug info
    extra_pitch_deg: float
    extra_yaw_deg: float
    activation: float
    radial_activation: float
    limit_activation: float
    reached_y_cm: float
    reached_z_cm: float
    reached_x_cm: float
    error_cm: float
    mode: str
    reachable: bool

    # 下一次 warm start 用；不要直接回傳給前端也可以
    solver_state: list[float]

    def motor_targets(self) -> dict[str, float]:
        """
        給 MotorManager.set_target_angles() 使用。
        """
        return dict(self.motor_angles_deg)

    def as_dict(self, include_solver_state: bool = False) -> dict[str, Any]:
        data: dict[str, Any] = {
            "target_y_cm": self.target_y_cm,
            "target_z_cm": self.target_z_cm,
            "angles_deg": dict(self.angles_deg),
            "motor_angles_deg": dict(self.motor_angles_deg),
            "motor_targets": self.motor_targets(),
            "extra_pitch_deg": self.extra_pitch_deg,
            "extra_yaw_deg": self.extra_yaw_deg,
            "activation": self.activation,
            "radial_activation": self.radial_activation,
            "limit_activation": self.limit_activation,
            "reached_x_cm": self.reached_x_cm,
            "reached_y_cm": self.reached_y_cm,
            "reached_z_cm": self.reached_z_cm,
            "error_cm": self.error_cm,
            "reachable": self.reachable,
            "mode": self.mode,
        }

        if include_solver_state:
            data["solver_state"] = list(self.solver_state)

        return data


# =========================
# Helper functions
# =========================


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    if edge0 == edge1:
        return 1.0 if x >= edge1 else 0.0

    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def rot_y(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rot_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def trans_x(length: float) -> np.ndarray:
    return np.array(
        [
            [1.0, 0.0, 0.0, length],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


# =========================
# Main helper
# =========================


class ArmIKHelper:
    """
    Core API / control loop 應該建立一個 singleton 使用。

    用法：
        ik = ArmIKHelper()

        result = ik.solve_yz(target_y_cm=8.0, target_z_cm=5.0)
        motor_targets = result.motor_targets()

        manager.set_target_angles(motor_targets)

    注意：
        這個 class 會保存上一次 solver_state。
        連續呼叫時，角度會比每次從零開始更平滑。
    """

    def __init__(self, config: ArmIKConfig | None = None) -> None:
        self.config = config or ArmIKConfig()
        self._last_v: np.ndarray | None = None

    def reset(self) -> None:
        """
        session start / stop / calibration 後可以呼叫，
        避免沿用上一段動作的 solver state。
        """
        self._last_v = None

    def approximate_extra_reach_cm(self) -> float:
        """
        粗估 J3/J4 extra 在 Y-Z 平面能提供的額外位移。
        """
        cfg = self.config
        max_extra_deg = max(cfg.max_extra_pitch_deg, cfg.max_extra_yaw_deg)
        effective_extra_length = cfg.l_rod + cfg.j3_forward_offset
        return effective_extra_length * math.sin(math.radians(max_extra_deg))

    def model_to_motor_angles(
        self, model_angles_deg: dict[str, float]
    ) -> dict[str, float]:
        output: dict[str, float] = {}
        for joint, angle in model_angles_deg.items():
            limit = self.config.joint_limits[joint]
            motor_deg = limit.model_to_motor(float(angle))
            output[joint] = limit.clamp_motor(motor_deg)
        return output

    def motor_to_model_angles(
        self,
        motor_angles_deg: dict[str, float],
    ) -> dict[str, float]:
        output: dict[str, float] = {}
        for joint, angle in motor_angles_deg.items():
            limit = self.config.joint_limits[joint]
            motor_deg = limit.clamp_motor(float(angle))
            output[joint] = limit.motor_to_model(motor_deg)
        return output

    def end_xyz_from_motor_angles(
        self,
        motor_angles_deg: dict[str, float],
    ) -> tuple[float, float, float]:
        model_angles = self.motor_to_model_angles(motor_angles_deg)
        _, _, rod_end, _ = self.forward_kinematics(model_angles)
        return (
            float(rod_end[0]),
            float(rod_end[1]),
            float(rod_end[2]),
        )

    def solve_xyz(
        self,
        *,
        target_x_cm: float,
        target_y_cm: float,
        target_z_cm: float,
        warm_start: bool = True,
        update_state: bool = True,
        reachable_error_threshold_cm: float = 0.5,
    ) -> IKResult:
        """
        Solve a robot-frame XYZ target.

        The current mechanism solver controls the Y/Z projection. target_x_cm
        is accepted to keep the public control flow explicit, but it is not used
        by the Y/Z-only IK solver yet.
        """
        _ = target_x_cm
        return self.solve_yz(
            target_y_cm=target_y_cm,
            target_z_cm=target_z_cm,
            warm_start=warm_start,
            update_state=update_state,
            reachable_error_threshold_cm=reachable_error_threshold_cm,
        )

    def clamp_model_angle(self, joint: str, model_deg: float) -> float:
        """
        將 model angle 夾到該 joint 的實體馬達範圍內。
        """
        return self.config.joint_limits[joint].clamp_model(model_deg)

    def _clamp_v_to_joint_limits(
        self,
        v: np.ndarray,
        max_extra_pitch: float,
        max_extra_yaw: float,
    ) -> np.ndarray:
        """
        將 solver 內部變數 v 夾到每顆馬達的可動範圍。

        v = [j1, j2, extra_pitch, extra_yaw]，單位 rad。

        J1 = j1
        J2 = j2
        J3 = -j2 + extra_pitch
        J4 = -j1 + extra_yaw

        因此：
            extra_pitch = J3 + j2
            extra_yaw   = J4 + j1

        J3 / J4 的限制會轉成 extra_pitch / extra_yaw 的限制。
        """
        v = np.array(v, dtype=float)

        # J1, J2 是直接 joint angle
        j1_deg = math.degrees(float(v[0]))
        j2_deg = math.degrees(float(v[1]))

        j1_deg = self.clamp_model_angle("J1", j1_deg)
        j2_deg = self.clamp_model_angle("J2", j2_deg)

        v[0] = math.radians(j1_deg)
        v[1] = math.radians(j2_deg)

        # 先套用 extra 自己的 activation 上限
        v[2] = clamp(float(v[2]), -max_extra_pitch, max_extra_pitch)
        v[3] = clamp(float(v[3]), -max_extra_yaw, max_extra_yaw)

        # J3 = -J2 + extra_pitch
        j2_rad = float(v[1])
        j3_model_min_deg, j3_model_max_deg = self.config.joint_limits[
            "J3"
        ].model_bounds_deg()
        extra_pitch_min = math.radians(j3_model_min_deg) + j2_rad
        extra_pitch_max = math.radians(j3_model_max_deg) + j2_rad

        lo = max(-max_extra_pitch, min(extra_pitch_min, extra_pitch_max))
        hi = min(max_extra_pitch, max(extra_pitch_min, extra_pitch_max))
        if lo <= hi:
            v[2] = clamp(float(v[2]), lo, hi)
        else:
            # 沒有交集時，優先滿足 joint limit，留下 error。
            v[2] = clamp(
                float(v[2]),
                min(extra_pitch_min, extra_pitch_max),
                max(extra_pitch_min, extra_pitch_max),
            )

        # J4 = -J1 + extra_yaw
        j1_rad = float(v[0])
        j4_model_min_deg, j4_model_max_deg = self.config.joint_limits[
            "J4"
        ].model_bounds_deg()
        extra_yaw_min = math.radians(j4_model_min_deg) + j1_rad
        extra_yaw_max = math.radians(j4_model_max_deg) + j1_rad

        lo = max(-max_extra_yaw, min(extra_yaw_min, extra_yaw_max))
        hi = min(max_extra_yaw, max(extra_yaw_min, extra_yaw_max))
        if lo <= hi:
            v[3] = clamp(float(v[3]), lo, hi)
        else:
            # 沒有交集時，優先滿足 joint limit，留下 error。
            v[3] = clamp(
                float(v[3]),
                min(extra_yaw_min, extra_yaw_max),
                max(extra_yaw_min, extra_yaw_max),
            )

        return v

    def solve_yz(
        self,
        target_y_cm: float,
        target_z_cm: float,
        *,
        warm_start: bool = True,
        update_state: bool = True,
        reachable_error_threshold_cm: float = 0.5,
    ) -> IKResult:
        """
        解 Y-Z 平面目標點。

        target_y_cm:
            左右偏移，cm。

        target_z_cm:
            上下偏移，cm。

        warm_start:
            True 時使用上一筆解當 initial guess，角度比較連續。
            表格離線掃描或單點測試可以設 False。

        update_state:
            True 時會把本次 solver 結果存成下一次 warm start。

        reachable_error_threshold_cm:
            error 小於這個值時，reachable=True。
        """
        initial_v = self._last_v if warm_start else None
        result = self._solve_yz(
            float(target_y_cm),
            float(target_z_cm),
            initial_v=initial_v,
            reachable_error_threshold_cm=reachable_error_threshold_cm,
        )

        if update_state:
            self._last_v = np.array(result.solver_state, dtype=float)

        return result

    def forward_kinematics(
        self,
        angles_deg: dict[str, float],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        回傳：
            points, rod_start, rod_end, end_forward

        points:
            每個關鍵點的位置，shape = (6, 3)

        rod_end:
            末端位置 [x, y, z]，單位 cm

        end_forward:
            末端 local x 軸在 world 座標中的方向
        """
        cfg = self.config

        j1 = math.radians(float(angles_deg["J1"]))
        j2 = math.radians(float(angles_deg["J2"]))
        j3 = math.radians(float(angles_deg["J3"]))
        j4 = math.radians(float(angles_deg["J4"]))

        T = np.eye(4, dtype=float)

        points: list[np.ndarray] = []
        points.append(T[:3, 3].copy())  # base

        # J1 yaw
        T = T @ rot_z(j1)
        points.append(T[:3, 3].copy())

        # J2 pitch
        T = T @ rot_y(j2)

        # J2 forward offset
        T = T @ trans_x(cfg.j2_forward_offset)
        points.append(T[:3, 3].copy())

        # short link
        T = T @ trans_x(cfg.l_short)
        points.append(T[:3, 3].copy())

        # J3 pitch
        T = T @ rot_y(j3)

        # J3 forward offset
        T = T @ trans_x(cfg.j3_forward_offset)
        rod_start = T[:3, 3].copy()
        points.append(rod_start)

        # J4 yaw
        T = T @ rot_z(j4)

        # ice cream rod
        T = T @ trans_x(cfg.l_rod)
        rod_end = T[:3, 3].copy()
        points.append(rod_end)

        end_forward = T[:3, 0].copy()

        return np.array(points), rod_start, rod_end, end_forward

    def _angles_from_vars(self, v: np.ndarray) -> dict[str, float]:
        """
        v = [j1, j2, extra_pitch, extra_yaw]，單位 rad。
        """
        j1, j2, extra_pitch, extra_yaw = v

        j1_deg = math.degrees(float(j1))
        j2_deg = math.degrees(float(j2))
        j3_deg = math.degrees(float(-j2 + extra_pitch))
        j4_deg = math.degrees(float(-j1 + extra_yaw))

        return {
            "J1": normalize_deg(j1_deg),
            "J2": normalize_deg(j2_deg),
            "J3": normalize_deg(j3_deg),
            "J4": normalize_deg(j4_deg),
            "extra_pitch": math.degrees(float(extra_pitch)),
            "extra_yaw": math.degrees(float(extra_yaw)),
        }

    def _end_yz_from_vars(self, v: np.ndarray) -> np.ndarray:
        angles = self._angles_from_vars(v)
        _, _, rod_end, _ = self.forward_kinematics(angles)
        return np.array([rod_end[1], rod_end[2]], dtype=float)

    def _raw_initial_guess_from_target(
        self,
        target_y: float,
        target_z: float,
    ) -> np.ndarray:
        """
        不套 joint limit 的初始猜測，用來判斷 J2 是否撞到限制。
        """
        cfg = self.config

        pitch_len = cfg.j2_forward_offset + cfg.l_short
        yaw_extra_len = cfg.j3_forward_offset

        j2 = math.asin(clamp(-target_z / pitch_len, -1.0, 1.0))
        effective_y_radius = pitch_len * math.cos(j2) + yaw_extra_len

        if abs(effective_y_radius) < 1e-9:
            j1 = 0.0
        else:
            j1 = math.asin(clamp(target_y / effective_y_radius, -1.0, 1.0))

        return np.array([j1, j2, 0.0, 0.0], dtype=float)

    def _initial_guess_from_target(
        self,
        target_y: float,
        target_z: float,
        max_extra_pitch: float,
        max_extra_yaw: float,
    ) -> np.ndarray:
        v = self._raw_initial_guess_from_target(target_y, target_z)
        return self._clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)

    def _joint_limit_activation(self, target_y: float, target_z: float) -> float:
        """
        原本 activation 只看半徑，導致 target 在半徑 15 內時 extra 被關掉。
        但如果 J2 已經撞到硬體限制，半徑 15 內也可能需要 J3 extra 幫忙。

        這裡用 raw J2 和被 joint limit clamp 後的 J2 差距，平滑打開 extra。
        """
        cfg = self.config
        raw_v = self._raw_initial_guess_from_target(target_y, target_z)
        raw_j2_deg = math.degrees(float(raw_v[1]))
        clamped_j2_deg = self.clamp_model_angle("J2", raw_j2_deg)

        diff_deg = abs(raw_j2_deg - clamped_j2_deg)
        return smoothstep(0.0, cfg.joint_limit_activation_full_deg, diff_deg)

    def _make_initial_candidates(
        self,
        target_y: float,
        target_z: float,
        max_extra_pitch: float,
        max_extra_yaw: float,
        initial_v: np.ndarray | None,
    ) -> list[np.ndarray]:
        """
        多組 initial guess，避免 numerical IK 卡在局部解或 joint limit 邊界。
        """
        cfg = self.config
        candidates: list[np.ndarray] = []

        if initial_v is not None:
            candidates.append(np.array(initial_v, dtype=float))

        base = self._initial_guess_from_target(
            target_y,
            target_z,
            max_extra_pitch,
            max_extra_yaw,
        )
        candidates.append(base)

        effective_len = max(1e-6, cfg.l_rod + cfg.j3_forward_offset)
        ep_guess = math.asin(
            clamp(
                -target_z / effective_len,
                -math.sin(max_extra_pitch),
                math.sin(max_extra_pitch),
            )
        )
        ey_guess = math.asin(
            clamp(
                target_y / effective_len,
                -math.sin(max_extra_yaw),
                math.sin(max_extra_yaw),
            )
        )

        v_guess = base.copy()
        v_guess[2] = ep_guess
        v_guess[3] = ey_guess
        candidates.append(
            self._clamp_v_to_joint_limits(v_guess, max_extra_pitch, max_extra_yaw)
        )

        for ep in (-max_extra_pitch, 0.0, max_extra_pitch):
            for ey in (-max_extra_yaw, 0.0, max_extra_yaw):
                v = base.copy()
                v[2] = ep
                v[3] = ey
                candidates.append(
                    self._clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)
                )

        unique: list[np.ndarray] = []
        seen: set[tuple[float, ...]] = set()

        for v in candidates:
            key = tuple(round(float(x), 5) for x in v)
            if key not in seen:
                unique.append(v)
                seen.add(key)

        return unique

    def _solve_numerical_ik_once(
        self,
        target: np.ndarray,
        v: np.ndarray,
        max_extra_pitch: float,
        max_extra_yaw: float,
    ) -> tuple[float, np.ndarray, dict[str, float], np.ndarray]:
        cfg = self.config
        v = self._clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)

        for _ in range(cfg.max_iterations):
            current = self._end_yz_from_vars(v)
            err = target - current

            if float(np.linalg.norm(err)) < cfg.stop_error_cm:
                break

            # numerical Jacobian: 2 x 4
            J = np.zeros((2, 4), dtype=float)

            for i in range(4):
                vv = v.copy()
                vv[i] += cfg.numerical_eps
                vv = self._clamp_v_to_joint_limits(
                    vv,
                    max_extra_pitch,
                    max_extra_yaw,
                )
                J[:, i] = (
                    self._end_yz_from_vars(vv) - current
                ) / cfg.numerical_eps

            reg = np.diag(
                [
                    cfg.j1_penalty,
                    cfg.j2_penalty,
                    cfg.extra_pitch_penalty,
                    cfg.extra_yaw_penalty,
                ]
            )

            A = J.T @ J + reg + 1e-6 * np.eye(4, dtype=float)
            b = J.T @ err

            try:
                step = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                step = np.linalg.pinv(A) @ b

            v += cfg.step_scale * step

            # 避免 solver 跑到太誇張的角度分支。
            v[0] = (float(v[0]) + math.pi) % (2.0 * math.pi) - math.pi
            v[1] = (float(v[1]) + math.pi) % (2.0 * math.pi) - math.pi

            # 套用每顆馬達的角度範圍限制。
            v = self._clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)

        v = self._clamp_v_to_joint_limits(v, max_extra_pitch, max_extra_yaw)
        raw_angles = self._angles_from_vars(v)

        angles_deg = {
            "J1": raw_angles["J1"],
            "J2": raw_angles["J2"],
            "J3": raw_angles["J3"],
            "J4": raw_angles["J4"],
        }

        _, _, rod_end, _ = self.forward_kinematics(angles_deg)

        error_cm = math.sqrt(
            (float(rod_end[1]) - float(target[0])) ** 2
            + (float(rod_end[2]) - float(target[1])) ** 2
        )

        return error_cm, v, raw_angles, rod_end

    def _solve_yz(
        self,
        target_y: float,
        target_z: float,
        *,
        initial_v: np.ndarray | None,
        reachable_error_threshold_cm: float,
    ) -> IKResult:
        cfg = self.config

        target = np.array([target_y, target_z], dtype=float)
        r = float(np.linalg.norm(target))

        radial_activation = smoothstep(
            cfg.extra_trigger_radius,
            cfg.extra_full_radius,
            r,
        )

        limit_activation = self._joint_limit_activation(target_y, target_z)

        activation = max(radial_activation, limit_activation)

        max_extra_pitch = math.radians(cfg.max_extra_pitch_deg) * activation
        max_extra_yaw = math.radians(cfg.max_extra_yaw_deg) * activation

        candidates = self._make_initial_candidates(
            target_y,
            target_z,
            max_extra_pitch,
            max_extra_yaw,
            initial_v=initial_v if cfg.use_multistart else None,
        )

        best: tuple[float, np.ndarray, dict[str, float], np.ndarray] | None = None

        for candidate in candidates:
            result = self._solve_numerical_ik_once(
                target,
                candidate,
                max_extra_pitch,
                max_extra_yaw,
            )

            if best is None or result[0] < best[0]:
                best = result

            if result[0] < cfg.stop_error_cm:
                break

            if not cfg.use_multistart:
                break

        if best is None:
            raise RuntimeError("IK solver failed to produce a candidate")

        error_cm, v, raw_angles, rod_end = best

        angles_deg = {
            "J1": raw_angles["J1"],
            "J2": raw_angles["J2"],
            "J3": raw_angles["J3"],
            "J4": raw_angles["J4"],
        }

        motor_angles_deg = self.model_to_motor_angles(angles_deg)

        reached_x = float(rod_end[0])
        reached_y = float(rod_end[1])
        reached_z = float(rod_end[2])

        if activation <= 1e-6:
            mode = "inside: extra disabled"
        elif limit_activation > radial_activation + 1e-6:
            mode = "joint-limit assist: extra enabled"
        elif activation >= 1.0 - 1e-6:
            mode = "outside: extra fully enabled"
        else:
            mode = "transition: extra smoothly enabled"

        return IKResult(
            target_y_cm=target_y,
            target_z_cm=target_z,
            angles_deg=angles_deg,
            motor_angles_deg=motor_angles_deg,
            extra_pitch_deg=raw_angles["extra_pitch"],
            extra_yaw_deg=raw_angles["extra_yaw"],
            activation=activation,
            radial_activation=radial_activation,
            limit_activation=limit_activation,
            reached_x_cm=reached_x,
            reached_y_cm=reached_y,
            reached_z_cm=reached_z,
            error_cm=error_cm,
            mode=mode,
            reachable=error_cm <= reachable_error_threshold_cm,
            solver_state=[float(x) for x in v],
        )


# =========================
# Optional standalone test
# =========================

if __name__ == "__main__":
    ik = ArmIKHelper()

    for y, z in [(0, 0), (10, 0), (20, 0), (0, -20), (25, 0), (-25, 0), (15, -15), (-25, -15)]:
        result = ik.solve_yz(y, z, warm_start=False)
        print(f"target=({y}, {z})")
        print(result.as_dict())
        print()
