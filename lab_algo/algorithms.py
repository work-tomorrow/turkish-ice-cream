from __future__ import annotations

import math
import random
from enum import Enum, auto

from common import (
    ACTIVE_BORDER_COLOR,
    ACTIVE_COLOR,
    BLIND_BORDER_COLOR,
    BLIND_COLOR,
    BOUNDARY_BORDER_COLOR,
    BOUNDARY_COLOR,
    IDLE_BORDER_COLOR,
    IDLE_COLOR,
    RETREAT_BORDER_COLOR,
    RETREAT_COLOR,
    AlgoResult,
    Vec2,
    WorldConfig,
    clamp,
    clamp_position,
    distance,
    move_toward,
)

from eight_direction_motion import (
    EightDirMode,
    run_eight_direction_motion,
)


class BaseAlgo:
    def on_enter(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
    ):
        pass

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        raise NotImplementedError


class IdleTeaseAlgo(BaseAlgo):
    """
    IDLE：
    不再完全靜止，而是在原地小幅來回晃動，
    模擬土耳其冰淇淋的挑釁動作。
    """

    def __init__(
        self,
        amplitude_x: float = 35.0,
        amplitude_y: float = 10.0,
        frequency: float = 1.5,
        follow_speed: float = 500.0,
    ):
        self.amplitude_x = amplitude_x
        self.amplitude_y = amplitude_y
        self.frequency = frequency
        self.follow_speed = follow_speed

        self.anchor: Vec2 | None = None
        self.t = 0.0

    def on_enter(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
    ):
        self.anchor = ice_pos
        self.t = 0.0

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        if self.anchor is None:
            self.anchor = ice_pos

        self.t += dt

        phase = 2.0 * math.pi * self.frequency * self.t

        offset = Vec2(
            math.sin(phase) * self.amplitude_x,
            math.sin(phase * 2.0) * self.amplitude_y,
        )

        target_pos = self.anchor + offset
        target_pos = clamp_position(
            target_pos,
            config.ice_radius,
            config.width,
            config.height,
        )

        next_pos = move_toward(
            current=ice_pos,
            target=target_pos,
            max_distance=self.follow_speed * dt,
        )

        return AlgoResult(
            pos=next_pos,
            color=IDLE_COLOR,
            border_color=IDLE_BORDER_COLOR,
            status="IDLE / teasing wiggle",
        )


class NormalRandom8DirAlgo(BaseAlgo):
    """
    NORMAL_ESCAPE：
    使用 random valid 8-direction motion。

    為了避免抖動：
    1. 不是每一 frame 都重新選方向。
    2. 方向會維持一小段時間。
    3. 即使時間還沒到，也不是 100% 鎖死，有小機率換方向。
    """

    def __init__(
        self,
        move_speed: float = 420.0,
        direction_hold_time_min: float = 0.16,
        direction_hold_time_max: float = 0.28,
        keep_direction_probability: float = 0.88,
    ):
        self.move_speed = move_speed

        self.direction_hold_time_min = direction_hold_time_min
        self.direction_hold_time_max = direction_hold_time_max
        self.keep_direction_probability = keep_direction_probability

        self.current_direction: Vec2 | None = None
        self.direction_timer = 0.0

    def on_enter(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
    ):
        self.current_direction = None
        self.direction_timer = 0.0

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        self.direction_timer = max(0.0, self.direction_timer - dt)

        locked_direction = None

        if (
            self.current_direction is not None
            and self.direction_timer > 0.0
            and random.random() < self.keep_direction_probability
        ):
            locked_direction = self.current_direction

        command = run_eight_direction_motion(
            current_pos=ice_pos,
            danger_pos=hand_pos,
            config=config,
            dt=dt,
            move_speed=self.move_speed,
            mode=EightDirMode.RANDOM_VALID,
            locked_direction=locked_direction,
        )

        if command.direction.length() > 0:
            direction_changed = (
                self.current_direction is None
                or command.direction != self.current_direction
            )

            if direction_changed or self.direction_timer <= 0.0:
                self.current_direction = command.direction
                self.direction_timer = random.uniform(
                    self.direction_hold_time_min,
                    self.direction_hold_time_max,
                )

        return AlgoResult(
            pos=command.next_pos,
            color=ACTIVE_COLOR,
            border_color=ACTIVE_BORDER_COLOR,
            status=(
                f"NORMAL_ESCAPE / {command.reason} / "
                f"hold={self.direction_timer:.2f}s"
            ),
        )


class NormalWeightedEscapeAlgo(BaseAlgo):
    """
    NORMAL_ESCAPE 的另一種版本：
    使用 weighted 8-direction motion。
    """

    def __init__(self, move_speed: float = 420.0):
        self.move_speed = move_speed

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        command = run_eight_direction_motion(
            current_pos=ice_pos,
            danger_pos=hand_pos,
            config=config,
            dt=dt,
            move_speed=self.move_speed,
            mode=EightDirMode.WEIGHTED_ESCAPE,
        )

        return AlgoResult(
            pos=command.next_pos,
            color=ACTIVE_COLOR,
            border_color=ACTIVE_BORDER_COLOR,
            status=f"NORMAL_ESCAPE / {command.reason}",
        )


class BoundaryAware8DirAlgo(BaseAlgo):
    """
    BOUNDARY_ESCAPE：
    使用 boundary-aware 8-direction motion。
    """

    def __init__(self, move_speed: float = 450.0):
        self.move_speed = move_speed

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        command = run_eight_direction_motion(
            current_pos=ice_pos,
            danger_pos=hand_pos,
            config=config,
            dt=dt,
            move_speed=self.move_speed,
            mode=EightDirMode.BOUNDARY_AWARE,
        )

        return AlgoResult(
            pos=command.next_pos,
            color=BOUNDARY_COLOR,
            border_color=BOUNDARY_BORDER_COLOR,
            status=f"BOUNDARY_ESCAPE / {command.reason}",
        )


class RetreatPhase(Enum):
    UP = auto()
    WAIT = auto()
    DOWN = auto()


class CloseRetreatAlgo(BaseAlgo):
    """
    CLOSE_RETREAT：
    不再瞬移，而是分成三個階段：

    1. UP:
       非常快速往上抽離。

    2. WAIT:
       在上方停一下，表示冰淇淋暫時離開遊戲。

    3. DOWN:
       快速往下放到一個遠離手的隨機位置。
    """

    def __init__(
        self,
        up_speed: float = 1300.0,
        down_speed: float = 1100.0,
        wait_time: float = 0.35,
        reach_epsilon: float = 3.0,
    ):
        self.up_speed = up_speed
        self.down_speed = down_speed
        self.wait_time = wait_time
        self.reach_epsilon = reach_epsilon

        self.is_active = False
        self.phase = RetreatPhase.UP
        self.timer = 0.0

        self.up_target: Vec2 | None = None
        self.down_target: Vec2 | None = None

    def on_enter(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
    ):
        self.start(ice_pos, config)

    def start(
        self,
        ice_pos: Vec2,
        config: WorldConfig,
    ):
        self.is_active = True
        self.phase = RetreatPhase.UP
        self.timer = 0.0

        self.up_target = Vec2(
            clamp(
                ice_pos.x,
                config.ice_radius,
                config.width - config.ice_radius,
            ),
            config.ice_radius,
        )

        self.down_target = None

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        if not self.is_active:
            self.start(ice_pos, config)

        if self.phase == RetreatPhase.UP:
            return self._update_up(ice_pos, config, dt)

        if self.phase == RetreatPhase.WAIT:
            return self._update_wait(ice_pos, hand_pos, last_hand_pos, config, dt)

        if self.phase == RetreatPhase.DOWN:
            return self._update_down(ice_pos, config, dt)

        raise RuntimeError(f"Unknown retreat phase: {self.phase}")

    def _update_up(
        self,
        ice_pos: Vec2,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        assert self.up_target is not None

        next_pos = move_toward(
            current=ice_pos,
            target=self.up_target,
            max_distance=self.up_speed * dt,
        )

        if distance(next_pos, self.up_target) <= self.reach_epsilon:
            next_pos = self.up_target
            self.phase = RetreatPhase.WAIT
            self.timer = self.wait_time

        return AlgoResult(
            pos=next_pos,
            color=RETREAT_COLOR,
            border_color=RETREAT_BORDER_COLOR,
            status="CLOSE_RETREAT / moving up",
        )

    def _update_wait(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        self.timer -= dt

        if self.timer <= 0.0:
            ref_hand = hand_pos or last_hand_pos
            self.down_target = self._random_far_position(ref_hand, config)
            self.phase = RetreatPhase.DOWN

        return AlgoResult(
            pos=ice_pos,
            color=RETREAT_COLOR,
            border_color=RETREAT_BORDER_COLOR,
            status=f"CLOSE_RETREAT / wait {max(self.timer, 0.0):.2f}s",
        )

    def _update_down(
        self,
        ice_pos: Vec2,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        assert self.down_target is not None

        next_pos = move_toward(
            current=ice_pos,
            target=self.down_target,
            max_distance=self.down_speed * dt,
        )

        if distance(next_pos, self.down_target) <= self.reach_epsilon:
            next_pos = self.down_target
            self.is_active = False

            return AlgoResult(
                pos=next_pos,
                color=ACTIVE_COLOR,
                border_color=ACTIVE_BORDER_COLOR,
                status="CLOSE_RETREAT / finished lowering",
            )

        return AlgoResult(
            pos=next_pos,
            color=RETREAT_COLOR,
            border_color=RETREAT_BORDER_COLOR,
            status="CLOSE_RETREAT / moving down",
        )

    def _random_far_position(
        self,
        hand_pos: Vec2 | None,
        config: WorldConfig,
    ) -> Vec2:
        if hand_pos is None:
            return Vec2(config.width / 2, config.height / 2)

        min_dist = (
            config.hand_radius
            + config.ice_radius
            + config.escape_trigger_distance
        )

        for _ in range(100):
            p = Vec2(
                random.uniform(
                    config.ice_radius,
                    config.width - config.ice_radius,
                ),
                random.uniform(
                    config.ice_radius + 80,
                    config.height - config.ice_radius,
                ),
            )

            if distance(p, hand_pos) >= min_dist:
                return p

        corners = [
            Vec2(config.ice_radius, config.ice_radius + 80),
            Vec2(config.width - config.ice_radius, config.ice_radius + 80),
            Vec2(config.ice_radius, config.height - config.ice_radius),
            Vec2(config.width - config.ice_radius, config.height - config.ice_radius),
        ]

        return max(corners, key=lambda p: distance(p, hand_pos))


class BlindSpot8DirAlgo(BaseAlgo):
    """
    BLIND_SPOT_ESCAPE：
    camera 看不到手，但最後一次看到的位置接近 blind zone 時，
    假設手可能在盲區中，先遠離 last_hand_pos 或 blind zone center。
    """

    def __init__(self, move_speed: float = 400.0):
        self.move_speed = move_speed

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        last_hand_pos: Vec2 | None,
        config: WorldConfig,
        dt: float,
    ) -> AlgoResult:
        danger_pos = last_hand_pos or config.blind_zone.center()

        command = run_eight_direction_motion(
            current_pos=ice_pos,
            danger_pos=danger_pos,
            config=config,
            dt=dt,
            move_speed=self.move_speed,
            mode=EightDirMode.ESCAPE_FROM_POINT,
        )

        next_pos = clamp_position(
            command.next_pos,
            config.ice_radius,
            config.width,
            config.height,
        )

        return AlgoResult(
            pos=next_pos,
            color=BLIND_COLOR,
            border_color=BLIND_BORDER_COLOR,
            status=f"BLIND_SPOT_ESCAPE / {command.reason}",
        )