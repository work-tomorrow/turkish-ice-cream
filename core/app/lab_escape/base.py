from __future__ import annotations

from core.app.lab_escape.common import AlgoResult, Vec2, WorldConfig


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
