from __future__ import annotations

from core.app.lab_escape.algo_variants import create_algorithms
from core.app.lab_escape.common import (
    RobotState,
    StateDecision,
    Vec2,
    WorldConfig,
    distance,
    distance_from_nearest_wall_with_config,
)


class ArmStateMachine:
    """Smooth-only state machine.

    States decide *when* to use a behavior. The behavior itself is always the smooth algorithm set.
    There is no profile switching and no eight-direction boundary prediction.
    """

    def __init__(self, config: WorldConfig):
        self.config = config
        self.state = RobotState.IDLE
        self.state_time = 0.0
        self.transition_reason = "init"

        self.last_hand_pos: Vec2 | None = None
        self.previous_hand_pos: Vec2 | None = None
        self.time_since_hand_seen = 999.0
        self.hand_speed = 0.0
        self.hand_velocity = Vec2(0.0, 0.0)

        self.algorithms = create_algorithms()

        self.min_state_duration = {
            RobotState.IDLE: 0.0,
            RobotState.NORMAL_ESCAPE: 0.12,
            RobotState.BOUNDARY_ESCAPE: 0.25,
            RobotState.CLOSE_RETREAT: 0.0,
            RobotState.BLIND_SPOT_ESCAPE: 0.20,
        }

        self.state_priority = {
            RobotState.IDLE: 0,
            RobotState.NORMAL_ESCAPE: 10,
            RobotState.BLIND_SPOT_ESCAPE: 15,
            RobotState.BOUNDARY_ESCAPE: 20,
            RobotState.CLOSE_RETREAT: 30,
        }

    def update(self, ice_pos: Vec2, hand_pos: Vec2 | None, dt: float):
        self._update_hand_memory(hand_pos, dt)
        desired = self._decide_next_state(ice_pos, hand_pos)
        final = self._apply_transition_guard(desired)

        if final.next_state != self.state:
            self.state = final.next_state
            self.state_time = 0.0
            self.algorithms[self.state].on_enter(
                ice_pos=ice_pos,
                hand_pos=hand_pos,
                last_hand_pos=self.last_hand_pos,
                config=self.config,
            )
        else:
            self.state_time += dt

        self.transition_reason = final.reason
        return self.algorithms[self.state].update(
            ice_pos=ice_pos,
            hand_pos=hand_pos,
            last_hand_pos=self.last_hand_pos,
            config=self.config,
            dt=dt,
        )

    def _update_hand_memory(self, hand_pos: Vec2 | None, dt: float) -> None:
        if hand_pos is None:
            self.previous_hand_pos = None
            self.hand_velocity = Vec2(0.0, 0.0)
            self.hand_speed = 0.0
            self.time_since_hand_seen += dt
            return

        if self.last_hand_pos is not None and dt > 1e-6:
            delta = hand_pos - self.last_hand_pos
            self.hand_velocity = delta * (1.0 / dt)
            self.hand_speed = self.hand_velocity.length()
        else:
            self.hand_velocity = Vec2(0.0, 0.0)
            self.hand_speed = 0.0

        self.previous_hand_pos = self.last_hand_pos
        self.last_hand_pos = hand_pos
        self.time_since_hand_seen = 0.0

    def _decide_next_state(self, ice_pos: Vec2, hand_pos: Vec2 | None) -> StateDecision:
        close_algo = self.algorithms[RobotState.CLOSE_RETREAT]
        if getattr(close_algo, "is_active", False):
            return StateDecision(RobotState.CLOSE_RETREAT, "keep CLOSE_RETREAT until animation finishes")

        if hand_pos is None:
            if self._should_use_blind_spot_escape():
                return StateDecision(RobotState.BLIND_SPOT_ESCAPE, "hand lost near blind zone")
            if self.state == RobotState.BOUNDARY_ESCAPE and not self._safe_from_boundary(ice_pos):
                return StateDecision(RobotState.BOUNDARY_ESCAPE, "keep BOUNDARY_ESCAPE until safely away from boundary")
            return StateDecision(RobotState.IDLE, "no hand observed")

        hand_ice_distance = distance(ice_pos, hand_pos)

        if self.config.enable_close_retreat and hand_ice_distance <= self.config.close_retreat_distance_cm:
            return StateDecision(
                RobotState.CLOSE_RETREAT,
                f"hand too close: {hand_ice_distance:.1f} <= {self.config.close_retreat_distance_cm:.1f}",
            )

        if self.state == RobotState.BOUNDARY_ESCAPE and not self._safe_from_boundary(ice_pos):
            return StateDecision(RobotState.BOUNDARY_ESCAPE, "keep BOUNDARY_ESCAPE until safely away from boundary")

        if self._near_boundary(ice_pos):
            return StateDecision(RobotState.BOUNDARY_ESCAPE, "ice is near workspace boundary")

        if hand_ice_distance <= self.config.escape_trigger_distance_cm:
            return StateDecision(
                RobotState.NORMAL_ESCAPE,
                f"hand inside escape trigger distance: {hand_ice_distance:.1f} <= {self.config.escape_trigger_distance_cm:.1f}",
            )

        if self.hand_speed > self.config.hand_stationary_speed_cm_s and self._hand_is_moving_toward_ice(ice_pos, hand_pos):
            return StateDecision(RobotState.NORMAL_ESCAPE, "hand is moving toward ice")

        return StateDecision(RobotState.IDLE, "hand is far or not approaching")

    def _apply_transition_guard(self, decision: StateDecision) -> StateDecision:
        if decision.next_state == self.state:
            return decision

        min_duration = self.min_state_duration[self.state]
        current_priority = self.state_priority[self.state]
        next_priority = self.state_priority[decision.next_state]

        if self.state_time < min_duration and next_priority <= current_priority:
            return StateDecision(
                self.state,
                (
                    f"state locked: {self.state.name} for {self.state_time:.2f}s "
                    f"< min {min_duration:.2f}s; blocked transition to {decision.next_state.name}"
                ),
            )

        return decision

    def _near_boundary(self, ice_pos: Vec2) -> bool:
        return (
            distance_from_nearest_wall_with_config(ice_pos, self.config.ice_radius_cm, self.config)
            <= self.config.boundary_enter_margin_cm
        )

    def _safe_from_boundary(self, ice_pos: Vec2) -> bool:
        return (
            distance_from_nearest_wall_with_config(ice_pos, self.config.ice_radius_cm, self.config)
            >= self.config.boundary_exit_margin_cm
        )

    def _should_use_blind_spot_escape(self) -> bool:
        if self.last_hand_pos is None:
            return False
        if self.time_since_hand_seen > self.config.hand_lost_blind_timeout_s:
            return False
        assert self.config.blind_zone is not None
        return self.config.blind_zone.expanded(self.config.blind_zone_detection_margin_cm).contains(self.last_hand_pos)

    def _hand_is_moving_toward_ice(self, ice_pos: Vec2, hand_pos: Vec2) -> bool:
        if self.previous_hand_pos is None:
            return False
        previous_distance = distance(ice_pos, self.previous_hand_pos)
        current_distance = distance(ice_pos, hand_pos)
        return current_distance < previous_distance
