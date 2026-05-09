from __future__ import annotations

from common import (
    RobotState,
    StateDecision,
    Vec2,
    WorldConfig,
    distance,
    distance_from_nearest_wall,
)

from algo_variants import create_algorithm_profile



ALGORITHM_PROFILE = "predictive_safe"
# 可選：
# ALGORITHM_PROFILE = "smooth"
# ALGORITHM_PROFILE = "performer"
# ALGORITHM_PROFILE = "safe"
# ALGORITHM_PROFILE = "predictive_safe"


class ArmStateMachine:
    def __init__(self, config: WorldConfig):
        self.config = config

        self.state = RobotState.IDLE
        self.state_time = 0.0
        self.transition_reason = "init"

        self.last_hand_pos: Vec2 | None = None
        self.prev_hand_pos: Vec2 | None = None
        self.hand_velocity = Vec2(0.0, 0.0)
        self.hand_sample_elapsed = 0.0
        self.time_since_hand_seen = 999.0
        self.decision_timer = 0.0

        self.algorithms = create_algorithm_profile(ALGORITHM_PROFILE)

        self.min_state_duration = {
            RobotState.IDLE: 0.0,
            RobotState.NORMAL_ESCAPE: 0.15,
            RobotState.BOUNDARY_ESCAPE: 0.25,
            RobotState.CLOSE_RETREAT: 0.0,
            RobotState.BLIND_SPOT_ESCAPE: 0.30,
        }

        self.state_priority = {
            RobotState.IDLE: 0,
            RobotState.NORMAL_ESCAPE: 10,
            RobotState.BOUNDARY_ESCAPE: 20,
            RobotState.BLIND_SPOT_ESCAPE: 30,
            RobotState.CLOSE_RETREAT: 40,
        }

    def update(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
        dt: float,
        hand_sample_fresh: bool = True,
    ):
        self._update_hand_memory(hand_pos, dt, hand_sample_fresh)

        self.decision_timer -= dt

        if self.decision_timer <= 0.0:
            desired_decision = self._decide_next_state(
                ice_pos=ice_pos,
                hand_pos=hand_pos,
            )

            final_decision = self._apply_transition_guard(desired_decision)
            self.decision_timer += self.config.control_period
            while self.decision_timer <= 0.0:
                self.decision_timer += self.config.control_period

            if final_decision.next_state != self.state:
                self.state = final_decision.next_state
                self.state_time = 0.0

                self.algorithms[self.state].on_enter(
                    ice_pos=ice_pos,
                    hand_pos=hand_pos,
                    last_hand_pos=self.last_hand_pos,
                    config=self.config,
                )
            else:
                self.state_time += dt

            self.transition_reason = final_decision.reason
        else:
            self.state_time += dt
            self.transition_reason = (
                f"hold {self.state.name}; "
                f"next decision in {self.decision_timer:.2f}s"
            )

        algo = self.algorithms[self.state]

        return algo.update(
            ice_pos=ice_pos,
            hand_pos=hand_pos,
            last_hand_pos=self.last_hand_pos,
            config=self.config,
            dt=dt,
        )

    def _update_hand_memory(
        self,
        hand_pos: Vec2 | None,
        dt: float,
        hand_sample_fresh: bool,
    ):
        self.hand_sample_elapsed += dt

        if not hand_sample_fresh:
            self.time_since_hand_seen += dt
            return

        sample_dt = max(self.hand_sample_elapsed, 1e-6)
        self.hand_sample_elapsed = 0.0

        if hand_pos is not None:
            if self.prev_hand_pos is not None:
                raw_velocity = (hand_pos - self.prev_hand_pos) * (1.0 / sample_dt)
                self.hand_velocity = self.hand_velocity * 0.65 + raw_velocity * 0.35

            self.prev_hand_pos = hand_pos
            self.last_hand_pos = hand_pos
            self.time_since_hand_seen = 0.0
        else:
            self.prev_hand_pos = None
            self.hand_velocity = self.hand_velocity * 0.85
            self.time_since_hand_seen += dt

    def _decide_next_state(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
    ) -> StateDecision:
        close_algo = self.algorithms[RobotState.CLOSE_RETREAT]

        if getattr(close_algo, "is_active", False):
            return StateDecision(
                RobotState.CLOSE_RETREAT,
                "keep CLOSE_RETREAT because retreat animation is active",
            )

        if hand_pos is None:
            if self._should_use_blind_spot_escape():
                return StateDecision(
                    RobotState.BLIND_SPOT_ESCAPE,
                    "hand lost near blind zone",
                )

            return StateDecision(
                RobotState.IDLE,
                "no hand observed",
            )

        hand_ice_distance = distance(ice_pos, hand_pos)

        actual_close_distance = (
            self.config.hand_radius
            + self.config.ice_radius
            + self.config.close_distance
        )

        if hand_ice_distance <= actual_close_distance:
            return StateDecision(
                RobotState.CLOSE_RETREAT,
                (
                    f"hand too close: "
                    f"{hand_ice_distance:.1f} <= {actual_close_distance:.1f}"
                ),
            )

        predicted_hand_distance = self._predicted_closest_hand_distance(ice_pos, hand_pos)

        predicted_close_distance = (
            self.config.hand_radius
            + self.config.ice_radius
            + self.config.close_distance * 0.65
        )

        if predicted_hand_distance <= predicted_close_distance:
            return StateDecision(
                RobotState.CLOSE_RETREAT,
                (
                    f"hand predicted too close: "
                    f"{predicted_hand_distance:.1f} <= {predicted_close_distance:.1f}"
                ),
            )

        boundary_distance = distance_from_nearest_wall(
            ice_pos,
            self.config.ice_radius,
            self.config.width,
            self.config.height,
        )

        if self.state == RobotState.BOUNDARY_ESCAPE:
            if boundary_distance < self.config.boundary_exit_margin:
                return StateDecision(
                    RobotState.BOUNDARY_ESCAPE,
                    (
                        f"stay boundary escape: "
                        f"boundary_distance {boundary_distance:.1f} "
                        f"< exit_margin {self.config.boundary_exit_margin:.1f}"
                    ),
                )
        else:
            if boundary_distance < self.config.boundary_enter_margin:
                return StateDecision(
                    RobotState.BOUNDARY_ESCAPE,
                    (
                        f"enter boundary escape: "
                        f"boundary_distance {boundary_distance:.1f} "
                        f"< enter_margin {self.config.boundary_enter_margin:.1f}"
                    ),
                )

        actual_escape_trigger_distance = (
            self.config.hand_radius
            + self.config.ice_radius
            + self.config.escape_trigger_distance
        )

        if hand_ice_distance <= actual_escape_trigger_distance:
            return StateDecision(
                RobotState.NORMAL_ESCAPE,
                (
                    f"hand inside escape range: "
                    f"{hand_ice_distance:.1f} <= "
                    f"{actual_escape_trigger_distance:.1f}"
                ),
            )

        if predicted_hand_distance <= actual_escape_trigger_distance:
            return StateDecision(
                RobotState.NORMAL_ESCAPE,
                (
                    f"hand predicted inside escape range: "
                    f"{predicted_hand_distance:.1f} <= "
                    f"{actual_escape_trigger_distance:.1f}"
                ),
            )

        return StateDecision(
            RobotState.IDLE,
            "hand is far away",
        )

    def _apply_transition_guard(
        self,
        decision: StateDecision,
    ) -> StateDecision:
        if decision.next_state == self.state:
            return decision

        min_duration = self.min_state_duration[self.state]

        current_priority = self.state_priority[self.state]
        next_priority = self.state_priority[decision.next_state]

        higher_priority_interrupt = next_priority > current_priority

        if self.state_time < min_duration and not higher_priority_interrupt:
            return StateDecision(
                self.state,
                (
                    f"state locked: {self.state.name} "
                    f"for {self.state_time:.2f}s "
                    f"< min {min_duration:.2f}s; "
                    f"blocked transition to {decision.next_state.name}"
                ),
            )

        return decision

    def _should_use_blind_spot_escape(self) -> bool:
        if self.last_hand_pos is None:
            return False

        if self.time_since_hand_seen > self.config.hand_lost_blind_timeout:
            return False

        expanded_blind_zone = self.config.blind_zone.expanded(
            self.config.blind_zone_detection_margin
        )

        return expanded_blind_zone.contains(self.last_hand_pos)

    def _predicted_closest_hand_distance(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2,
    ) -> float:
        closest = distance(ice_pos, hand_pos)

        for horizon in (0.10, 0.20, 0.32, 0.45):
            predicted_hand = hand_pos + self.hand_velocity * horizon
            closest = min(closest, distance(ice_pos, predicted_hand))

        return closest
