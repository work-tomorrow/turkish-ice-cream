from __future__ import annotations

import math

from core.app.lab_escape.common import (
    RobotState,
    StateDecision,
    Vec2,
    WorldConfig,
    distance,
    distance_to_square_boundary_along_direction,
)

from core.app.lab_escape.algo_variants import create_algorithm_profile
from core.app.lab_escape.eight_direction_motion import DIRECTIONS_8



ALGORITHM_PROFILE = "turkish_spec"
# 可選：
# ALGORITHM_PROFILE = "smooth"
# ALGORITHM_PROFILE = "performer"
# ALGORITHM_PROFILE = "safe"


class ArmStateMachine:
    def __init__(self, config: WorldConfig):
        self.config = config

        self.state = RobotState.IDLE
        self.state_time = 0.0
        self.transition_reason = "init"

        self.last_hand_pos: Vec2 | None = None
        self.previous_hand_pos: Vec2 | None = None
        self.time_since_hand_seen = 999.0
        self.hand_speed = float("inf")
        self.hand_velocity = Vec2(0.0, 0.0)
        self.last_hand_motion_direction: Vec2 | None = None

        self.algorithms = create_algorithm_profile(ALGORITHM_PROFILE)

        self.min_state_duration = {
            RobotState.IDLE: 0.0,
            RobotState.NORMAL_ESCAPE: 0.0,
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
    ):
        self._update_hand_memory(hand_pos, dt)

        desired_decision = self._decide_next_state(
            ice_pos=ice_pos,
            hand_pos=hand_pos,
        )

        final_decision = self._apply_transition_guard(desired_decision)

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

        algo = self.algorithms[self.state]
        self._configure_flat_escape_direction(algo, ice_pos, hand_pos)

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
    ):
        if hand_pos is not None:
            if self.last_hand_pos is not None and dt > 1e-6:
                hand_delta = hand_pos - self.last_hand_pos
                self.hand_velocity = hand_delta * (1.0 / dt)
                self.hand_speed = self.hand_velocity.length()
                if self.hand_speed > self.config.hand_stationary_speed_cm_s:
                    self.last_hand_motion_direction = hand_delta.normalized()
            else:
                self.hand_velocity = Vec2(0.0, 0.0)
                self.hand_speed = float("inf")
            self.previous_hand_pos = self.last_hand_pos
            self.last_hand_pos = hand_pos
            self.time_since_hand_seen = 0.0
        else:
            self.hand_speed = 0.0
            self.hand_velocity = Vec2(0.0, 0.0)
            self.previous_hand_pos = None
            self.time_since_hand_seen += dt

    def _decide_next_state(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
    ) -> StateDecision:
        if self.config.enable_close_retreat:
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

        can_use_boundary_escape = (
            hand_ice_distance <= self.config.boundary_escape_max_target_distance_cm
        )

        if hand_ice_distance < self.config.contact_escape_distance_cm:
            if (
                can_use_boundary_escape
                and self._flat_escape_near_boundary(ice_pos, hand_pos)
            ):
                return StateDecision(
                    RobotState.BOUNDARY_ESCAPE,
                    "close hand leaves <=6cm along a flat-escape direction",
                )

            return StateDecision(
                RobotState.NORMAL_ESCAPE,
                (
                    f"hand inside contact escape distance: "
                    f"{hand_ice_distance:.1f} < {self.config.contact_escape_distance_cm:.1f}"
                ),
            )

        if self.hand_speed <= self.config.hand_stationary_speed_cm_s:
            if self.last_hand_motion_direction is not None:
                return StateDecision(
                    RobotState.NORMAL_ESCAPE,
                    "hand is stationary; escape opposite previous trajectory",
                )

            return StateDecision(
                RobotState.IDLE,
                f"hand is stationary: {self.hand_speed:.1f}cm/s",
            )

        if not self._hand_is_moving_toward_ice(ice_pos, hand_pos):
            return StateDecision(
                RobotState.IDLE,
                "hand is not approaching ice",
            )

        if (
            can_use_boundary_escape
            and self._flat_escape_near_boundary(ice_pos, hand_pos)
        ):
            return StateDecision(
                RobotState.BOUNDARY_ESCAPE,
                "a flat-escape direction is within 6cm of boundary",
            )

        if self.config.enable_close_retreat:
            actual_close_distance = (
                self.config.hand_radius_cm
                + self.config.ice_radius_cm
                + self.config.close_distance_cm
            )

            if hand_ice_distance <= actual_close_distance:
                return StateDecision(
                    RobotState.CLOSE_RETREAT,
                    (
                        f"hand too close: "
                        f"{hand_ice_distance:.1f} <= {actual_close_distance:.1f}"
                    ),
                )

        return StateDecision(
            RobotState.NORMAL_ESCAPE,
            f"hand is moving: distance={hand_ice_distance:.1f}",
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

        if self.time_since_hand_seen > self.config.hand_lost_blind_timeout_s:
            return False

        expanded_blind_zone = self.config.blind_zone.expanded(
            self.config.blind_zone_detection_margin_cm
        )

        return expanded_blind_zone.contains(self.last_hand_pos)

    def _hand_is_moving_toward_ice(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2,
    ) -> bool:
        if self.previous_hand_pos is None:
            return False

        previous_distance = distance(ice_pos, self.previous_hand_pos)
        current_distance = distance(ice_pos, hand_pos)

        return current_distance < previous_distance

    def _configure_flat_escape_direction(
        self,
        algo,
        ice_pos: Vec2,
        hand_pos: Vec2 | None,
    ):
        if not hasattr(algo, "set_forced_directions"):
            return

        if hand_pos is None:
            algo.set_forced_directions([])
            if hasattr(algo, "set_preferred_directions"):
                algo.set_preferred_directions([])
            return

        forced_directions = self._direct_approach_escape_directions(
            ice_pos,
            hand_pos,
        )
        algo.set_forced_directions(forced_directions)

        if not hasattr(algo, "set_preferred_directions"):
            return

        if forced_directions:
            algo.set_preferred_directions([])
        elif self.hand_speed <= self.config.hand_stationary_speed_cm_s:
            algo.set_preferred_directions(
                self._stationary_escape_directions()
            )
        else:
            algo.set_preferred_directions([])

    def _stationary_escape_directions(self) -> list[Vec2]:
        if self.last_hand_motion_direction is None:
            return []

        return [self.last_hand_motion_direction * -1.0]

    def _direct_approach_escape_directions(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2,
    ) -> list[Vec2]:
        if self.hand_velocity.length() < self.config.hand_stationary_speed_cm_s:
            return []

        target_to_ice = (ice_pos - hand_pos).normalized()
        hand_direction = self.hand_velocity.normalized()

        if hand_direction.dot(target_to_ice) < self.config.direct_approach_alignment:
            return []

        return [
            Vec2(-hand_direction.y, hand_direction.x),
            Vec2(hand_direction.y, -hand_direction.x),
        ]

    def _flat_escape_near_boundary(
        self,
        ice_pos: Vec2,
        hand_pos: Vec2,
    ) -> bool:
        toward_hand = (hand_pos - ice_pos).normalized()
        min_angle = self.config.flat_escape_min_angle_deg
        if distance(ice_pos, hand_pos) < self.config.close_flat_escape_distance_cm:
            min_angle = self.config.close_flat_escape_min_angle_deg

        min_dot = math.cos(math.radians(min_angle))

        for direction in DIRECTIONS_8:
            if direction.dot(toward_hand) >= min_dot:
                continue

            boundary_distance = distance_to_square_boundary_along_direction(
                ice_pos,
                direction,
                self.config.ice_radius_cm,
                self.config.y_min_cm,
                self.config.y_max_cm,
                self.config.z_min_cm,
                self.config.z_max_cm,
            )

            if boundary_distance <= self.config.boundary_escape_distance_cm:
                return True

        return False
