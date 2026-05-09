import asyncio
import logging

from core.app.models import RobotState

logger = logging.getLogger(__name__)

CONTROL_INTERVAL_SECONDS = 0.1


class RobotController:
    def __init__(self, app):
        self.app = app

    async def run(self) -> None:
        while True:
            if self.app.state.robot_state is not RobotState.started:
                await asyncio.sleep(CONTROL_INTERVAL_SECONDS)
                continue

            try:
                self.step()
            except Exception:
                logger.exception("Robot controller failed")
                self.app.state.running = False
                self.app.state.robot_state = RobotState.error

            await asyncio.sleep(CONTROL_INTERVAL_SECONDS)

    def step(self) -> None:
        hand = self.app.state.hand
        motor_manager = self.app.state.motor_manager

        motor_state = motor_manager.get_state()

        targets = self.compute_targets(
            hand=hand,
            motor_state=motor_state,
        )

        motor_manager.set_target(targets)

    def compute_targets(self, *, hand, motor_state: dict) -> dict[str, float]:
        motors = motor_state["motors"]

        # 第一版：維持所有 motor 目前 target，不真的移動
        targets = {
            motor_id: float(state["target_angle"]) for motor_id, state in motors.items()
        }

        return targets
