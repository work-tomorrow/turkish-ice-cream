import asyncio
import logging

from core.app.lab_escape_controller import LabEscapeController
from core.app.models import RobotState

logger = logging.getLogger(__name__)

CONTROL_INTERVAL_SECONDS = 0.1


class RobotController:
    def __init__(self, app):
        self.app = app
        self.escape_controller = LabEscapeController()
        self._was_started = False

    async def run(self) -> None:
        while True:
            if self.app.state.robot_state is not RobotState.started:
                if self._was_started:
                    self.escape_controller.reset()
                    self._was_started = False
                await asyncio.sleep(CONTROL_INTERVAL_SECONDS)
                continue

            try:
                self._was_started = True
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
        result = self.escape_controller.compute_targets(
            hand=hand,
            motor_state=motor_state,
            dt=float(motor_state.get("dt_s", CONTROL_INTERVAL_SECONDS)),
        )
        logger.debug(
            "escape controller: state=%s target=(%.2f, %.2f) reachable=%s err=%s status=%s",
            result.lab_state,
            result.target_y_cm,
            result.target_z_cm,
            result.ik_reachable,
            result.ik_error_cm,
            result.status,
        )
        return result.targets
