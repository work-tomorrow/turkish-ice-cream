import asyncio
import inspect
import unittest
from types import SimpleNamespace

import core.app.main as core_main
from core.app import router
from core.app.main import app
from core.app.models import (
    Hand,
    HandDetectionBox,
    HandDetectionResult,
    MotorDirection,
    MotorJogRequest,
    RobotState,
)
from core.app.vision_state import (
    apply_hand_detection,
    mark_vision_disconnected,
    vision_is_stale,
)
from motor_control import MotorManager
from motor_control.mock_motors import build_mock_motors


class CoreApiMotorTest(unittest.TestCase):
    def test_core_receives_remote_vision_instead_of_running_hand_tracking(self) -> None:
        main_source = inspect.getsource(core_main)

        self.assertNotIn("hand_tracking", main_source)
        self.assertNotIn("_vision_loop", main_source)
        self.assertIn("vision_stale_monitor", main_source)

        route_paths = {route.path for route in app.routes}
        self.assertIn("/ws/hand-ingest", route_paths)
        self.assertNotIn("/ws/vision", route_paths)

    def test_motor_routes_replace_actions_routes(self) -> None:
        schema = app.openapi()

        paths = schema["paths"]
        self.assertIn("/motors", paths)
        self.assertNotIn("/motors/calibration", paths)
        self.assertIn("/motors/{motor_id}/jog", paths)
        self.assertIn("/motors/{motor_id}/zero", paths)
        self.assertIn("/motors/zero", paths)
        self.assertNotIn("/actions", paths)
        self.assertNotIn("/actions/zero", paths)
        self.assertNotIn("/emergency-stop", paths)

        schemas = schema["components"]["schemas"]
        self.assertIn("MotorsResponse", schemas)
        self.assertIn("MotorStatus", schemas)
        self.assertIn("MotorJogRequest", schemas)
        self.assertIn("MotorJogResponse", schemas)
        self.assertIn("MotorJogResult", schemas)
        self.assertIn("MotorZeroResponse", schemas)
        self.assertIn("RobotState", schemas)
        self.assertNotIn("MotorCalibrationResponse", schemas)
        self.assertNotIn("MotorCalibrationStatus", schemas)
        self.assertNotIn("ActionsResponse", schemas)
        self.assertNotIn("ActionRequest", schemas)
        self.assertNotIn("ActionCommandResponse", schemas)
        self.assertNotIn("ActionResult", schemas)
        self.assertNotIn("ZeroAngleResponse", schemas)
        self.assertNotIn("ActionName", schemas)
        self.assertNotIn("EmergencyStopSetResponse", schemas)
        self.assertNotIn("EmergencyStopClearResponse", schemas)

    def test_status_response_is_structured_without_session_counters(self) -> None:
        schema = app.openapi()
        status_schema = schema["components"]["schemas"]["StatusResponse"]
        self.assertEqual(set(status_schema["properties"]), {"system", "robot", "vision"})

        with _request_context() as request:
            response = asyncio.run(router.get_status(request))
            payload = response.model_dump()

            self.assertEqual(
                set(payload),
                {"system", "robot", "vision"},
            )
            self.assertEqual(
                payload["system"],
                {"ready": True},
            )
            self.assertEqual(payload["robot"], {"state": "idle"})
            self.assertEqual(
                set(payload["vision"]),
                {"hand_detected", "hand"},
            )
            self.assertNotIn("tease_count", payload)
            self.assertNotIn("max_tease", payload)

    def test_hand_ingest_updates_status_from_rpi_point_of_view(self) -> None:
        with _request_context() as request:
            apply_hand_detection(
                request.app.state,
                HandDetectionResult(
                    detected=True,
                    box=HandDetectionBox(x1=10, y1=20, x2=50, y2=80),
                    confidence=0.75,
                ),
                now=100.0,
            )

            response = asyncio.run(router.get_status(request))
            payload = response.model_dump()

            self.assertTrue(payload["system"]["ready"])
            self.assertTrue(payload["vision"]["hand_detected"])
            self.assertEqual(payload["vision"]["hand"]["x"], 30.0)
            self.assertEqual(payload["vision"]["hand"]["y"], 50.0)
            self.assertEqual(payload["vision"]["hand"]["confidence"], 0.75)

    def test_remote_vision_disconnect_and_timeout_clear_hand(self) -> None:
        with _request_context() as request:
            apply_hand_detection(
                request.app.state,
                HandDetectionResult(
                    detected=True,
                    box=HandDetectionBox(x1=0, y1=0, x2=20, y2=20),
                    confidence=0.9,
                ),
                now=10.0,
            )

            self.assertFalse(vision_is_stale(request.app.state, 1.0, now=10.5))
            self.assertTrue(vision_is_stale(request.app.state, 1.0, now=11.5))

            mark_vision_disconnected(request.app.state)
            response = asyncio.run(router.get_status(request))
            payload = response.model_dump()

            self.assertFalse(payload["system"]["ready"])
            self.assertFalse(payload["vision"]["hand_detected"])
            self.assertFalse(payload["vision"]["hand"]["detected"])

    def test_motors_lists_motor_state(self) -> None:
        with _request_context() as request:
            response = asyncio.run(router.get_motors(request))
            payload = response.model_dump()
            self.assertEqual(
                {motor["id"] for motor in payload["motors"]},
                {"m1", "m2", "m3", "m4"},
            )
            for motor in payload["motors"]:
                self.assertIn("current_angle", motor)
                self.assertIn("min_angle", motor)
                self.assertIn("max_angle", motor)
                self.assertIn("step_degrees", motor)

    def test_jog_motor_uses_path_motor_id(self) -> None:
        with _request_context() as request:
            response = asyncio.run(
                router.jog_motor(
                    "m1",
                    MotorJogRequest(
                        direction=MotorDirection.positive,
                        step_degrees=3,
                    ),
                    request,
                )
            )
            payload = response.model_dump()
            self.assertTrue(payload["ok"])
            self.assertEqual(payload["action"]["motor_id"], "m1")
            self.assertEqual(payload["action"]["delta_degrees"], 3.0)
            self.assertEqual(payload["robot_state"], "idle")

    def test_zero_motor_routes_support_one_or_all_motors(self) -> None:
        with _request_context() as request:
            one_response = asyncio.run(router.zero_motor("m3", request))
            all_response = asyncio.run(router.zero_all_motors(request))

            self.assertEqual(one_response.model_dump()["zeroed_motor_ids"], ["m3"])
            self.assertEqual(
                set(all_response.model_dump()["zeroed_motor_ids"]),
                {"m1", "m2", "m3", "m4"},
            )

    def test_zero_motor_route_redefines_absolute_zero_for_future_jogs(self) -> None:
        with _request_context() as request:
            asyncio.run(
                router.jog_motor(
                    "m3",
                    MotorJogRequest(
                        direction=MotorDirection.positive,
                        step_degrees=3,
                    ),
                    request,
                )
            )
            asyncio.run(router.zero_motor("m3", request))
            response = asyncio.run(
                router.jog_motor(
                    "m3",
                    MotorJogRequest(
                        direction=MotorDirection.positive,
                        step_degrees=3,
                    ),
                    request,
                )
            )

            payload = response.model_dump()
            self.assertEqual(payload["action"]["current_angle"], 3.0)

    def test_session_stop_stops_motors_and_interaction(self) -> None:
        with _request_context() as request:
            request.app.state.running = True
            request.app.state.robot_state = RobotState.started
            asyncio.run(
                router.jog_motor(
                    "m3",
                    MotorJogRequest(
                        direction=MotorDirection.positive,
                        step_degrees=3,
                    ),
                    request,
                )
            )

            stop_response = asyncio.run(router.delete_session(request))
            response = asyncio.run(
                router.jog_motor(
                    "m3",
                    MotorJogRequest(
                        direction=MotorDirection.positive,
                        step_degrees=3,
                    ),
                    request,
                )
            )

            self.assertFalse(stop_response.session.active)
            self.assertFalse(request.app.state.running)
            payload = response.model_dump()
            self.assertEqual(payload["action"]["current_angle"], 96.0)


class _request_context:
    def __enter__(self):
        self.motor_manager = MotorManager(motors=build_mock_motors())
        state = SimpleNamespace(
            running=False,
            robot_state=RobotState.idle,
            tease_count=0,
            max_tease=10,
            hand=Hand(),
            vision_ready=True,
            last_hand_result_at=None,
            motor_manager=self.motor_manager,
            motor_manager_ready=False,
        )
        return SimpleNamespace(app=SimpleNamespace(state=state))

    def __exit__(self, *args):
        self.motor_manager.cleanup()


if __name__ == "__main__":
    unittest.main()
