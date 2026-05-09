import asyncio
import inspect
import unittest
from types import SimpleNamespace

import core.app.main as core_main
from core.app import router
from core.app.main import app
from core.app.models import (
    DebugMoveRequest,
    Hand,
    HandDetectionBox,
    HandDetectionResult,
    MotorDirection,
    MotorJogRequest,
    RobotState,
)
from core.app.robot_controller import RobotController
from core.app.vision_state import (
    apply_hand_detection,
    mark_vision_disconnected,
    vision_is_stale,
)
from motor_control import MotorManager
from motor_control.mock_motors import build_mock_motors


class DebugMoveApiTest(unittest.TestCase):
    def test_debug_session_conflict_when_session_started(self) -> None:
        from fastapi import HTTPException

        with _request_context() as request:
            request.app.state.robot_state = RobotState.started
            with self.assertRaises(HTTPException) as ctx:
                asyncio.run(router.start_debug_session(request))
            self.assertEqual(ctx.exception.status_code, 409)

    def test_debug_move_requires_active_debug_session(self) -> None:
        from fastapi import HTTPException

        with _request_context() as request:
            with self.assertRaises(HTTPException) as ctx:
                asyncio.run(
                    router.move_debug_yz(
                        DebugMoveRequest(y_cm=0.0, z_cm=0.0),
                        request,
                    )
                )
            self.assertEqual(ctx.exception.status_code, 409)

    def test_debug_move_rejects_unreachable_target(self) -> None:
        from fastapi import HTTPException

        class UnreachableController:
            def debug_yz_targets(self, **kwargs):
                return None, 9.0

        with _request_context() as request:
            request.app.state.controller = UnreachableController()
            request.app.state.debug_session_active = True
            with self.assertRaises(HTTPException) as ctx:
                asyncio.run(
                    router.move_debug_yz(
                        DebugMoveRequest(y_cm=0.0, z_cm=0.0),
                        request,
                    )
                )
            self.assertEqual(ctx.exception.status_code, 400)
            self.assertIn("not reachable", str(ctx.exception.detail))

    def test_debug_move_replaces_existing_move(self) -> None:
        async def sleeper() -> None:
            await asyncio.sleep(999)

        async def exercise(request) -> None:
            await router.start_debug_session(request)
            first_task = asyncio.create_task(sleeper())
            request.app.state.debug_move_task = first_task

            response = await router.move_debug_yz(
                DebugMoveRequest(y_cm=1.0, z_cm=2.0, duration_s=5.0),
                request,
            )

            self.assertTrue(response.moving)
            self.assertTrue(first_task.done())
            self.assertIsNot(request.app.state.debug_move_task, first_task)
            await router.stop_debug_session(request)

        with _request_context() as request:
            asyncio.run(exercise(request))

    def test_debug_session_stop_cancels_task_and_stops_motors(self) -> None:
        async def sleeper() -> None:
            await asyncio.sleep(999)

        async def exercise(request) -> None:
            request.app.state.debug_session_active = True
            request.app.state.debug_move_task = asyncio.create_task(sleeper())

            response = await router.stop_debug_session(request)

            self.assertFalse(response.active)
            self.assertFalse(response.moving)
            self.assertFalse(request.app.state.debug_session_active)
            self.assertIsNone(request.app.state.debug_move_task)

        with _request_context() as request:
            asyncio.run(exercise(request))

    def test_session_conflict_when_debug_session_active(self) -> None:
        from fastapi import HTTPException

        with _request_context() as request:
            request.app.state.debug_session_active = True
            with self.assertRaises(HTTPException) as ctx:
                asyncio.run(router.create_session(request))
            self.assertEqual(ctx.exception.status_code, 409)


class CoreApiMotorTest(unittest.TestCase):
    def test_core_receives_remote_vision_instead_of_running_hand_tracking(self) -> None:
        main_source = inspect.getsource(core_main)

        self.assertNotIn("hand_tracking", main_source)
        self.assertNotIn("_vision_loop", main_source)
        self.assertNotIn("build_mock_motors", main_source)
        self.assertIn("vision_stale_monitor", main_source)

        route_paths = {route.path for route in app.routes}
        self.assertIn("/ws/hand-ingest", route_paths)
        self.assertNotIn("/ws/vision", route_paths)

    def test_motor_routes_replace_actions_routes(self) -> None:
        schema = app.openapi()

        paths = schema["paths"]
        self.assertIn("/debug/session", paths)
        self.assertIn("/debug/move", paths)
        self.assertNotIn("/debug/circle-demo/start", paths)
        self.assertNotIn("/debug/circle-demo/stop", paths)
        self.assertIn("/motors", paths)
        self.assertNotIn("patch", schema["paths"]["/session"])
        self.assertNotIn("/motors/calibration", paths)
        self.assertIn("/motors/{motor_id}/jog", paths)
        self.assertIn("/motors/{motor_id}/zero", paths)
        self.assertIn("/motors/zero", paths)
        self.assertNotIn("/actions", paths)
        self.assertNotIn("/actions/zero", paths)
        self.assertNotIn("/emergency-stop", paths)
        self.assertNotIn("/vision/frame", paths)

        schemas = schema["components"]["schemas"]
        self.assertIn("MotorsResponse", schemas)
        self.assertIn("MotorStatus", schemas)
        self.assertIn("DebugSessionResponse", schemas)
        self.assertIn("DebugMoveRequest", schemas)
        self.assertIn("DebugMoveResponse", schemas)
        self.assertNotIn("CircleDemoResponse", schemas)
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
        self.assertNotIn("SessionPatchRequest", schemas)
        self.assertNotIn("SessionPatchResponse", schemas)
        self.assertNotIn("SessionCreateRequest", schemas)

    def test_status_response_is_structured_without_session_counters(self) -> None:
        schema = app.openapi()
        status_schema = schema["components"]["schemas"]["StatusResponse"]
        self.assertEqual(
            set(status_schema["properties"]), {"system", "robot", "vision"}
        )

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
            self.assertEqual(payload["vision"]["hand"]["area"], 2400.0)
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
            hand=Hand(),
            vision_ready=True,
            last_hand_result_at=None,
            motor_manager=self.motor_manager,
            motor_manager_ready=False,
            debug_session_active=False,
            debug_move_task=None,
            debug_move_target_y_cm=None,
            debug_move_target_z_cm=None,
            debug_move_duration_s=None,
        )
        app_ns = SimpleNamespace(state=state)
        state.controller = RobotController(app_ns)
        return SimpleNamespace(app=app_ns)

    def __exit__(self, *args):
        self.motor_manager.cleanup()


if __name__ == "__main__":
    unittest.main()
