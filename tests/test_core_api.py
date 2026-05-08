import asyncio
import unittest
from types import SimpleNamespace

from core.app import router
from core.app.main import app
from core.app.models import Hand, MotorDirection, MotorJogRequest, RobotState
from motor_control import MotorManager


class CoreApiMotorTest(unittest.TestCase):
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
                {"ready": True, "emergency_stopped": False},
            )
            self.assertEqual(payload["robot"], {"state": "idle"})
            self.assertEqual(
                set(payload["vision"]),
                {"hand_detected", "hand"},
            )
            self.assertNotIn("tease_count", payload)
            self.assertNotIn("max_tease", payload)

    def test_motors_lists_motor_state(self) -> None:
        with _request_context() as request:
            response = asyncio.run(router.get_motors(request))
            payload = response.model_dump()
            self.assertNotIn("step_degrees", payload)
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


class _request_context:
    def __enter__(self):
        self.motor_manager = MotorManager()
        state = SimpleNamespace(
            running=False,
            emergency_stopped=False,
            robot_state=RobotState.idle,
            tease_count=0,
            max_tease=10,
            hand=Hand(),
            motor_manager=self.motor_manager,
            motor_manager_ready=False,
        )
        return SimpleNamespace(app=SimpleNamespace(state=state))

    def __exit__(self, *args):
        self.motor_manager.cleanup()


if __name__ == "__main__":
    unittest.main()
