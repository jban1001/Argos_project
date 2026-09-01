import importlib.util
import json
import math
from pathlib import Path
import unittest


PATH = Path(__file__).with_name("argos_mode.py")
SPEC = importlib.util.spec_from_file_location("argos_mode", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ArgosModeTest(unittest.TestCase):
    def test_mode_payload_is_strict_and_compact(self):
        value = json.loads(MODULE.mode_payload("follow", "req-1"))
        self.assertEqual(
            value, {"schema": 1, "request_id": "req-1", "mode": "follow"})
        with self.assertRaises(ValueError):
            MODULE.mode_payload("unsafe", "req-2")

    def test_dispatch_uses_map_radians_and_explicit_clearance(self):
        value = json.loads(MODULE.dispatch_payload(
            "fire-1", 1.2, -0.4, 90.0, False))
        self.assertEqual(value["frame_id"], "map")
        self.assertAlmostEqual(value["yaw"], math.pi / 2)
        self.assertIs(value["main_cleared"], False)

    def test_coordinate_fire_requires_explicit_clearance_flag(self):
        args = MODULE.parser().parse_args(
            ["coordinate-fire", "--x", "1", "--y", "2"])
        self.assertFalse(args.confirm_main_clear)

    def test_dispatch_ack_requires_exact_target_and_nonterminal_state(self):
        expected = json.loads(MODULE.dispatch_payload(
            "fire-1", 1.2, -0.4, 90.0, False))
        status = {
            "mission_id": "fire-1",
            "state": "WAIT_CLEARANCE",
            "target": dict(expected),
        }
        status["target"].pop("schema")
        status["target"].pop("mission_id")
        self.assertTrue(MODULE.dispatch_was_accepted(status, expected))
        status["target"]["x"] = 9.0
        self.assertFalse(MODULE.dispatch_was_accepted(status, expected))
        status["target"]["x"] = 1.2
        status["state"] = "COMPLETE"
        self.assertFalse(MODULE.dispatch_was_accepted(status, expected))


if __name__ == "__main__":
    unittest.main()
