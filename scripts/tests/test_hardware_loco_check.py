"""Pure-Python safety checks: this script must never construct motion APIs."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location(
    "loco_check", Path(__file__).resolve().parents[1] / "hardware_loco_check.py"
)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def message():
    return SimpleNamespace(
        header=SimpleNamespace(identity=SimpleNamespace(), policy=SimpleNamespace()),
        parameter="",
    )


class ReadOnlyQueryTests(unittest.TestCase):
    def test_only_getters(self):
        self.assertEqual(set(check.READ_QUERIES), {7001, 7002, 7003})
        for api_id in check.READ_QUERIES:
            request = check.read_request(message, api_id, 123)
            self.assertEqual(request.header.identity.api_id, api_id)
            self.assertEqual(request.header.identity.id, 123)
            self.assertEqual(request.parameter, "{}")
            self.assertFalse(request.header.policy.noreply)

    def test_motion_and_mode_commands_cannot_be_constructed(self):
        for api_id in (0, 1008, 7004, 7101, 7102, 7105, 7110, 7111):
            with self.subTest(api_id=api_id), self.assertRaises(ValueError):
                check.read_request(message, api_id, 123)


if __name__ == "__main__":
    unittest.main()
