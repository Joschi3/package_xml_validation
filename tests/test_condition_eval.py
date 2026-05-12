import logging
import unittest

from package_xml_validation.helpers import condition_eval


class TestConditionEval(unittest.TestCase):
    def setUp(self):
        condition_eval._WARNED_EXPRESSIONS.clear()

    def test_none_is_true(self):
        self.assertTrue(condition_eval.evaluate_condition(None))

    def test_empty_string_is_true(self):
        self.assertTrue(condition_eval.evaluate_condition(""))
        self.assertTrue(condition_eval.evaluate_condition("   "))

    def test_env_var_match(self):
        self.assertTrue(
            condition_eval.evaluate_condition(
                "$ROS_DISTRO == 'jazzy'", {"ROS_DISTRO": "jazzy"}
            )
        )
        self.assertFalse(
            condition_eval.evaluate_condition(
                "$ROS_DISTRO == 'jazzy'", {"ROS_DISTRO": "humble"}
            )
        )

    def test_boolean_compound(self):
        ctx = {"ROS_DISTRO": "jazzy", "ROS_VERSION": "2"}
        self.assertTrue(
            condition_eval.evaluate_condition(
                "$ROS_DISTRO == 'jazzy' and $ROS_VERSION == '2'", ctx
            )
        )
        self.assertFalse(
            condition_eval.evaluate_condition(
                "$ROS_DISTRO == 'humble' and $ROS_VERSION == '2'", ctx
            )
        )

    def test_malformed_returns_true_with_warning(self):
        logger = logging.getLogger("test_malformed")
        with self.assertLogs(logger, level="WARNING") as captured:
            result = condition_eval.evaluate_condition("garbage((", {}, logger=logger)
        self.assertTrue(result)
        self.assertTrue(any("Could not parse" in m for m in captured.output))

    def test_malformed_warns_only_once(self):
        logger = logging.getLogger("test_dedup")
        with self.assertLogs(logger, level="WARNING") as captured:
            condition_eval.evaluate_condition("garbage((", {}, logger=logger)
            condition_eval.evaluate_condition("garbage((", {}, logger=logger)
            condition_eval.evaluate_condition("garbage((", {}, logger=logger)
        warnings = [m for m in captured.output if "Could not parse" in m]
        self.assertEqual(len(warnings), 1)


if __name__ == "__main__":
    unittest.main()
