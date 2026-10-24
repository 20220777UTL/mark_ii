"""纯 Python 阵容回归；不加载原生运动控制，不向车辆发送任务。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from Play.Normal.NormalPlayRoster import field_numbers


class NormalPlayRosterTest(unittest.TestCase):
    def test_none_before_vision_ready(self):
        self.assertEqual(field_numbers(None, 0), ())

    def test_nonconsecutive_numbers_and_nonzero_goalie(self):
        self.assertEqual(field_numbers(None, 5, (9, 5, 2)), (2, 9))

    def test_late_arrival_fills_second_role(self):
        first = field_numbers(None, 0, (0, 7))
        self.assertEqual(first, (7,))
        self.assertEqual(field_numbers(None, 0, (0, 3, 7), first), (7, 3))

    def test_dropped_frame_does_not_change_roles(self):
        self.assertEqual(field_numbers(None, 0, (0,), (7, 3)), (7, 3))
        self.assertEqual(field_numbers(None, 0, (), (7, 3), reselect=True), (7, 3))

    def test_more_visible_players_do_not_interrupt_pass(self):
        self.assertEqual(field_numbers(None, 0, (0, 1, 2, 3, 7), (7, 3)), (7, 3))

    def test_reentry_accepts_substitution(self):
        self.assertEqual(field_numbers(None, 0, (0, 3, 9), (7, 3), reselect=True), (3, 9))

    def test_explicit_roster_keeps_configured_order_without_vision(self):
        self.assertEqual(field_numbers((7, 5, 3), 5), (7, 3))

    def test_explicit_bad_config_still_reports_error(self):
        for configured in ((0, 1, 1), (0, 1), (1, 2, 3), (0, -1, 2), (0, 1, 16)):
            with self.subTest(configured=configured), self.assertRaises(ValueError):
                field_numbers(configured, 0)


if __name__ == "__main__":
    unittest.main()
