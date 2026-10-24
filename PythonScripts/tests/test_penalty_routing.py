import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from Play.Normal.PenaltyRouting import crosses, inside, project, waypoint


class PenaltyRoutingTest(unittest.TestCase):
    front, side = 3380, 1120

    def test_opposite_wings_go_around_both_front_corners(self):
        start, target = (3693, 1398), (3693, -1398)
        self.assertTrue(crosses(start, target, self.front, self.side))
        first = waypoint(start, target, self.front, self.side)
        second = waypoint(first, target, self.front, self.side)
        self.assertEqual(first, (self.front, self.side))
        self.assertEqual(second, (self.front, -self.side))
        self.assertEqual(waypoint(second, target, self.front, self.side), target)

    def test_inside_target_is_projected_and_inside_start_exits(self):
        self.assertEqual(project((3500, 0), self.front, self.side), (self.front, 0))
        self.assertEqual(waypoint((3500, 0), (0, 2000), self.front, self.side), (self.front, 0))

    def test_clear_path_is_unchanged(self):
        self.assertEqual(waypoint((0, 0), (3000, 100), self.front, self.side), (3000, 100))

    def test_paths_reach_projected_target_without_entering_box(self):
        rng = random.Random(34)
        for _ in range(1000):
            start = (rng.uniform(-4500, 4400), rng.uniform(-2900, 2900))
            target = (rng.uniform(-4500, 4400), rng.uniform(-2900, 2900))
            current = project(start, self.front, self.side)
            end = project(target, self.front, self.side)
            for step in range(4):
                nxt = waypoint(current, target, self.front, self.side)
                self.assertFalse(inside(nxt, self.front, self.side))
                self.assertFalse(crosses(current, nxt, self.front, self.side))
                current = nxt
                if current == end:
                    break
            self.assertEqual(current, end)


if __name__ == '__main__':
    unittest.main()
