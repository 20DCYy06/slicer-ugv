import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from preparation_path import candidate_goals, observed_path


def make_grid(cells):
    return NS(info=NS(width=5, height=5, resolution=1., origin=NS(position=NS(x=0., y=0.))),
              data=cells)


class PreparationPathTest(unittest.TestCase):
    def test_checks_between_sparse_planner_poses(self):
        cells = [0] * 25
        self.assertAlmostEqual(observed_path(make_grid(cells), [(0.5, 1.5), (4.5, 1.5)], 5.), 4.)
        for value in (-1, 99, 100):
            cells[7] = value
            self.assertIsNone(observed_path(make_grid(cells), [(0.5, 1.5), (4.5, 1.5)], 5.))

    def test_curved_path_checks_bounds_and_actual_distance(self):
        grid = make_grid([0] * 25)
        points = [(0.5, 0.5), (1.5, 1.5), (3.5, 1.5)]
        self.assertAlmostEqual(observed_path(grid, points, 4.), math.sqrt(2.) + 2.)
        self.assertIsNone(observed_path(grid, points, 3.))
        self.assertIsNone(observed_path(grid, [(0.5, 0.5), (-0.1, 0.5)], 4.))
        self.assertIsNone(observed_path(grid, [(0.5, 0.5)], 4.))

    def test_forward_then_alternatives_and_shorter_goals(self):
        goals = list(candidate_goals((2., 3., math.pi / 2), (2.5, 1.2)))
        self.assertAlmostEqual(goals[0][0], 2.)
        self.assertAlmostEqual(goals[0][1], 5.5)
        self.assertEqual(len(goals), 16)
        self.assertTrue(any(y < 3. for _, y, _ in goals))


    def test_roundtrip_uses_a_shorter_known_straight_outbound_and_return(self):
        from preparation_path import straight_roundtrip_goal

        cells = [0] * 25
        grid = make_grid(cells)
        for bad_cell in (-1, 90, 100):
            with self.subTest(bad_cell=bad_cell):
                cells[8] = bad_cell
                visits = []
                start = (1.15, 1.5, 0.)

                def plan(goal):
                    visits.append(goal)
                    return [start[:2], goal[:2]]

                def known(points):
                    return observed_path(grid, points, 3., max_cost=66) is not None

                goal = straight_roundtrip_goal(start, 2., 1.25, plan, known)
                self.assertEqual(visits, [goal])
                self.assertAlmostEqual(goal[0], 2.9)
                self.assertEqual(goal[1:], (1.5, 0.))

    def test_roundtrip_rejects_detours_and_reports_no_adequate_segment(self):
        from preparation_path import straight_roundtrip_goal

        start = (1.15, 1.5, 0.)
        for waypoint in ((1.5, 2.3), (0.9, 1.5), (1.2, 1.5)):
            with self.subTest(waypoint=waypoint):
                def plan(goal):
                    return [start[:2], (1.9, 1.5), waypoint, goal[:2]]

                self.assertIsNone(straight_roundtrip_goal(start, 2., 1.25, plan, lambda _: True))
        self.assertIsNone(straight_roundtrip_goal(start, 1., 1.2, lambda _: self.fail("too short"), lambda _: True))
        self.assertIsNone(straight_roundtrip_goal((4.1, 1.5, 0.), 2., 1.25, lambda _: self.fail("outside graph"),
                                                 lambda pts: observed_path(make_grid([0] * 25), pts, 3.) is not None))
