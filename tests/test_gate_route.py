from pathlib import Path
import unittest

from gate_route import build_legs, build_points, load_route, summary


ROOT = Path(__file__).resolve().parents[1]


class GateRouteTest(unittest.TestCase):
    def test_demo_route_expands_every_gate_and_respects_minimum_duration(self):
        route = load_route(ROOT / "config" / "demo_gate_route.yaml")
        points = build_points(route)
        legs = build_legs(route, points)
        details = summary(route, points, legs)

        self.assertEqual(len(points), 14)  # start + 3 points/gate + finish
        self.assertEqual(sum(point.gate_center for point in points), 4)
        self.assertTrue(all(leg.duration_s >= 1.5 for leg in legs))
        self.assertEqual(details["total_distance_m"], 6.010)
