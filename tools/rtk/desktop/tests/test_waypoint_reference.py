"""The desktop can estimate WGS84 waypoints without running navigation."""
from pathlib import Path
import sys
import unittest

TESTS = Path(__file__).resolve()
sys.path.insert(0, str(TESTS.parents[1]))
sys.path.insert(0, str(TESTS.parents[4]/'src'/'ugv_navigation'/'scripts'))
from geographic_core import enu_to_geodetic
from waypoint_reference import WaypointReferences


ORIGIN = dict(latitude=31., longitude=121., altitude_ellipsoid_m=42.)


def reference(stamp, x, y, yaw, east, north):
    lat, lon, height = enu_to_geodetic((east, north, 0.), ORIGIN)
    return (dict(stamp=stamp, x=x, y=y, yaw=yaw),
            dict(latitude=lat, longitude=lon, altitude_ellipsoid_m=height))


class WaypointTests(unittest.TestCase):
    def test_two_synchronized_stations_estimate_a_clicked_point(self):
        refs = WaypointReferences()
        refs.capture(*reference(10., 0., 0., 0., 0., .3), (.3, 0.))
        self.assertIsNone(refs.convert(2., 0.))
        refs.capture(*reference(20., 3., 0., 0., 0., 3.3), (.3, 0.))
        expected = enu_to_geodetic((-2., 4., 0.), ORIGIN)
        point = refs.convert(4., 2.)
        self.assertAlmostEqual(point['latitude'], expected[0], places=8)
        self.assertAlmostEqual(point['longitude'], expected[1], places=8)

    def test_rejects_turn_in_place_and_disagreeing_baselines(self):
        refs = WaypointReferences()
        refs.capture(*reference(10., 0., 0., 0., 0., .3), (.3, 0.))
        with self.assertRaises(ValueError):
            refs.capture(*reference(20., 0., 0., 0., 0., 3.3), (.3, 0.))
        with self.assertRaises(ValueError):
            refs.capture(*reference(21., 3., 0., 0., 0., 2.5), (.3, 0.))
        self.assertIsNone(refs.convert(2., 0.))

    def test_reset_drops_the_transform(self):
        refs = WaypointReferences()
        refs.capture(*reference(10., 0., 0., 0., 0., 0.), (0., 0.))
        refs.capture(*reference(20., 3., 0., 0., 0., 3.), (0., 0.))
        refs.reset()
        self.assertIsNone(refs.convert(2., 0.))
