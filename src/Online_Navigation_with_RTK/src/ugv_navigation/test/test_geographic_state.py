import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'cup_gnss/src'))
try:
    import geographic_state as state
except ModuleNotFoundError as error:
    if error.name != 'geographic_state':
        raise
    state = None


class GeographicStateTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(state, 'geographic observation policy is missing')
        self.fix = dict(stamp=10., stamp_ns=10000000000, latitude=22., longitude=113.,
                        altitude=20., status=2, frame='gnss_link', covariance_type=0,
                        covariance=[0.] * 9)
        self.quality = dict(stamp=10., stamp_ns=10000000000, gga_quality='4',
                            fresh_rtk_usable='true', fix_valid='true', session='test')

    def test_exact_epoch_quality_and_raw_fixed_are_required(self):
        self.assertEqual(state.qualified_fix(self.fix, self.quality, 10.1, 2.5), '')
        for change in (dict(gga_quality='5'), dict(fresh_rtk_usable='false'),
                       dict(fix_valid='false'), dict(stamp_ns=9999999999)):
            self.assertTrue(state.qualified_fix(self.fix, dict(self.quality, **change), 10.1, 2.5))

    def test_stale_future_invalid_and_unknown_height_are_not_observations(self):
        for now in (13., 9.):
            self.assertTrue(state.qualified_fix(self.fix, self.quality, now, 2.5))
        for change in (dict(status=-1), dict(altitude=math.nan), dict(latitude=100.), dict(frame='base_link')):
            self.assertTrue(state.qualified_fix(dict(self.fix, **change), self.quality, 10.1, 2.5))

    def test_covariance_unknown_uses_explicit_assumption_not_zero(self):
        covariance, assumed = state.horizontal_covariance(self.fix, .5)
        self.assertTrue(assumed)
        self.assertEqual(covariance, [.25, 0., 0., .25])
        fixed = dict(self.fix, covariance_type=3, covariance=[.04, .01, 0., .01, .09, 0., 0., 0., .16])
        self.assertEqual(state.horizontal_covariance(fixed, .5), ([.04, .01, .01, .09], False))

    def test_normal_motion_is_continuous_but_reset_invalidates(self):
        previous = dict(stamp=10., x=3., y=2., yaw=3.13, frame='odom', child='base_link')
        current = dict(previous, stamp=10.1, x=3.02, yaw=-3.13)
        self.assertFalse(state.odom_reset(previous, current))
        for change in (dict(stamp=9.), dict(x=0., y=0.), dict(frame='other'), dict(stamp=10., x=0., y=0.)):
            self.assertTrue(state.odom_reset(previous, dict(current, **change)))

    def test_existing_antenna_horizontal_configuration_is_used_once(self):
        site = dict(antenna_in_base_link_m=[None, None, None])
        source = {'sensors': {'base_to_laser': {'xyz': [.07, .01, .18]}}}
        self.assertEqual(state.antenna_xy(site, source), [.07, .01])
        self.assertEqual(state.antenna_xy(dict(antenna_in_base_link_m=[.2, 0., None]), source), [.2, 0.])


if __name__ == '__main__':
    unittest.main()
