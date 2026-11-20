"""Real ROS message adapter boundaries; no master, transport or hardware."""
from collections import deque
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sensor_msgs.msg import NavSatFix
import rospy
from gnss_global import GeographicLocalization


class GlobalAdapterTest(unittest.TestCase):
    def node(self):
        node = GeographicLocalization.__new__(GeographicLocalization)
        node.lock = threading.RLock()
        node.options = dict(max_fix_age_s=2.5, max_odom_gap_s=.5, max_gnss_innovation_m=3.,
                            gnss_position_std_m=.5, min_alignment_displacement_m=.75,
                            max_alignment_residual_m=.5)
        node.origin = dict(latitude=0., longitude=0., altitude_ellipsoid_m=0.)
        node.datum, node.lever = 'WGS84', [.07, 0.]
        node.alignment = dict(yaw=0., translation=[0., 0.], heading_std_rad=.1)
        node.registered, node.collecting, node.generation = True, False, 1
        node.reason, node.latest, node.qualified_stamp = '', None, None
        node.pairs, node.pending, node.qualities = [], {}, {}
        node.odoms = deque([dict(stamp=t, x=0., y=0., yaw=0., vx=0., wz=0.) for t in (10., 10.2)])
        node.globals = deque()
        node.observation_count = node.rejected_count = 0
        node.gnss_pub, node.cancel_pub = Mock(), Mock()
        node.save, node.hold = Mock(), Mock()
        return node

    def fix(self, distance):
        fix = NavSatFix()
        fix.header.stamp, fix.header.frame_id = rospy.Time.from_sec(10.1), 'gnss_link'
        fix.status.status = 2
        fix.latitude, fix.longitude, fix.altitude = 0., math.degrees((distance+.07)/6378137.), 0.
        return fix

    def test_global_prediction_must_cover_fix_before_innovation_gate(self):
        node = self.node()
        node.globals.extend(dict(stamp=t, x=0., y=0., yaw=0.) for t in (10., 10.09))
        quality = dict(gga_quality='4', fresh_rtk_usable='true', fix_valid='true')
        self.assertFalse(node.process_fix(self.fix(100.), quality, 10.2))
        node.gnss_pub.publish.assert_not_called()
        node.globals.append(dict(stamp=10.2, x=0., y=0., yaw=0.))
        self.assertTrue(node.process_fix(self.fix(100.), quality, 10.2))
        node.gnss_pub.publish.assert_not_called()
        self.assertEqual(node.rejected_count, 1)

    def test_persistence_failure_does_not_prevent_reset_cancellation_or_hold(self):
        node = self.node()
        node.save.side_effect = OSError('disk unavailable')
        with patch('gnss_global.threading.Thread') as thread:
            node.invalidate('odom_position_reset')
            node.cancel_pub.publish.assert_called_once()
            thread.return_value.start.assert_called_once()
        self.assertFalse(node.registered)
        self.assertIn('persist', node.reason)

    def test_repeated_invalid_samples_do_not_spawn_repeated_hold_requests(self):
        node = self.node()
        with patch('gnss_global.threading.Thread') as thread:
            node.invalidate('odom_position_reset')
            node.invalidate('odom_position_reset')
            node.cancel_pub.publish.assert_called_once()
            thread.return_value.start.assert_called_once()

    def test_finish_cannot_report_registered_when_session_save_fails(self):
        node = self.node()
        node.registered, node.collecting = False, True
        node.pairs = [dict(stamp=t, odom=dict(stamp=t, x=x, y=0., yaw=0., vx=v, wz=0.), enu=[x+.07, 0., 0.])
                      for t, x, v in ((10., 0., 0.), (11., 1., .2), (12., 2., 0.))]
        node.odoms[-1] = dict(stamp=12., x=2., y=0., yaw=0., vx=0., wz=0.)
        node.save.side_effect = OSError('disk unavailable')
        with patch('gnss_global.rospy.Time.now', return_value=rospy.Time.from_sec(12.)), \
                patch('gnss_global.rospy.wait_for_service'), patch('gnss_global.rospy.ServiceProxy'):
            response = node.finish(None)
        self.assertFalse(response.success)
        self.assertFalse(node.registered)

    def test_first_origin_save_failure_is_reported_without_selecting_another_origin(self):
        node = self.node()
        node.origin, node.registered, node.collecting = None, False, True
        node.save.side_effect = OSError('disk unavailable')
        quality = dict(gga_quality='4', fresh_rtk_usable='true', fix_valid='true')
        self.assertTrue(node.process_fix(self.fix(0.), quality, 10.2))
        self.assertIsNotNone(node.origin)
        self.assertFalse(node.collecting)
        self.assertFalse(node.registered)
        self.assertIn('persist_failed', node.reason)

    def test_time_reset_clears_buffers_even_before_registration_without_spurious_hold(self):
        node = self.node()
        node.registered, node.collecting = False, False
        with patch('gnss_global.threading.Thread') as thread:
            node.invalidate('odom_time_reversed')
            self.assertFalse(node.odoms)
            node.cancel_pub.publish.assert_not_called()
            thread.return_value.start.assert_not_called()


if __name__ == '__main__':
    unittest.main()
