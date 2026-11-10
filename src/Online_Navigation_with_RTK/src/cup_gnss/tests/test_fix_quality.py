"""Per-fix quality correlation with real Noetic messages, no master or devices."""
import importlib.util
import io
import math
from pathlib import Path
import sys
import unittest
from unittest import mock

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from cup_gnss.core import GnssModel
from cup_gnss.quality import apply_rtk_policy
from cup_gnss.sources import LiveSource

try:
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from cup_gnss.upstream_nmea import NmeaFixAdapter
    import libnmea_navsat_driver.driver
    ROS_IMPORT_ERROR = None
except ImportError as error:
    ROS_IMPORT_ERROR = str(error)


def gga(epoch, quality):
    body = ('GNGGA,{},3112.0000,N,12130.0000,E,{},18,0.8,18.2,M,-4.5,M,1.0,0001'
            .format(epoch, quality))
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '${}*{:02X}'.format(body, checksum)


def event(index, epoch, quality):
    return dict(v=1, kind='nmea', session='TEST_ONLY-current-session', seq=index,
                captured_at=2000., _received_wall=2000., _received_mono=1000.,
                nmea=gga(epoch, quality))


def combined(model):
    return apply_rtk_policy(model.snapshot(1000.), dict(enabled=True, fatal=False),
                            dict(rtcm_serial_write_age_s=.1,
                                 rtcm_serial_write_session='TEST_ONLY-current-session'))


def snapshot(quality=4, epoch='120002.00'):
    model = GnssModel(source='serial')
    model.ingest(dict(kind='transport', connected=True), 2000., 1000.)
    model.ingest(event(1, '120001.00', quality), 2000., 1000.)
    model.ingest(event(2, epoch, quality), 2000., 1000.)
    return combined(model)


def values(message):
    return {entry.key: entry.value for entry in message.status[0].values}


@unittest.skipIf(ROS_IMPORT_ERROR is not None, 'Noetic dependency unavailable: ' + str(ROS_IMPORT_ERROR))
class FixQualityCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('gnss_node_quality_test',
                                                       PACKAGE / 'scripts' / 'gnss_node.py')
        cls.node = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.node)

    def quality_message(self, state, stamp):
        helper = getattr(self.node, 'quality_message', None)
        self.assertTrue(callable(helper), 'A per-fix diagnostic message helper is required')
        return helper(state, stamp, 'serial')

    def test_quality_metadata_distinguishes_q4_and_q5_at_their_exact_fix_stamps(self):
        adapter = NmeaFixAdapter()
        for quality, fresh in ((4, 'true'), (5, 'false')):
            state = snapshot(quality)
            stamp = rospy.Time(123, quality)
            fix = adapter.to_fix(gga('120002.00', quality), state, stamp, 'gnss_link')
            message = self.quality_message(state, fix.header.stamp)
            self.assertIs(message.header.stamp, fix.header.stamp)
            self.assertEqual(fix.status.status, 2)  # Both are standard GBAS_FIX.
            metadata = values(message)
            self.assertEqual(metadata['gga_quality'], str(quality))
            self.assertEqual(metadata['fresh_rtk_usable'], fresh)
            self.assertEqual(metadata['fix_valid'], 'true')
            self.assertEqual(metadata['session'], 'TEST_ONLY-current-session')
            self.assertEqual(metadata['gga_utc_time'], '120002.00')
            self.assertEqual(message.status[0].name, 'cup_gnss/receiver')
            self.assertEqual(message.status[0].hardware_id, 'serial')
            stream = io.BytesIO()
            message.serialize(stream)
            restored = DiagnosticArray().deserialize(stream.getvalue())
            self.assertEqual(restored.header.stamp, fix.header.stamp)
            self.assertEqual(values(restored), metadata)

    def test_quality_metadata_keeps_invalid_utc_and_no_fix_policy(self):
        for state, reason in ((snapshot(4, '999999.00'), 'invalid_gga_time'),
                              (snapshot(0), 'receiver_has_no_measured_fix')):
            message = self.quality_message(state, rospy.Time(10, 42))
            metadata = values(message)
            self.assertEqual(metadata['fix_valid'], 'false')
            self.assertEqual(metadata['fresh_rtk_usable'], 'false')
            self.assertEqual(metadata['invalid_reason'], reason)
            self.assertEqual(message.status[0].message, state['message'])
            self.assertEqual(message.status[0].level, state['level'])

    def run_node(self, qualities=(4, 4, 5), adaptation_failure=False,
                 receipt_clock=(2000., 1000., 100.), processing_clock=None,
                 simulated=False, idle_clock=None):
        """Replace only I/O/lifecycle boundaries; use real model, policy and adapter."""
        events = [dict(kind='transport', connected=True)]
        events.extend(event(i, '12000{}.00'.format(i), quality)
                      for i, quality in enumerate(qualities, 1))
        publishers = {}
        clock = list(receipt_clock)  # wall, monotonic, ROS; independent clock domains.
        processing_clock = receipt_clock if processing_clock is None else processing_clock

        class Output:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        def publisher(topic, message_type, **kwargs):
            publishers[topic] = Output()
            return publishers[topic]

        class Source:
            def __init__(self, queue, **kwargs):
                self.queue = queue

            def start(self):
                # Use the real receipt boundary; all measurements are queued
                # before the processing thread resumes at processing_clock.
                receiver = LiveSource(self.queue)
                for item in events:
                    if item['kind'] == 'nmea':
                        item['captured_at'] = clock[0]
                    receiver.emit(item)
                clock[:] = processing_clock

            def rtcm_write_snapshot(self):
                return dict(rtcm_serial_write_age_s=.1,
                            rtcm_serial_write_session='TEST_ONLY-current-session')

            def get_gga(self):
                raise AssertionError('No transport runs in this fixture')

            def write_rtcm(self, data):
                raise AssertionError('No devices run in this fixture')

            def stop(self):
                pass

            def join(self, timeout):
                pass

            def is_alive(self):
                return False

        class Corrections:
            def __init__(self, **kwargs):
                pass

            def snapshot(self):
                return dict(enabled=True, fatal=False)

            def start(self):
                pass

            def stop(self):
                pass

            def join(self, timeout):
                pass

        adapter = NmeaFixAdapter()
        original_to_fix = adapter.to_fix
        rejected = []

        def to_fix(raw, state, stamp, frame):
            if adaptation_failure and state['fix']['valid'] and not rejected:
                rejected.append(True)
                raise ValueError('TEST_ONLY adaptation mismatch')
            return original_to_fix(raw, state, stamp, frame)

        checks = iter(range(len(events) + (2 if idle_clock is not None else 1)))

        def shutdown():
            index = next(checks)
            if index == len(events) and idle_clock is not None:
                clock[:] = idle_clock
                return False
            return index >= len(events)

        params = {'~source': 'serial', '~ntrip_enabled': True}
        with mock.patch.object(self.node.rospy, 'init_node'), \
                mock.patch.object(self.node.rospy, 'get_param', side_effect=lambda key, default: params.get(key, default)), \
                mock.patch.object(self.node.rospy, 'Publisher', side_effect=publisher), \
                mock.patch.object(self.node.rospy, 'is_shutdown', side_effect=shutdown), \
                mock.patch.object(self.node.rospy, 'loginfo'), \
                mock.patch.object(self.node.rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(clock[2])), \
                mock.patch.object(self.node.rospy.rostime, 'is_wallclock', return_value=not simulated), \
                mock.patch.object(self.node.time, 'monotonic', side_effect=lambda: clock[1]), \
                mock.patch.object(self.node.time, 'time', side_effect=lambda: clock[0]), \
                mock.patch.object(self.node, 'SerialSource', Source), \
                mock.patch.object(self.node, 'NtripClient', Corrections), \
                mock.patch.object(self.node, 'NmeaFixAdapter', return_value=adapter), \
                mock.patch.object(adapter, 'to_fix', side_effect=to_fix):
            self.node.main()
        return publishers

    def test_queued_fix_keeps_host_receipt_epoch_for_actual_odom_pairing(self):
        # Review F2: 1.9 s queue delay remains within the existing 2 s source
        # budget. The moving odom must pair at reception x=.5, never x=.975.
        published = self.run_node(qualities=(4, 4),
                                  receipt_clock=(2002., 1000., 2002.),
                                  processing_clock=(2003.9, 1001.9, 2003.9))
        fix = published['/gnss/fix'].messages[-1]
        quality = published['/gnss/fix_quality'].messages[-1]
        self.assertEqual(fix.status.status, 2)
        self.assertEqual(fix.header.stamp, rospy.Time(2002))
        self.assertIs(quality.header.stamp, fix.header.stamp)

        scripts = PACKAGE.parent / 'ugv_navigation' / 'scripts'
        with mock.patch.object(sys, 'path', [str(scripts)] + sys.path):
            from geographic_core import interpolate_odom
            from geographic_state import qualified_fix
        record = dict(stamp=fix.header.stamp.to_sec(), stamp_ns=fix.header.stamp.to_nsec(),
                      frame=fix.header.frame_id, status=fix.status.status,
                      latitude=fix.latitude, longitude=fix.longitude, altitude=fix.altitude)
        metadata = dict(values(quality), stamp_ns=quality.header.stamp.to_nsec())
        self.assertEqual(qualified_fix(record, metadata, 2004., 2.5), '')
        odoms = [dict(stamp=stamp, x=x, y=0., yaw=0.)
                 for stamp, x in ((2001.9, .475), (2002.1, .525),
                                  (2003.8, .95), (2004., 1.))]
        paired = interpolate_odom(odoms, record['stamp'])
        self.assertEqual(paired['stamp'], 2002.)
        self.assertAlmostEqual(paired['x'], .5)

    def test_simulated_clock_retains_receipt_epoch_when_paused_accelerated_or_reset(self):
        # No subtraction of host elapsed time from /clock: its rate and epoch
        # need not agree with wall/monotonic time, and may jump while queued.
        for process_ros in (72., 91., 12.):
            with self.subTest(process_ros=process_ros):
                published = self.run_node(qualities=(4, 4), simulated=True,
                                          receipt_clock=(2002., 1000., 72.),
                                          processing_clock=(2003.9, 1001.9, process_ros))
                fix = published['/gnss/fix'].messages[-1]
                quality = published['/gnss/fix_quality'].messages[-1]
                self.assertEqual(fix.status.status, 2)
                self.assertEqual(fix.header.stamp, rospy.Time(72))
                self.assertIs(quality.header.stamp, fix.header.stamp)

    def test_invalid_heartbeat_never_refreshes_cached_valid_position(self):
        published = self.run_node(qualities=(4, 4),
                                  receipt_clock=(2002., 1000., 2002.),
                                  processing_clock=(2003.9, 1001.9, 2003.9),
                                  idle_clock=(2005., 1003., 2005.))
        fixes = published['/gnss/fix'].messages
        valid = [fix for fix in fixes if fix.status.status >= 0]
        self.assertEqual(len(valid), 1)
        self.assertEqual(valid[0].header.stamp, rospy.Time(2002))
        heartbeat = fixes[-1]
        quality = published['/gnss/fix_quality'].messages[-1]
        self.assertEqual(heartbeat.status.status, -1)
        self.assertTrue(all(math.isnan(getattr(heartbeat, key))
                            for key in ('latitude', 'longitude', 'altitude')))
        self.assertIs(quality.header.stamp, heartbeat.header.stamp)
        self.assertEqual(values(quality)['fix_valid'], 'false')
        self.assertEqual(values(quality)['fresh_rtk_usable'], 'false')

    def test_runtime_publishes_one_exact_stamp_quality_message_for_every_fix(self):
        published = self.run_node()
        self.assertIn('/gnss/fix_quality', published, 'Every fix needs associated raw quality metadata')
        fixes = published['/gnss/fix'].messages
        quality = published['/gnss/fix_quality'].messages
        self.assertEqual(len(quality), len(fixes))
        for fix, metadata in zip(fixes, quality):
            self.assertIs(metadata.header.stamp, fix.header.stamp)
            self.assertEqual(values(metadata)['fix_valid'], 'true' if fix.status.status >= 0 else 'false')
        self.assertEqual([values(item)['gga_quality'] for item in quality[-2:]], ['4', '5'])
        self.assertEqual([values(item)['fresh_rtk_usable'] for item in quality[-2:]], ['true', 'false'])

    def test_runtime_preserves_one_hz_diagnostics_and_raw_nmea_publications(self):
        published = self.run_node()
        self.assertEqual(len(published['/gnss/diagnostics'].messages), 1)
        self.assertEqual(published['/gnss/diagnostics'].messages[0].status[0].name, 'cup_gnss/receiver')
        self.assertEqual([item.data for item in published['/gnss/nmea'].messages],
                         [gga('120001.00', 4), gga('120002.00', 4), gga('120003.00', 5)])
        self.assertEqual([item.status.status for item in published['/gnss/fix'].messages][-2:], [2, 2])

    def test_adaptation_rejection_publishes_recomputed_invalid_quality_snapshot(self):
        published = self.run_node(qualities=(4, 4), adaptation_failure=True)
        self.assertIn('/gnss/fix_quality', published)
        fix = published['/gnss/fix'].messages[-1]
        message = published['/gnss/fix_quality'].messages[-1]
        self.assertEqual(fix.status.status, -1)
        self.assertIs(message.header.stamp, fix.header.stamp)
        self.assertEqual(values(message)['fix_valid'], 'false')
        self.assertEqual(values(message)['fresh_rtk_usable'], 'false')
        self.assertEqual(values(message)['invalid_reason'], 'upstream_message_adaptation_failed')


if __name__ == '__main__':
    unittest.main(verbosity=2)
