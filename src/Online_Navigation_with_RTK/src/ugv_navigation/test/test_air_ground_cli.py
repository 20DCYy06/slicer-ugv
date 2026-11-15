#!/usr/bin/env python3
"""Computer sender boundaries; no master, network, devices or motion."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

try:
    import air_ground as cli
    import air_ground_wire as wire
except ImportError:
    cli = wire = None


class PointMessage:
    def __init__(self):
        self.latitude = self.longitude = 0.
        self.has_altitude = self.has_yaw = False
        self.altitude_ellipsoid_m = self.yaw = 0.


class CommandMessage:
    def __init__(self):
        self.final_goal = PointMessage()
        self.via_points = []


class AirGroundCliTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(cli, 'air_ground computer tool must be implemented')
        self.assertIsNotNone(wire, 'formal wire helper must be implemented')

    def state(self, **updates):
        state = dict(protocol_version=1, server_id='receiver-boot', last_sequence=18,
                     task_id='active-task', phase='RUNNING', updated_stamp=100.)
        state.update(updates)
        return state

    def point(self, **updates):
        point = dict(latitude=22., longitude=113., altitude_ellipsoid_m=None, yaw=None)
        point.update(updates)
        return point

    def record(self, **updates):
        value = dict(kind='slicer_geographic_target', version=1, datum='WGS84',
                     physical_reference='base_link_horizontal', latitude=22., longitude=113.,
                     altitude_ellipsoid_m=18.4,
                     height_semantics='GNSS antenna ellipsoid height; horizontal base_link')
        value.update(updates)
        return value

    def parse_error(self, argv):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            cli.arguments(argv)

    def test_parser_has_five_operations_and_status(self):
        args = cli.arguments(['set', '--latitude', '22', '--longitude', '113'])
        self.assertIsNone(args.altitude_ellipsoid)
        self.assertIsNone(args.yaw)
        self.assertEqual(cli.arguments(['replace-via']).via_file, [])
        for operation in ('pause', 'resume', 'cancel', 'status'):
            self.assertEqual(cli.arguments([operation]).command, operation)
        self.assertEqual(str(cli.arguments(['replay', 'request.json']).request_file), 'request.json')

    def test_parser_rejects_mixed_partial_invalid_coordinates_and_invalid_waits(self):
        for argv in (['set'], ['set', '--latitude', '22'],
                     ['set', '--latitude', '91', '--longitude', '113'],
                     ['set', '--latitude', 'nan', '--longitude', '113'],
                     ['set', '--target-file', 'mark.json', '--longitude', '113'],
                     ['set', '--target-file', 'mark.json', '--altitude-ellipsoid', '3'],
                     ['pause', '--receipt-timeout', '0']):
            with self.subTest(argv=argv):
                self.parse_error(argv)

    def test_target_file_keeps_wgs84_reference_and_height_without_lever_or_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mark.json'
            path.write_text(json.dumps(self.record(capture={'generation': 1})))
            self.assertEqual(wire.load_target(path, yaw=.7), self.point(altitude_ellipsoid_m=18.4, yaw=.7))
            self.assertIsNone(wire.load_target(path)['yaw'])

    def test_target_file_rejects_wrong_schema_datum_reference_and_nonfinite_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mark.json'
            for update in (dict(kind='other'), dict(version=True), dict(version=2),
                           dict(datum='MSL'), dict(physical_reference='antenna'),
                           dict(latitude=91), dict(longitude=True),
                           dict(altitude_ellipsoid_m=None), dict(altitude_ellipsoid_m=float('nan'))):
                with self.subTest(update=update), self.assertRaises(ValueError):
                    path.write_text(json.dumps(self.record(**update)))
                    wire.load_target(path)

    def test_new_set_uses_receiver_sequence_and_new_uuid_request_and_task(self):
        args = cli.arguments(['set', '--latitude', '22', '--longitude', '113', '--yaw', '.7'])
        command = cli.build_request(args, self.state(), identifier=iter(['request-uuid', 'task-uuid']).__next__)
        self.assertEqual(command, dict(protocol_version=1, server_id='receiver-boot', sequence=19,
                                      request_id='request-uuid', task_id='task-uuid', operation='SET_TASK',
                                      final_goal=self.point(yaw=.7), via_points=[]))

    def test_other_operations_use_current_task_and_empty_replace_clears(self):
        for name, operation in (('replace-via', 'REPLACE_VIA_POINTS'), ('pause', 'PAUSE'),
                                ('resume', 'RESUME'), ('cancel', 'CANCEL')):
            command = cli.build_request(cli.arguments([name]), self.state(), identifier=lambda: 'request')
            self.assertEqual((command['operation'], command['task_id'], command['sequence']),
                             (operation, 'active-task', 19))
            self.assertIsNone(command['final_goal'])
            self.assertEqual(command['via_points'], [])
        with self.assertRaisesRegex(ValueError, 'task'):
            cli.build_request(cli.arguments(['resume']), self.state(task_id=''))

    def test_batch_preserves_file_order_and_validates_every_point_before_publish(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory) / 'a.json', Path(directory) / 'b.json'
            first.write_text(json.dumps(self.record(latitude=22.1)))
            second.write_text(json.dumps(self.record(latitude=22.2)))
            args = cli.arguments(['set', '--target-file', str(first), '--via-file', str(second),
                                  '--via-file', str(first)])
            command = cli.build_request(args, self.state())
            self.assertEqual([point['latitude'] for point in command['via_points']], [22.2, 22.1])
            second.write_text(json.dumps(self.record(datum='wrong')))
            with self.assertRaises(ValueError):
                cli.build_request(args, self.state())

    def test_state_freshness_checks_receive_and_ros_stamp_and_boot_envelope(self):
        self.assertIsNone(wire.state_problem(self.state(), 10., 10.2, 100.2, max_age=1.))
        for state, received, now, ros_now in (
                (self.state(), None, 10., 100.), (self.state(), 8., 10., 100.),
                (self.state(), 10., 10.2, 102.), (self.state(updated_stamp=100.3), 10., 10.2, 100.2),
                (self.state(protocol_version=True), 10., 10.2, 100.2),
                (self.state(server_id=''), 10., 10.2, 100.2),
                (self.state(last_sequence=True), 10., 10.2, 100.2),
                (self.state(last_sequence=-1), 10., 10.2, 100.2)):
            with self.subTest(state=state, received=received, ros_now=ros_now):
                self.assertIsNotNone(wire.state_problem(state, received, now, ros_now, max_age=1.))

    def test_sequence_exhaustion_does_not_wrap(self):
        with self.assertRaisesRegex(ValueError, 'sequence'):
            cli.build_request(cli.arguments(['cancel']), self.state(last_sequence=2**64-1))

    def test_receiver_bootstrap_and_sender_restart_need_no_persistent_sender_lease(self):
        state = self.state(last_sequence=None, task_id=None)
        self.assertIsNone(wire.state_problem(state, 10., 10.2, 100.2))
        args = cli.arguments(['set', '--latitude', '22', '--longitude', '113'])
        self.assertEqual(cli.build_request(args, state)['sequence'], 1)
        # A fresh sender uses the receiver's current global sequence.
        self.assertEqual(cli.build_request(args, self.state(last_sequence=220))['sequence'], 221)

    def test_saved_request_replay_keeps_exact_identity_and_content_and_never_overwrites(self):
        command = cli.build_request(cli.arguments(['pause']), self.state())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'request.json'
            cli.save_request(path, command)
            replay = cli.build_request(cli.arguments(['replay', str(path)]), self.state(last_sequence=50))
            self.assertEqual(replay, command)
            with self.assertRaises(FileExistsError):
                cli.save_request(path, command)
            with self.assertRaisesRegex(ValueError, 'server'):
                cli.build_request(cli.arguments(['replay', str(path)]), self.state(server_id='new-boot'))

    def test_message_mapping_preserves_optional_fields_and_five_formal_operations(self):
        for name in ('pause', 'resume', 'cancel', 'replace-via'):
            original = cli.build_request(cli.arguments([name]), self.state())
            message = wire.make_command(original, CommandMessage, PointMessage)
            self.assertEqual(wire.command_to_dict(message), original)
        original = cli.build_request(cli.arguments(['set', '--latitude', '22', '--longitude', '113']), self.state())
        message = wire.make_command(original, CommandMessage, PointMessage)
        self.assertFalse(message.final_goal.has_altitude)
        self.assertFalse(message.final_goal.has_yaw)
        self.assertEqual(message.final_goal.altitude_ellipsoid_m, 0.)
        self.assertEqual(wire.command_to_dict(message), original)
        original['final_goal'] = self.point(altitude_ellipsoid_m=0., yaw=0.)
        message = wire.make_command(original, CommandMessage, PointMessage)
        self.assertTrue(message.final_goal.has_altitude)
        self.assertTrue(message.final_goal.has_yaw)
        self.assertEqual(wire.command_to_dict(message), original)

    def test_formal_mapping_rejects_yaw_on_via_invalid_batch_and_version(self):
        command = cli.build_request(cli.arguments(['set', '--latitude', '22', '--longitude', '113']), self.state())
        for update in (dict(protocol_version=2), dict(sequence=0), dict(request_id=''),
                       dict(operation='SEND'), dict(via_points=[self.point(yaw=0.)]),
                       dict(via_points=[self.point(longitude=181.)])):
            with self.subTest(update=update), self.assertRaises(ValueError):
                wire.make_command(dict(command, **update), CommandMessage, PointMessage)

    def test_matching_receipt_requires_identity_and_explicit_accepted(self):
        request = cli.build_request(cli.arguments(['pause']), self.state())
        self.assertIsNone(cli.matching_receipt(self.state(), request))
        state = self.state(last_command=dict(request_id=request['request_id'], accepted=True))
        self.assertEqual(cli.matching_receipt(state, request)['accepted'], True)
        self.assertIsNone(cli.matching_receipt(dict(state, server_id='another-boot'), request))
        self.assertIsNone(cli.matching_receipt(self.state(last_command=dict(request_id='old', accepted=True)), request))
        self.assertIsNone(cli.matching_receipt(self.state(last_command=dict(request_id=request['request_id'])), request))

    def test_ros_constructor_uses_exact_public_topics_and_nonlatched_formal_command(self):
        calls = []
        fake_ros = SimpleNamespace(
            Subscriber=lambda *args, **kwargs: calls.append(('subscriber', args, kwargs)),
            Publisher=lambda *args, **kwargs: calls.append(('publisher', args, kwargs)))
        modules = {'rospy': fake_ros,
                   'slicer_air_ground_msgs.msg': SimpleNamespace(Command=CommandMessage, GeoPoint=PointMessage),
                   'std_msgs.msg': SimpleNamespace(String=str)}
        with patch.dict(sys.modules, modules):
            cli.AirGroundClient(cli.arguments(['pause']))
        self.assertEqual(calls[0][1][:2], ('/air_ground/state', str))
        self.assertEqual(calls[1][1], ('/air_ground/command', CommandMessage))
        self.assertEqual(calls[1][2], dict(queue_size=1, latch=False))

    def test_ros_sender_uses_formal_command_nonlatched_and_publishes_once(self):
        client = cli.AirGroundClient.__new__(cli.AirGroundClient)
        command = cli.build_request(cli.arguments(['pause']), self.state())
        sent = []
        client.publisher = SimpleNamespace(get_num_connections=lambda: 1, publish=sent.append)
        client.ros = SimpleNamespace(is_shutdown=lambda: False)
        client.command_type, client.point_type = CommandMessage, PointMessage
        client.args = cli.arguments(['pause'])
        client.publish_once(command)
        self.assertEqual(len(sent), 1)
        self.assertEqual(wire.command_to_dict(sent[0]), command)

    def test_no_subscriber_is_bounded_and_publishes_nothing(self):
        client = cli.AirGroundClient.__new__(cli.AirGroundClient)
        sent = []
        client.publisher = SimpleNamespace(get_num_connections=lambda: 0, publish=sent.append)
        client.ros = SimpleNamespace(is_shutdown=lambda: False)
        client.command_type, client.point_type = CommandMessage, PointMessage
        client.args = cli.arguments(['pause', '--connect-timeout', '.1'])
        with patch.object(cli.time, 'monotonic', side_effect=[10., 10.2]):
            with self.assertRaisesRegex(RuntimeError, 'not published'):
                client.publish_once(cli.build_request(client.args, self.state()))
        self.assertEqual(sent, [])

    def test_only_explicit_accepted_receipt_exits_zero_and_prints_receipt_semantics(self):
        for accepted, expected_code in ((True, 0), (False, 1)):
            client = cli.AirGroundClient.__new__(cli.AirGroundClient)
            client.args = cli.arguments(['pause'])
            client.fresh_state = self.state
            sent = []
            client.publish_once = sent.append
            client.wait_receipt = lambda request: dict(request_id=request['request_id'], accepted=accepted)
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(client.run(), expected_code)
            self.assertEqual(len(sent), 1)
            self.assertIn('not task completion', json.loads(output.getvalue())['meaning'])

    def test_receipt_timeout_does_not_retry_and_leaves_exact_request_saved(self):
        client = cli.AirGroundClient.__new__(cli.AirGroundClient)
        client.fresh_state = self.state
        sent = []
        client.publish_once = sent.append
        client.wait_receipt = lambda request: (_ for _ in ()).throw(RuntimeError('receipt timeout'))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'request.json'
            client.args = cli.arguments(['pause', '--save-request', str(path)])
            with self.assertRaisesRegex(RuntimeError, 'receipt timeout'):
                client.run()
            self.assertEqual(len(sent), 1)
            self.assertEqual(json.loads(path.read_text()), sent[0])

    def test_wire_rejects_nondefault_payload_for_unused_final_goal(self):
        command = cli.build_request(cli.arguments(['pause']), self.state())
        message = wire.make_command(command, CommandMessage, PointMessage)
        message.final_goal.latitude = 22.
        with self.assertRaisesRegex(ValueError, 'final_goal'):
            wire.command_to_dict(message)

    def test_receiver_decoder_preserves_invalid_coordinates_and_protocol_for_core_rejection(self):
        self.assertTrue(hasattr(wire, 'decode_command'))
        command = cli.build_request(cli.arguments(['set', '--latitude', '22', '--longitude', '113']), self.state())
        message = wire.make_command(command, CommandMessage, PointMessage)
        message.protocol_version = 9
        message.final_goal.latitude = 91.
        message.final_goal.has_altitude = True
        message.final_goal.altitude_ellipsoid_m = math.nan
        decoded = wire.decode_command(message)
        self.assertEqual(decoded['protocol_version'], 9)
        self.assertEqual(decoded['final_goal']['latitude'], 91.)
        self.assertTrue(math.isnan(decoded['final_goal']['altitude_ellipsoid_m']))
        with self.assertRaises(ValueError):
            wire.command_to_dict(message)

    def test_receiver_decoder_preserves_unused_payload_and_explicit_zero_via_yaw(self):
        self.assertTrue(hasattr(wire, 'decode_command'))
        command = cli.build_request(cli.arguments(['pause']), self.state())
        message = wire.make_command(command, CommandMessage, PointMessage)
        message.final_goal.latitude = 22.
        via = PointMessage()
        via.has_yaw = True
        via.yaw = 0.
        message.via_points = [via]
        decoded = wire.decode_command(message)
        self.assertEqual(decoded['final_goal'], self.point(longitude=0.))
        self.assertEqual(decoded['via_points'][0]['yaw'], 0.)
        self.assertEqual(decoded['operation'], 'PAUSE')

    def test_receiver_decoder_ignores_numeric_values_when_optional_flags_are_false(self):
        self.assertTrue(hasattr(wire, 'decode_command'))
        command = cli.build_request(cli.arguments(['pause']), self.state())
        message = wire.make_command(command, CommandMessage, PointMessage)
        message.final_goal.altitude_ellipsoid_m = math.nan
        message.final_goal.yaw = 1.2
        self.assertIsNone(wire.decode_command(message)['final_goal'])
        message.operation = 'SET_TASK'
        message.final_goal.latitude, message.final_goal.longitude = 22., 113.
        self.assertEqual(wire.decode_command(message)['final_goal'], self.point())

    def test_invalid_receiver_request_is_cached_before_same_identity_changes_content(self):
        self.assertTrue(hasattr(wire, 'decode_command'))
        from air_ground_core import TaskCore
        for invalid_latitude in (91., math.nan):
            with self.subTest(latitude=invalid_latitude):
                command = cli.build_request(cli.arguments(['set', '--latitude', '22', '--longitude', '113']),
                                            self.state(last_sequence=None))
                command['request_id'] = 'R'
                message = wire.make_command(command, CommandMessage, PointMessage)
                message.final_goal.latitude = invalid_latitude
                core = TaskCore('receiver-boot')
                rejected = core.handle(wire.decode_command(message))
                self.assertFalse(rejected['accepted'])
                self.assertIn('latitude', rejected['reason'])
                self.assertTrue(core.handle(wire.decode_command(message))['duplicate'])
                message.final_goal.latitude = 31.
                changed = core.handle(wire.decode_command(message))
                self.assertFalse(changed['accepted'])
                self.assertIn('different content', changed['reason'])
                self.assertIsNone(core.snapshot()['task_id'])

    def test_message_package_is_minimal_and_has_no_navigation_dependencies(self):
        package = ROOT.parent / 'slicer_air_ground_msgs'
        self.assertTrue((package / 'package.xml').is_file())
        cmake = (package / 'CMakeLists.txt').read_text()
        self.assertIn('generate_messages', cmake)
        geo_fields = [row.strip() for row in (package / 'msg/GeoPoint.msg').read_text().splitlines()
                      if row.strip() and not row.startswith('#')]
        self.assertEqual(geo_fields, ['float64 latitude', 'float64 longitude', 'bool has_altitude',
                                      'float64 altitude_ellipsoid_m', 'bool has_yaw', 'float64 yaw'])
        command_fields = [row.strip() for row in (package / 'msg/Command.msg').read_text().splitlines()
                          if row.strip() and not row.startswith('#') and '=' not in row]
        self.assertEqual(command_fields, ['uint8 protocol_version', 'string server_id', 'uint64 sequence',
                                          'string request_id', 'string task_id', 'string operation',
                                          'GeoPoint final_goal', 'GeoPoint[] via_points'])
        self.assertNotIn('move_base', (package / 'package.xml').read_text())


if __name__ == '__main__':
    unittest.main()
