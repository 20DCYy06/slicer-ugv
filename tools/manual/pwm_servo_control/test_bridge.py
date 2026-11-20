"""用假HID验证按住/松开生命周期；绝不打开设备。"""
import contextlib
import io
import json
import unittest
from unittest.mock import patch
import hid_bridge
from protocol import move_frame


class FakeBoard:
    def __init__(self):
        self.path = 'fake/no-hardware'
        self.frames = []
        self.closed = False

    def voltage(self):
        return 6600

    def send(self, frame):
        self.frames.append(frame)
        return frame.hex(' ')

    def close(self):
        self.closed = True


class BridgeTest(unittest.TestCase):
    def run_requests(self, requests):
        board = FakeBoard()
        data = ''.join(json.dumps(r) + '\n' for r in requests)
        with patch.object(hid_bridge, 'Board', return_value=board), \
             patch('sys.argv', ['hid_bridge.py']), patch('sys.stdin', io.StringIO(data)), \
             contextlib.redirect_stdout(io.StringIO()):
            hid_bridge.main()
        self.assertTrue(board.closed)
        return board.frames

    def test_connect_and_quit_has_no_move(self):
        self.assertEqual(self.run_requests([{'op': 'quit'}]), [])

    def test_release_sends_neutral_without_ramp(self):
        frames = self.run_requests([{'op': 'speed8', 'value': 100}, {'op': 'stop8'}, {'op': 'quit'}])
        self.assertEqual(frames, [move_frame(8, 2000, 300), move_frame(8, 1500, 0)])

    def test_eof_stops_active_motor_without_ramp(self):
        frames = self.run_requests([{'op': 'speed8', 'value': -100}])
        self.assertEqual(frames[-1], move_frame(8, 1500, 0))

    def test_position_only_quit_does_not_move_8(self):
        frames = self.run_requests([{'op': 'angle9', 'value': 90}, {'op': 'quit'}])
        self.assertEqual(frames, [move_frame(9, 1500, 300)])
