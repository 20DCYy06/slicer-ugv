import copy
import json
from pathlib import Path
import tempfile
import unittest
from protocol import move_frame, hid_report, voltage_frame, voltage_mv, speed_pulse, angle_pulse, load_config
from hid_bridge import boards

CFG = load_config(Path(__file__).with_name('config.json'))


class ProtocolTest(unittest.TestCase):
    def test_move_little_endian_and_channel_identity(self):
        # 厂家9路2300us/800ms样例中该路的完整单路形式。
        self.assertEqual(move_frame(9, 2300, 800).hex(), '5555080301200309fc08')
        self.assertEqual(move_frame(8, 1500, 300).hex(), '55550803012c0108dc05')

    def test_reject_other_hardware_channels(self):
        for ch in (0, 1, 7, 10, 15):
            with self.assertRaises(ValueError): move_frame(ch, 1500, 300)
        for pulse in (499, 2501):
            with self.assertRaises(ValueError): move_frame(8, pulse, 300)

    def test_hid_framing(self):
        r = hid_report(voltage_frame())
        self.assertEqual(len(r), 65)
        self.assertEqual(r[:5], bytes.fromhex('005555020f'))
        self.assertEqual(r[5:], bytes(60))

    def test_voltage_ignores_usb_padding_and_rejects_truncated(self):
        self.assertEqual(voltage_mv(bytes.fromhex('5555040fc119') + b'\xff' * 58), 6593)
        self.assertIsNone(voltage_mv(bytes.fromhex('5555040fc1')))
        self.assertIsNone(voltage_mv(b'random data'))

    def test_continuous_mapping_and_neutral(self):
        s = CFG['channel8']
        self.assertEqual([speed_pulse(v, s) for v in (-100, 0, 100)], [1000, 1500, 2000])
        reverse = dict(s, direction=-1)
        self.assertEqual(speed_pulse(50, reverse), speed_pulse(-50, s))
        for v in (-101, 101):
            with self.assertRaises(ValueError): speed_pulse(v, s)

    def test_positional_mapping(self):
        p = CFG['channel9']
        self.assertEqual([angle_pulse(v, p) for v in (0, 90, 180)], [500, 1500, 2500])
        self.assertEqual(angle_pulse(0, dict(p, direction=-1)), 2500)
        with self.assertRaises(ValueError): angle_pulse(181, p)

    def test_config_validation(self):
        c = copy.deepcopy(CFG)
        c['channel8']['neutral_us'] = 2600
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'invalid.json'
            p.write_text(json.dumps(c))
            with self.assertRaises(ValueError): load_config(p)


if __name__ == '__main__':
    unittest.main()
