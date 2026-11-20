import io
import contextlib
import unittest
from protocol import crc16_modbus,encode_frame,parse_frame,parse_kv
from keyboard import AirGrip


class FakeSerial:
    def __init__(self,reply=True):self.frames=[];self.buf=bytearray();self.reply=reply;self.closed=False
    @property
    def in_waiting(self):return len(self.buf)
    def write(self,b):
        self.frames.append(b)
        ok,req=parse_frame(b.decode());assert ok
        if self.reply:
            self.buf.extend(encode_frame('0','EVT','system_state=RELEASE').encode())
            self.buf.extend(encode_frame(req['seq'],'RSP','code=OK;system_state=RELEASE').encode())
        return len(b)
    def read(self,n):
        # 故意把每一行拆成碎片，模拟USB分包。
        n=min(n,3);data=bytes(self.buf[:n]);del self.buf[:n];return data
    def close(self):self.closed=True


def fake(reply=True):
    a=AirGrip.__new__(AirGrip);a.cfg={'ack_timeout_s':.01};a.seq=0;a.buffer=bytearray();a.status={};a.must_release=False;a.ser=FakeSerial(reply);return a


class AirGripTests(unittest.TestCase):
    def test_crc_standard(self):self.assertEqual(crc16_modbus(b'123456789'),0x4b37)
    def test_protocol_matches_upstream_frames(self):
        # Captured outputs of the upstream AirGrip host encoder.
        expected = [
            (('1', 'PUMP_ON', ''), '@1|PUMP_ON||95C9\n'),
            (('2', 'PUMP_OFF', ''), '@2|PUMP_OFF||91C8\n'),
            (('3', 'GET_STATUS', ''), '@3|GET_STATUS||96AF\n'),
            (('4', 'SET_LINK', 'connected=1'), '@4|SET_LINK|connected=1|D777\n'),
        ]
        for fields, frame in expected:
            self.assertEqual(encode_frame(*fields), frame)
    def test_bad_crc_rejected(self):self.assertFalse(parse_frame('@1|PUMP_ON||0000')[0])
    def test_fragmented_ack_and_event(self):
        a=fake();self.assertEqual(a.request('PING')['code'],'OK');self.assertEqual(a.status['system_state'],'RELEASE')
    def test_query_and_close_never_changes_pump(self):
        a=fake();a.request('PING');a.request('GET_STATUS');a.close()
        self.assertEqual([parse_frame(b.decode())[1]['cmd'] for b in a.ser.frames],['PING','GET_STATUS'])
    def test_on_off_mapping(self):
        a=fake()
        with contextlib.redirect_stdout(io.StringIO()):a.set_pump(True);a.set_pump(False)
        self.assertFalse(a.must_release)
        self.assertEqual([parse_frame(b.decode())[1]['cmd'] for b in a.ser.frames],['PUMP_ON','PUMP_OFF'])
    def test_exit_releases_if_on_attempted(self):
        a=fake()
        with contextlib.redirect_stdout(io.StringIO()):a.set_pump(True);a.close()
        self.assertEqual(parse_frame(a.ser.frames[-1].decode())[1]['cmd'],'PUMP_OFF')
    def test_lost_ack_is_not_success(self):
        a=fake(False)
        with self.assertRaises(TimeoutError):a.set_pump(True)
        self.assertTrue(a.must_release)
