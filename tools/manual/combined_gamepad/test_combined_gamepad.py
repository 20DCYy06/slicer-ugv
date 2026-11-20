import json
import os
from pathlib import Path
import struct
import threading
import time
import unittest
from unittest.mock import patch
import combined_gamepad as c

HERE=Path(__file__).resolve().parent
MAP=json.loads((HERE/'combined_gamepad.json').read_text())['buttons']
SERVO=json.loads((HERE.parent/'b15_servo/config.json').read_text())


class FakePump:
    instances=[]
    def __init__(self,cfg):
        self.cfg=cfg;self.status={};self.calls=[];self.closed=False
        self.on_started=threading.Event();self.allow_ack=threading.Event()
        self.__class__.instances.append(self)
    def request(self,cmd):return {'code':'OK','fw':'1.2T'}
    def send(self,*args):pass
    def receive(self):return []
    def set_pump(self,on):
        self.calls.append(on)
        if on:self.on_started.set();self.allow_ack.wait(.4)
        return {'code':'OK','pump':str(int(on))}
    def close(self):self.closed=True


class IntegrationTests(unittest.TestCase):
    def test_user_xyba_numbers(self):
        self.assertEqual((MAP['suction'],MAP['clockwise'],MAP['release'],MAP['counterclockwise']),(3,0,1,2))
    def test_hold_release_and_both_stop(self):
        self.assertEqual(c.servo_target({0},MAP,SERVO),SERVO['clockwise_us'])
        self.assertEqual(c.servo_target({2},MAP,SERVO),SERVO['counterclockwise_us'])
        for held in (set(),{0,2},{1},{3}):self.assertEqual(c.servo_target(held,MAP,SERVO),1500)
    def test_b_release_not_drive_stop_and_release_wins(self):
        self.assertTrue(c.pump_target([3],MAP))
        self.assertIs(c.pump_target([1],MAP),False)
        self.assertIs(c.pump_target([3,1],MAP),False)
        self.assertIsNone(c.pump_target([0,2,7],MAP))
    def test_event_down_up_init(self):
        joy=c.HeldJoystick.__new__(c.HeldJoystick);joy.path='/fake';joy.fd=99;joy.axes={};joy.held=set()
        events=[c.EVENT.pack(0,1,0x81,0),c.EVENT.pack(1,1,1,0)]
        def read(fd,n):
            if events:return events.pop(0)
            raise BlockingIOError()
        with patch.object(c.os.path,'exists',return_value=True),patch.object(c.os,'read',side_effect=read):
            axes,pressed=joy.poll();self.assertEqual(pressed,[0]);self.assertEqual(joy.held,{0})
            events.extend([c.EVENT.pack(2,0,1,0)])
            axes,pressed=joy.poll();self.assertEqual(pressed,[]);self.assertEqual(joy.held,set())
    def test_initial_button_does_not_start(self):
        joy=c.HeldJoystick.__new__(c.HeldJoystick);joy.path='/fake';joy.fd=99;joy.axes={};joy.held=set()
        with patch.object(c.os.path,'exists',return_value=True),patch.object(c.os,'read',side_effect=[c.EVENT.pack(0,1,0x81,3),BlockingIOError()]):
            axes,pressed=joy.poll();self.assertEqual(pressed,[]);self.assertEqual(joy.held,set())
    def test_same_chassis_sender_for_drive_and_servo(self):
        car=c.CombinedChassis.__new__(c.CombinedChassis)
        car.config=c.gamepad_config(c.load_config());car.nonce=123;frames=[]
        car.send=lambda kind,payload=b'':frames.append((kind,payload))
        car.speed(.2,.1);car.servo_pulse(2000);car.servo_pulse(1500)
        self.assertEqual([f[0] for f in frames],[0x16,0x31,0x31])
        self.assertEqual(struct.unpack('<Ihh',frames[0][1]),(123,200,100))
        self.assertEqual(struct.unpack('<H',frames[-1][1]),(1500,))
    def test_pb14_and_legacy_pb15_capability(self):
        for magic,pin in ((b'B14P',14),(b'B15P',15)):
            car=c.CombinedChassis.__new__(c.CombinedChassis)
            car.replies={}
            def send(kind, magic=magic, pin=pin):
                self.assertEqual(kind,0x30)
                car.replies[0x86]=struct.pack('<4sBBHH',magic,1,pin,1500,500)
            car.send=send
            car.wait=lambda predicate:self.assertTrue(predicate())
            car.servo_check()
            self.assertEqual(car.servo_pin,pin)
        car.replies={}
        car.send=lambda kind:car.replies.update({0x86:struct.pack('<4sBBHH',b'B14P',1,15,1500,500)})
        with self.assertRaises(RuntimeError):car.servo_check()

    def test_existing_drive_limits_and_directions(self):
        cfg=c.gamepad_config(c.load_config());gear=c.Gear(cfg)
        v,w=c.gamepad_target({1:-1,2:-1},gear,cfg['gamepad'])
        self.assertEqual((v,w),(.1,.05))
        for _ in range(30):gear.change(1)
        self.assertEqual((gear.v,gear.w),(1.,.5))
    def test_combined_config_uses_recovered_speed_and_shared_devices(self):
        config = json.loads((HERE / 'combined_gamepad.json').read_text())
        cfg = c.gamepad_config(c.load_config(HERE / config['hardware_config']))
        self.assertEqual(cfg['speed']['linear_max'], 1.)
        for key in ('servo_config', 'airgrip_config'):
            self.assertTrue((HERE / config[key]).is_file())

    def test_pump_ack_does_not_block_caller_and_latest_release(self):
        cfg=json.loads((HERE.parent/'airgrip/config.json').read_text())
        worker=c.PumpWorker(cfg,FakePump)
        try:
            worker.start();fake=FakePump.instances[-1]
            worker.command(True);self.assertTrue(fake.on_started.wait(.3))
            # 设备ACK等待尚未结束，调用者仍可连续处理底盘/舵机控制。
            self.assertTrue(worker.thread.is_alive());start=time.monotonic()
            for _ in range(100):worker.command(True)
            worker.command(False)
            self.assertLess(time.monotonic()-start,.1)
            fake.allow_ack.set();end=time.monotonic()+.5
            while fake.calls!=[True,False] and time.monotonic()<end:time.sleep(.005)
            self.assertEqual(fake.calls,[True,False])
        finally:worker.close()
        self.assertTrue(fake.closed)
    def test_protocol_namespaces_are_independent(self):
        import protocol
        import airgrip.protocol as air_protocol
        self.assertTrue(hasattr(protocol,'packet'))
        self.assertTrue(hasattr(air_protocol,'encode_frame'))
        self.assertIsNot(protocol,air_protocol)


if __name__=='__main__':unittest.main()
