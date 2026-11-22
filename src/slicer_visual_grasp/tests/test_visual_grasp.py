"""全部是假图像/假反馈；不打开相机、底盘或气泵。"""
import copy
import json
from pathlib import Path
import threading
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
import cv2
import numpy as np
from slicer_visual_grasp.paths import DEFAULT_CONFIG
from slicer_visual_grasp.logic import GraspLogic, Observation, Pressure, image_velocity, validate_calibration
from slicer_visual_grasp.vision import PinkDetector, Camera
from slicer_visual_grasp.devices import Motion, PressureLink
from slicer_visual_grasp.nozzle_calibration import capture_sample

ROOT = Path(__file__).resolve().parent
CFG = json.loads(DEFAULT_CONFIG.read_text())
# 仅供旧纯算法状态机的历史回归；现行Runtime已不导入该升降状态机。
CFG['grasp'].update(max_lower_s=3.0,lift_s=1.0)
CFG['target']['color_space']='hsv'
CFG['alignment']['two_point']=False  # 原状态机回归；双点控制另有独立测试
CAL = dict(aim_px=[320,240], jacobian=[[300,800],[-3400,-500]],
           resolution=[640,480], device=CFG['camera']['device'])


class LogicTests(unittest.TestCase):
    def setUp(self):
        self.flow = GraspLogic(copy.deepcopy(CFG), CAL, 0)

    def aligned(self):
        for n in range(CFG['alignment']['stable_frames']):
            self.flow.step(n*.05, Observation(n,n*.05,(320,240)), Pressure(n*.05))
        self.assertEqual(self.flow.state, 'STOP_BASE')

    def lowering(self):
        self.aligned()
        out = self.flow.step(.3, Observation(10,.3,(320,240)), Pressure(.3))
        self.assertTrue(out.pump_on)
        self.flow.step(.4, Observation(11,.4,None), Pressure(.4,True,False,'SUCTION_FREE',True))
        self.assertEqual(self.flow.state, 'LOWER')

    def test_happy_path_feedback_then_one_second_lift(self):
        self.lowering()
        self.assertEqual(self.flow.step(.5, Observation(20,.5,None), Pressure(.5,True,False,'SUCTION_FREE',True)).servo,'lower')
        contact = self.flow.step(.6, Observation(21,.6,None), Pressure(.6,True,True,'SUCTION_ATTACHED',True))
        self.assertEqual(contact.servo,'neutral')
        out = self.flow.step(.7, Observation(22,.7,None), Pressure(.7,True,True,'SUCTION_ATTACHED',True))
        self.assertEqual(out.servo,'lift')
        out = self.flow.step(1.71, Observation(23,1.71,None), Pressure(1.71,True,True,'SUCTION_ATTACHED',True))
        self.assertEqual(self.flow.state,'DONE')
        self.assertEqual(out.servo,'neutral')
        self.assertFalse(out.pump_on)  # 不重复发泵命令，更不自动PUMP_OFF

    def test_ack_alone_never_lowers(self):
        self.aligned()
        self.flow.step(.3,Observation(10,.3,(320,240)),Pressure(.3))
        out = self.flow.step(.4,Observation(11,.4,None),Pressure(.4,False,False,'RELEASE',True))
        self.assertEqual(self.flow.state,'PUMP_START')
        self.assertEqual(out.servo,'neutral')

    def test_duplicate_frame_does_not_count_as_multiple_stable_frames(self):
        for t in (0,.05,.1,.15,.2):
            self.flow.step(t,Observation(1,0,(320,240)),Pressure(t))
        self.assertEqual(self.flow.stable,1)
        self.assertEqual(self.flow.state,'ALIGN')

    def test_loss_stops_without_blind_drive(self):
        out = self.flow.step(.1,Observation(1,.1,None),Pressure(.1))
        self.assertEqual(self.flow.state,'ERROR')
        self.assertTrue(out.immediate_stop)
        self.assertEqual((out.v,out.w,out.servo),(0,0,'neutral'))

    def test_stale_camera_stops(self):
        self.flow.step(1,Observation(1,0,(350,240)),Pressure(1))
        self.assertEqual(self.flow.state,'ERROR')

    def test_aligned_but_base_still_decelerating_does_not_start_pump(self):
        self.aligned()
        out = self.flow.step(.3,Observation(10,.3,(320,240)),Pressure(.3),stopped=False)
        self.assertFalse(out.pump_on)
        self.assertEqual(self.flow.state,'STOP_BASE')

    def test_drift_while_stopping_goes_back_to_alignment(self):
        self.aligned()
        self.flow.step(.3,Observation(10,.3,(350,240)),Pressure(.3))
        self.assertEqual(self.flow.state,'ALIGN')

    def test_stale_pressure_stops_lowering(self):
        self.lowering()
        out = self.flow.step(2,Observation(20,2,None),Pressure(.4,True,False,'SUCTION_FREE',True))
        self.assertTrue(out.immediate_stop)
        self.assertEqual(self.flow.state,'ERROR')

    def test_residual_attach_at_start_is_not_a_new_pick(self):
        self.aligned()
        self.flow.step(.3,Observation(10,.3,(320,240)),Pressure(.3,False,True,'RELEASE'))
        self.assertEqual(self.flow.state,'ERROR')

    def test_descent_limit_and_suction_loss_stop(self):
        self.lowering()
        self.flow.step(3.5,Observation(30,3.5,None),Pressure(3.5,True,False,'SUCTION_FREE',True))
        self.assertEqual(self.flow.state,'ERROR')
        self.flow = GraspLogic(CFG,CAL,0)
        self.lowering()
        self.flow.step(.6,Observation(20,.6,None),Pressure(.6,True,True,'SUCTION_ATTACHED',True))
        out = self.flow.step(.7,Observation(21,.7,None),Pressure(.7,True,False,'SUCTION_FREE',True))
        self.assertEqual(self.flow.state,'ERROR')
        self.assertEqual(out.servo,'neutral')

    def test_image_control_converges_from_all_sides_with_measured_signs(self):
        matrix=np.asarray(CAL['jacobian'],float)
        for offset in ([80,60],[-80,60],[80,-60],[-80,-60]):
            xy=np.asarray(CAL['aim_px'],float)+offset
            for _ in range(300):
                v,w=image_velocity(xy,CAL['aim_px'],matrix,CFG['alignment'])
                self.assertLessEqual(abs(v),CFG['alignment']['max_linear_m_s']+1e-9)
                self.assertLessEqual(abs(w),CFG['alignment']['max_angular_rad_s']+1e-9)
                xy += .05*matrix@np.array((v,w))
            self.assertLess(np.linalg.norm(xy-CAL['aim_px']),.1)

    def test_camera_change_and_singular_calibration_rejected(self):
        validate_calibration(CAL,CFG['camera'])
        for data in ({**CAL,'device':'other'}, {**CAL,'jacobian':[[1,2],[2,4]]}, {**CAL,'resolution':[320,240]}):
            with self.assertRaises(ValueError):
                validate_calibration(data,CFG['camera'])


class DetectionTests(unittest.TestCase):
    @staticmethod
    def image(x=220,y=180):
        image=np.zeros((480,640,3),np.uint8)
        pink=cv2.cvtColor(np.uint8([[[165,170,220]]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
        cv2.rectangle(image,(x,y),(x+100,y+70),pink,-1)
        return image

    def test_pink_center_and_translation(self):
        d=PinkDetector(CFG['target'])
        center,box,_=d.detect(self.image())
        np.testing.assert_allclose(center,[270,215],atol=1)
        center,_,_=d.detect(self.image(225,185))
        np.testing.assert_allclose(center,[275,220],atol=1)

    def test_loss_does_not_relock_another_pink_object(self):
        d=PinkDetector(CFG['target'])
        d.detect(self.image())
        self.assertIsNone(d.detect(np.zeros((480,640,3),np.uint8))[0])
        self.assertIsNone(d.detect(self.image())[0])
        d.select((200,160,150,120))
        self.assertIsNotNone(d.detect(self.image())[0])

    def test_non_pink_is_ignored(self):
        image=np.zeros((480,640,3),np.uint8)
        cv2.rectangle(image,(200,200),(300,280),(255,0,0),-1)
        self.assertIsNone(PinkDetector(CFG['target']).detect(image)[0])

    def test_idle_preview_can_reacquire_after_auto_white_balance(self):
        d=PinkDetector(CFG['target']);d.retain_loss=False
        d.detect(self.image())
        self.assertIsNone(d.detect(np.zeros((480,640,3),np.uint8))[0])
        self.assertIsNotNone(d.detect(self.image())[0])

    def test_pale_pink_hue_wrap_matches_usb_white_balance(self):
        for hue in (5,20,170):
            image=np.zeros((480,640,3),np.uint8)
            pink=cv2.cvtColor(np.uint8([[[hue,20,150]]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
            cv2.rectangle(image,(200,150),(300,220),pink,-1)
            result=PinkDetector(CFG['target']).detect(image)[0]
            self.assertIsNotNone(result, 'hue='+str(hue))
            np.testing.assert_allclose(result,[250,185],atol=1)

    def test_camera_failure_clears_old_box_and_target(self):
        image=self.image()
        class FakeCapture:
            def __init__(self,*args):self.grabs=0;self.released=False
            def set(self,*args):pass
            def isOpened(self):return True
            def get(self,key):
                return 30 if key==cv2.CAP_PROP_FPS else cv2.VideoWriter_fourcc(*'YUYV')
            def grab(self):self.grabs+=1;return self.grabs==1
            def retrieve(self):return True,image
            def release(self):self.released=True;camera.quit.set()
        capture=FakeCapture()
        camera=Camera(CFG['camera'],CFG['target'])
        with patch('slicer_visual_grasp.vision.cv2.VideoCapture',return_value=capture),self.assertLogs('visual_grasp',level='WARNING'):
            camera.run()
        frame,observation,box,error=camera.snapshot()
        self.assertIsNotNone(frame)
        self.assertIsNone(box)
        self.assertIsNone(observation.center)
        self.assertIsNotNone(error)
        self.assertTrue(capture.released)

    def test_camera_reenumeration_reopens_preview_without_restoring_active_action(self):
        camera=Camera(CFG['camera'],CFG['target']);camera.active=True
        calls=[]
        def attempt():
            calls.append(1)
            if len(calls)==1:raise RuntimeError('simulated ENODEV')
            self.assertFalse(camera.active)
        with patch.object(camera,'capture_once',side_effect=attempt),\
                patch.object(camera.quit,'wait',return_value=False),self.assertLogs('visual_grasp',level='WARNING'):
            camera.run()
        self.assertEqual(len(calls),2)
        self.assertIsNone(camera.box)


class IOTests(unittest.TestCase):
    def test_pressure_start_recovers_from_first_ping_timeout(self):
        device = Mock()
        device.request.side_effect = [TimeoutError('PING'), {'pong': '1'},
                                      {'pump': '0', 'attach': '0'}]
        device.receive.return_value = []
        link = PressureLink({'open_settle_s': 0., 'status_poll_s': .1,
                             'link_refresh_s': .25}, lambda _: None)
        with patch('slicer_visual_grasp.devices.AirGrip', return_value=device):
            try:
                link.start()
                self.assertGreater(link.snapshot().stamp, 0.)
            finally:
                link.close()
        self.assertEqual([entry.args[0] for entry in device.request.call_args_list],
                         ['PING', 'PING', 'GET_STATUS'])
        device.set_pump.assert_not_called()

    def test_partial_status_cannot_refresh_old_pressure(self):
        p=PressureLink({},lambda s:None)
        p.update(dict(pump='1',attach='0',system_state='SUCTION_FREE'))
        before=p.snapshot()
        p.update(dict(code='OK',connected='1'))
        self.assertEqual(p.snapshot(),before)
        p.on_ack=True
        p.update(dict(pump='1',attach='1',system_state='SUCTION_ATTACHED'))
        self.assertTrue(p.snapshot().on_ack)
        self.assertTrue(p.snapshot().attached)

    def test_motion_never_queries_or_controls_servo(self):
        class FakeCar:
            def __init__(self):self.pulses=[];self.speeds=[]
            def healthy(self):pass
            def speed(self,v,w):self.speeds.append((v,w))
            def servo_pulse(self,v):raise AssertionError('不得发送舵机PWM')
            def servo_check(self):raise AssertionError('不得查询舵机')
            def connect(self):pass
            def close(self):self.closed=True
        class FakeSlew:
            v=w=0
            def step(self,v,w,dt):self.v,self.w=v,w;return v,w
        from slicer_visual_grasp import devices
        fake=FakeCar()
        with patch.object(devices,'load_config',return_value={'control':{'publish_hz':100000}}),\
             patch.object(devices,'FeedbackChassis',return_value=fake),patch.object(devices,'Slew',return_value=FakeSlew()):
            m=Motion({'hardware_config':'unused'},ROOT,threading.Event(),lambda s:None)
        m.tick(.04,.1);m.tick();m.close()
        self.assertEqual(m.car.speeds,[(.04,.1),(0,0)])
        self.assertTrue(m.car.closed)
        m.cancelled.set()
        with self.assertRaises(InterruptedError):m.tick(.1,.1)

    def test_closing_pressure_transport_never_auto_releases(self):
        calls=[]
        class FakeAirGrip:
            def __init__(self,cfg):self.ser=self
            def request(self,cmd):return dict(pump='1',attach='1',system_state='SUCTION_ATTACHED')
            def close(self):calls.append('CLOSE_TRANSPORT')
            def set_pump(self,on):calls.append(('PUMP',on))
        p=PressureLink({},lambda s:None)
        p.quit.set()
        with patch('slicer_visual_grasp.devices.AirGrip',FakeAirGrip):p.run()
        self.assertEqual(calls,['CLOSE_TRANSPORT'])


class CaptureTests(unittest.TestCase):
    def test_capture_saves_images_and_separate_face_pixels_without_actuators(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);now=time.monotonic();frame=np.zeros((480,640,3),np.uint8)
            first=Observation(1,now,(293,171))
            result,folder=capture_sample(root,'face1',frame,first,[first],CFG['camera'])
            self.assertEqual(result['aim_px'],[293,171])
            self.assertEqual(result['offset_from_image_center_px'],[-27,-69])
            self.assertFalse(result['actuator_commands_sent'])
            self.assertFalse(result['nozzle_target_calibrated'])
            self.assertEqual(result['sample_kind'],'reachable_placement')
            for name in ('sample.json','image.jpg','marked.png'):self.assertTrue((folder/name).exists())
            second=Observation(2,now,(300,190))
            capture_sample(root,'face2',frame,second,[],CFG['camera'])
            profiles=json.loads((root/'data/nozzle_profiles.json').read_text())
            self.assertEqual(profiles['face1']['object_center_px'],[293,171])
            self.assertEqual(profiles['face2']['object_center_px'],[300,190])
            self.assertFalse((root/'data/aim.json').exists())

    def test_capture_rejects_lost_or_stale_frame(self):
        with tempfile.TemporaryDirectory() as temp:
            for obs in (Observation(2,time.monotonic()-2,(293,171)),):
                with self.assertRaises(ValueError):
                    capture_sample(Path(temp),'face1',np.zeros((480,640,3),np.uint8),obs,[],CFG['camera'])


if __name__=='__main__':
    unittest.main()
