import copy,json,math,tempfile,threading,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from slicer_visual_grasp.paths import DEFAULT_CONFIG
from slicer_visual_grasp.logic import Observation,Pressure
from slicer_visual_grasp.snapshot_move import make_plan,LockedMove,FeedbackPose,wrap
from slicer_visual_grasp.pipeline import Runtime
ROOT=Path(__file__).resolve().parent
CFG=json.loads(DEFAULT_CONFIG.read_text())
Q=[[200,150],[400,150],[400,290],[200,290]]


class SnapshotTests(unittest.TestCase):
    def test_manual_servo_jog_uses_configured_direction_and_releases_link(self):
        from slicer_visual_grasp import pipeline
        events = []
        cancel = threading.Event()

        class FakeServo:
            def __init__(self, path): events.append(('open', path.name))
            def check(self): events.append(('check',))
            def pulse(self, value): events.append(('pulse', value)); cancel.set()
            def receive(self): return []
            def close(self): events.append(('close',))

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'servo.json').write_text(json.dumps({
                'clockwise_us': 2100, 'counterclockwise_us': 900,
                'neutral_us': 1500, 'tap_duration_s': 0.3, 'refresh_hz': 20,
            }))
            for setting, expected in [('clockwise', 2100), ('counterclockwise', 900)]:
                cancel.clear()
                events.clear()
                cfg = dict(CFG, servo_config='servo.json', servo_up_direction=setting)
                with patch.object(pipeline, 'Servo', FakeServo, create=True):
                    Runtime(cfg, root, None, cancel, lambda _message: None).jog_servo('up')
                self.assertEqual(events, [('open', 'config.yaml'), ('check',), ('pulse', expected), ('close',)])

    def test_known_surface_converts_pixels_to_body_meters(self):
        obs=Observation(1,0,(300,220),(344,344),1,None,Q)
        cfg={**CFG['snapshot_move'],'approach_extra_m':0.}
        p=make_plan(obs,CFG['camera'],cfg)
        np.testing.assert_allclose(p['body_xy_m'],[-.062,-.022],atol=1e-6)

    def test_three_cm_is_added_along_last_visual_direction(self):
        for center in ((300,220),(380,380),(300,380),(380,220)):
            obs=Observation(1,0,center,(344,344),1,None,Q)
            p=make_plan(obs,CFG['camera'],{**CFG['snapshot_move'],'approach_extra_m':.03})
            before=np.array(p['vision_body_xy_m']);after=np.array(p['body_xy_m'])
            self.assertAlmostEqual(np.linalg.norm(after)-np.linalg.norm(before),.03,places=6)
            np.testing.assert_allclose(after/np.linalg.norm(after),before/np.linalg.norm(before),atol=1e-6)
        p=make_plan(Observation(1,0,(344,344),(344,344),1,None,Q),CFG['camera'],CFG['snapshot_move'])
        np.testing.assert_allclose(p['body_xy_m'],[0,0])

    def test_encoder_imu_feedback_integration_and_timer_wrap(self):
        pose=FeedbackPose();pose.update(0,.1,0)
        for i in range(1,51):pose.update(i*20000,.1,0)
        np.testing.assert_allclose(pose.tuple(),[.1,0,0],atol=1e-6)
        pose=FeedbackPose();pose.update(2**32-10000,0,.2);pose.update(10000,0,.2)
        self.assertAlmostEqual(pose.yaw,.004)

    def test_locked_motion_reaches_all_quadrants_and_restores_yaw(self):
        for xy in ([.06,.02],[.06,-.02],[-.06,.02],[-.06,-.02],[0,.04],[0,-.04],[0,0]):
            cfg=CFG['snapshot_move'];p=np.zeros(3);m=LockedMove(xy,p,cfg)
            for _ in range(20000):
                v,w=m.step(p)
                self.assertLessEqual(abs(v),cfg['max_linear_m_s']+1e-9)
                self.assertLessEqual(abs(w),cfg['max_angular_rad_s']+1e-9)
                dt=.02;p[:2]+=v*dt*np.array([math.cos(p[2]+w*dt/2),math.sin(p[2]+w*dt/2)]);p[2]+=w*dt
                if m.stage=='DONE':break
            self.assertEqual(m.stage,'DONE',str(xy))
            self.assertLess(np.linalg.norm(p[:2]-xy),.012)
            self.assertLessEqual(abs(wrap(p[2])),cfg['yaw_tolerance_rad']+.001)

    def test_pipeline_reads_camera_once_then_moves_and_sucks(self):
        from slicer_visual_grasp import pipeline
        clock=[0.];commands=[];camera_calls=[];pump_commands=[];poses=[]
        class Camera:
            def snapshot(self):
                camera_calls.append(1)
                if len(camera_calls)>1:raise AssertionError('movement must not reread vision')
                return None,Observation(1,clock[0],(300,220),(344,344),1,None,Q),None,None
            def set_active(self,active):pass
        class Motion:
            def __init__(self,*a):self.pose=np.zeros(3);poses.append(self.pose)
            def wait_stationary(self):clock[0]+=.4;return tuple(self.pose)
            def feedback(self):return tuple(self.pose),0.,0.
            def tick(self,v=0,w=0,servo='neutral',immediate=False):
                dt=.02;self.pose[:2]+=v*dt*np.array([math.cos(self.pose[2]+w*dt/2),math.sin(self.pose[2]+w*dt/2)]);self.pose[2]+=w*dt
                commands.append((v,w,servo));clock[0]+=dt
            def close(self):commands.append((0,0,'neutral'))
        class Pump:
            def __init__(self,*a):self.on=False;self.since=0
            def start(self):pass
            def command(self,on):
                self.on=on;self.since=clock[0]
                pump_commands.append((on,tuple(poses[-1]),commands[-1]))
            def snapshot(self):
                attached=self.on and clock[0]-self.since>=.1
                return Pressure(clock[0],self.on,attached,'SUCTION_ATTACHED' if attached else 'SUCTION_FREE',self.on)
            def close(self):pass
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'pump.json').write_text('{}');cfg=copy.deepcopy(CFG);cfg['airgrip_config']='pump.json'
            with patch.object(pipeline,'Motion',Motion),patch.object(pipeline,'PressureLink',Pump),patch('slicer_visual_grasp.pipeline.time.monotonic',side_effect=lambda:clock[0]):
                Runtime(cfg,root,Camera(),threading.Event(),lambda s:None).pick()
            self.assertEqual(len(camera_calls),1)
            plan=json.loads((root/'data/last_move_plan.json').read_text())
            np.testing.assert_allclose(plan['vision_body_xy_m'],[-.062,-.022],atol=1e-6)
            self.assertAlmostEqual(np.linalg.norm(plan['body_xy_m']),math.hypot(.062,.022)+.03,places=6)
            self.assertTrue(any(v or w for v,w,s in commands))
            self.assertTrue(all(s=='neutral' for v,w,s in commands))
            self.assertEqual(len(pump_commands),1)
            self.assertTrue(pump_commands[0][0])
            self.assertEqual(pump_commands[0][2][:2],(0,0))
            self.assertLess(np.linalg.norm(np.array(pump_commands[0][1][:2])-plan['body_xy_m']),.012)

if __name__=='__main__':unittest.main()
