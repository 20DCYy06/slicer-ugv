import copy,json,math,struct,tempfile,threading,unittest
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
from slicer_visual_grasp.paths import DEFAULT_CONFIG
from slicer_visual_grasp.logic import Observation,Pressure
from slicer_visual_grasp.snapshot_move import make_plan,LockedMove,FeedbackPose,wrap
from slicer_visual_grasp.pipeline import Runtime,Servo
ROOT=Path(__file__).resolve().parent
CFG=json.loads(DEFAULT_CONFIG.read_text())
Q=[[200,150],[400,150],[400,290],[200,290]]


class SnapshotTests(unittest.TestCase):
    def test_headless_pick_times_out_on_an_observation_before_the_request(self):
        clock = [10.]
        camera = Mock()
        camera.snapshot.return_value = (
            None, Observation(1, 9.9, (300, 220), (344, 344), 1, None, Q), None, None)
        cancel = Mock()
        cancel.is_set.return_value = False
        cancel.wait.side_effect = lambda duration: clock.__setitem__(0, clock[0] + duration)
        runtime = Runtime(CFG, ROOT, camera, cancel, lambda _: None)
        with patch('slicer_visual_grasp.pipeline.time.monotonic', side_effect=lambda: clock[0]), \
                patch('slicer_visual_grasp.pipeline.PressureLink') as pressure, \
                patch('slicer_visual_grasp.pipeline.Motion') as motion:
            with self.assertRaisesRegex(TimeoutError, '等待新相机目标超时'):
                runtime.pick(wait_for_target=True)
        self.assertAlmostEqual(clock[0], 15.)
        pressure.assert_not_called()
        motion.assert_not_called()

    def test_visual_servo_rejects_pb15_without_sending_a_pulse(self):
        servo = Servo.__new__(Servo)
        servo.moved = False
        servo.link = Mock()
        servo.send = Mock()
        servo.receive = Mock(return_value=[
            (0x86, 1, struct.pack('<4sBBHH', b'B15P', 1, 15, 1500, 500))
        ])
        runtime = Runtime(CFG, ROOT, None, threading.Event(), lambda _: None)
        with patch('slicer_visual_grasp.pipeline.Servo', return_value=servo):
            with self.assertRaisesRegex(RuntimeError, 'PB14.*PB15'):
                runtime.move_servo('up', .1)
        servo.send.assert_called_once_with(0x30)
        servo.link.close.assert_called_once()
        self.assertEqual(runtime.lowered_s, 0.)

    def test_release_uses_recorded_lowering_time_after_pump_off(self):
        runtime = Runtime(CFG, ROOT, None, threading.Event(), lambda _: None)
        runtime.lowered_s = .23
        with patch.object(runtime, 'pump_off', create=True) as pump, \
                patch.object(runtime, 'move_servo', create=True) as lift:
            pump.side_effect = lambda: self.assertFalse(lift.called)
            runtime.release()
        pump.assert_called_once()
        lift.assert_called_once_with('up', .23)

    def test_known_surface_converts_pixels_to_body_meters(self):
        obs=Observation(1,0,(300,220),(344,344),1,None,Q)
        cfg={**CFG['snapshot_move'],'approach_extra_m':0.}
        p=make_plan(obs,CFG['camera'],cfg)
        np.testing.assert_allclose(p['body_xy_m'],[-.062,-.022],atol=1e-6)

    def test_legacy_three_cm_setting_never_adds_motion(self):
        for center in ((300,220),(380,380),(300,380),(380,220),(344,344)):
            obs = Observation(1, 0, center, (344,344), 1, None, Q)
            plan = make_plan(obs, CFG['camera'], {**CFG['snapshot_move'], 'approach_extra_m': .03})
            np.testing.assert_allclose(plan['body_xy_m'],
                                       [(center[1]-344)*.0005, (center[0]-344)*.0005],
                                       atol=1e-6)

    def simulate_alignment(self, observe, expected_error=None):
        from slicer_visual_grasp import pipeline
        now = [1.]
        pose = [0., 0., 0.]
        logs = []
        samples = []
        pump = Mock()
        pump.snapshot.side_effect = lambda: Pressure(now[0])

        class Camera:
            def snapshot(self):
                seq = len(samples) + 1
                samples.append((now[0], pose[0]))
                return None, observe(seq, now[0], pose[0]), None, None

            def set_active(self, active):
                pass

        class Motion:
            def __init__(self, *args):
                pass

            def wait_stationary(self, deadline=None):
                now[0] += .3
                return tuple(pose)

            def feedback(self):
                return tuple(pose), 0., 0.

            def tick(self, v=0., w=0.):
                pose[0] += v * .02 * math.cos(pose[2])
                pose[1] += v * .02 * math.sin(pose[2])
                pose[2] += w * .02
                now[0] += .02

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as folder, patch.object(pipeline, 'Motion', Motion), \
                patch.object(pipeline.time, 'monotonic', side_effect=lambda: now[0]):
            runtime = Runtime(CFG, Path(folder), Camera(), threading.Event(), logs.append)
            outcome = self.assertRaisesRegex(RuntimeError, expected_error) if expected_error else nullcontext()
            with outcome:
                runtime.move_to_target(pump, 0.)
            plan_path = Path(folder) / 'data/last_move_plan.json'
            plan = json.loads(plan_path.read_text()) if plan_path.exists() else None
        return pose, samples, logs, plan

    def test_occlusion_finishes_only_the_latest_remaining_vector(self):
        def observe(seq, stamp, x):
            targets = {1: .09, 2: .13}
            target = targets.get(seq)
            center = None if target is None else (344, 344+(target-x)*2000)
            return Observation(seq, stamp, center, (344,344), 1, None, Q)

        pose, samples, logs, plan = self.simulate_alignment(observe)
        self.assertLess(samples[1][0]-samples[0][0], 1.2)
        self.assertAlmostEqual(pose[0], .13, delta=.009)
        self.assertEqual(plan['frame_seq'], 2)
        self.assertTrue(any('VISION_OCCLUDED' in line for line in logs))
        self.assertTrue(any('VISION_ALIGNED' in line for line in logs))

    def test_alternating_visual_bearing_allows_translation_then_stops_if_not_converging(self):
        def observe(seq, stamp, x):
            # Same longitudinal error and lateral jitter as seen in the field log.
            center = (344 + (36 if seq % 2 else -36), 264)
            return Observation(seq, stamp, center, (344,344), 1, None, Q)

        _, samples, logs, _ = self.simulate_alignment(observe, expected_error='连续.*未收敛')
        self.assertAlmostEqual(samples[1][0]-samples[0][0], 1., delta=.03)
        self.assertLess(min(x for _, x in samples), -.015, 'yaw corrections must leave time to drive')
        self.assertLess(samples[-1][0] - samples[0][0], 25.)
        self.assertFalse(any('VISION_ALIGNED' in line for line in logs))

    def test_nozzle_inside_surface_still_approaches_the_green_center(self):
        def observe(seq, stamp, x):
            center_y = 344 + (-.018 - x) * 2000
            q = [[244,center_y-70],[444,center_y-70],[444,center_y+70],[244,center_y+70]]
            return Observation(seq, stamp, (344,center_y), (344,344), 1, None, q)

        pose, samples, _, _ = self.simulate_alignment(observe)
        self.assertGreaterEqual(len(samples), 2)
        self.assertLess(pose[0], -.014)

    def test_in_tolerance_frame_then_occlusion_finishes_its_residual_before_grasp(self):
        def observe(seq, stamp, x):
            center = (344,338) if seq == 1 else None
            return Observation(seq, stamp, center, (344,344), 1, None, Q)

        pose, samples, logs, plan = self.simulate_alignment(observe)
        self.assertGreaterEqual(len(samples), 3)
        self.assertAlmostEqual(pose[0], -.003, delta=.001)
        self.assertEqual(plan['frame_seq'], 1)
        self.assertTrue(any('丢框剩余位移' in line for line in logs))

    def test_fresh_stationary_frame_outside_threshold_requires_a_correction(self):
        def observe(seq, stamp, x):
            target = 0. if seq == 1 else .014
            return Observation(seq, stamp, (344, 344+(target-x)*2000), (344,344), 1, None, Q)

        pose, samples, _, plan = self.simulate_alignment(observe)
        self.assertGreaterEqual(len(samples), 3)
        self.assertGreater(pose[0], .010)
        self.assertEqual(plan['frame_seq'], 2)

    def test_stale_frame_is_not_a_confirmed_occlusion(self):
        def observe(seq, stamp, x):
            return Observation(1, 1.3, (344,344) if seq == 1 else None, (344,344), 1, None, Q)

        with self.assertRaisesRegex(TimeoutError, '到位复核|新相机'):
            self.simulate_alignment(observe)

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

    def test_pipeline_moves_grasps_and_lifts_then_releases_remaining_travel(self):
        from slicer_visual_grasp import pipeline
        clock=[0.];commands=[];camera_calls=[];pump_commands=[];poses=[];servo_pulses=[];pump_closes=[]
        attach_after = [3.19]
        attach_until = [float('inf')]
        pump_start_error = [None]
        interrupt_lift = [False]
        release_device = Mock()
        class Camera:
            def snapshot(self):
                camera_calls.append(clock[0])
                x,y,yaw = poses[-1]
                dx,dy = -.062-x, -.022-y
                forward = math.cos(yaw)*dx+math.sin(yaw)*dy
                left = -math.sin(yaw)*dx+math.cos(yaw)*dy
                center = (344+2000*left,344+2000*forward)
                return None,Observation(len(camera_calls),clock[0],center,(344,344),1,None,Q),None,None
            def set_active(self,active):pass
        class Motion:
            def __init__(self,*a):self.pose=np.zeros(3);poses.append(self.pose)
            def wait_stationary(self, deadline=None):clock[0]+=.4;return tuple(self.pose)
            def feedback(self):return tuple(self.pose),0.,0.
            def tick(self,v=0,w=0,servo='neutral',immediate=False):
                dt=.02;self.pose[:2]+=v*dt*np.array([math.cos(self.pose[2]+w*dt/2),math.sin(self.pose[2]+w*dt/2)]);self.pose[2]+=w*dt
                commands.append((v,w,servo));clock[0]+=dt
            def close(self):commands.append((0,0,'neutral'))
        class ClockEvent:
            def is_set(self):
                return (interrupt_lift[0] and len(servo_pulses) >= 2
                        and all(value == 2100 for _, value in servo_pulses[-2:]))
            def wait(self,seconds):clock[0]+=max(seconds, 1e-9);return False
        class FakeServo:
            def __init__(self,*a):pass
            def check(self, *, required_pin):
                if required_pin != 14:
                    raise AssertionError('visual grasp requires PB14')
            def pulse(self,value):servo_pulses.append((clock[0], value))
            def receive(self):return []
            def close(self):servo_pulses.append((clock[0], 1500))
        class Pump:
            def __init__(self,*a):self.on=False;self.since=0
            def start(self):
                if pump_start_error[0] is not None:
                    raise pump_start_error[0]
            def command(self,on):
                self.on=on;self.since=clock[0]
                pump_commands.append((on,tuple(poses[-1]),commands[-1]))
            def snapshot(self):
                attached=self.on and attach_after[0] <= clock[0]-self.since < attach_until[0]
                return Pressure(clock[0],self.on,attached,'SUCTION_ATTACHED' if attached else 'SUCTION_FREE',self.on)
            def close(self, release=False):pump_closes.append(release)
        def assert_lift(pulses, duration):
            self.assertTrue(pulses[:-1])
            self.assertTrue(all(value == 2100 for _, value in pulses[:-1]))
            self.assertEqual(pulses[-1][1], 1500)
            self.assertAlmostEqual(pulses[-1][0] - pulses[0][0], duration)

        with tempfile.TemporaryDirectory() as folder, \
                patch.object(pipeline, 'Motion', Motion), patch.object(pipeline, 'PressureLink', Pump), \
                patch.object(pipeline, 'Servo', FakeServo), patch.object(pipeline.time, 'sleep'), \
                patch.object(pipeline.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch('airgrip.keyboard.AirGrip', return_value=release_device):
            root = Path(folder)
            (root/'pump.json').write_text('{"ack_timeout_s": 0.8}')
            cfg = copy.deepcopy(CFG)
            cfg['airgrip_config'] = 'pump.json'
            cfg['grasp']['max_lower_s'] = 20.
            pump_start_error[0] = TimeoutError('startup PING')
            runtime = Runtime(cfg, root, Camera(), ClockEvent(), lambda s: None)
            with self.assertRaisesRegex(TimeoutError, 'startup PING'):
                runtime.pick()
            self.assertFalse(commands, 'AirGrip startup failure must prevent chassis commands')
            self.assertEqual(pump_commands, [])
            self.assertEqual(servo_pulses, [])
            self.assertEqual(pump_closes, [False])
            pump_start_error[0] = None
            pump_closes.clear()
            runtime = Runtime(cfg, root, Camera(), ClockEvent(), lambda s: None)
            runtime.pick()
            first_neutral = next(i for i, (_, value) in enumerate(servo_pulses) if value == 1500)
            self.assertGreater(first_neutral, 0)
            self.assertTrue(all(value == 900 for _, value in servo_pulses[:first_neutral]))
            t = servo_pulses[first_neutral][0] - servo_pulses[0][0]
            self.assertGreater(t, 2.)
            assert_lift(servo_pulses[first_neutral+1:], 2.)
            self.assertAlmostEqual(runtime.lowered_s, t - 2.)
            servo_pulses.clear()
            release_device.set_pump.side_effect = lambda on: self.assertFalse(servo_pulses)
            runtime.release()
            release_device.set_pump.assert_called_once_with(False)
            assert_lift(servo_pulses, t - 2.)
            self.assertAlmostEqual(runtime.lowered_s, 0.)
            self.assertEqual(pump_closes,[False])
            self.assertGreaterEqual(len(camera_calls), 3)
            plan = json.loads((root/'data/last_move_plan.json').read_text())
            self.assertNotIn('approach_extra_m', plan)
            self.assertTrue(any(v or w for v,w,s in commands))
            self.assertTrue(all(s=='neutral' for v,w,s in commands))
            self.assertEqual(len(pump_commands),1)
            self.assertTrue(pump_commands[0][0])
            self.assertEqual(pump_commands[0][2][:2],(0,0))
            self.assertLess(np.linalg.norm(np.array(pump_commands[0][1][:2])-[-.062,-.022]),.012)
            attach_after[0] = .09
            camera_calls.clear()
            servo_pulses.clear()
            runtime = Runtime(cfg, root, Camera(), ClockEvent(), lambda s: None)
            runtime.pick()
            first_neutral = next(i for i, (_, value) in enumerate(servo_pulses) if value == 1500)
            assert_lift(servo_pulses[first_neutral+1:], 2.)
            self.assertEqual(runtime.lowered_s, 0.)
            with self.assertRaisesRegex(RuntimeError, '先点击释放'):
                runtime.pick()
            self.assertGreaterEqual(len(camera_calls), 3)
            servo_pulses.clear()
            release_device.set_pump.reset_mock()
            runtime.release()
            release_device.set_pump.assert_called_once_with(False)
            self.assertEqual(servo_pulses, [])

            attach_after[0] = 3.19
            camera_calls.clear()
            interrupt_lift[0] = True
            runtime = Runtime(cfg, root, Camera(), ClockEvent(), lambda s: None)
            with self.assertRaises(InterruptedError):
                runtime.pick()
            self.assertEqual(servo_pulses[-1][1], 1500)
            self.assertAlmostEqual(runtime.lowered_s, t - .1)
            interrupt_lift[0] = False
            servo_pulses.clear()
            runtime.release()
            assert_lift(servo_pulses, t - .1)

            cfg['grasp']['max_lower_s'] = .05
            camera_calls.clear()
            servo_pulses.clear()
            runtime = Runtime(cfg, root, Camera(), ClockEvent(), lambda s: None)
            with self.assertRaisesRegex(TimeoutError, '下降时限'):
                runtime.pick()
            self.assertAlmostEqual(runtime.lowered_s, .05)
            self.assertEqual(servo_pulses[-1][1], 1500)
            self.assertFalse(any(value == 2100 for _, value in servo_pulses))
            self.assertEqual(pump_closes,[False, False, False, True])

            cfg['grasp']['max_lower_s'] = 20.
            attach_until[0] = 4.
            camera_calls.clear()
            runtime = Runtime(cfg, root, Camera(), ClockEvent(), lambda s: None)
            with self.assertRaisesRegex(RuntimeError, '抬升结束时未确认仍然吸附'):
                runtime.pick()
            self.assertTrue(runtime.needs_release)

if __name__=='__main__':unittest.main()
