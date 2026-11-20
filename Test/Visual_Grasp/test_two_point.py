import json,time,copy,unittest
from pathlib import Path
import cv2,numpy as np
from logic import Observation,Pressure,GraspLogic
from two_point import NozzleDetector,surface_center,relative_feature

ROOT=Path(__file__).resolve().parent
CFG=json.loads((ROOT/'config.json').read_text())
CAL=dict(aim_px=[312,166],jacobian=[[300,800],[-3400,-500]])

class TwoPointTests(unittest.TestCase):
    def test_dynamic_nozzle_replaces_cached_fixed_pixel(self):
        f=GraspLogic(CFG,CAL,0)
        obs=Observation(1,.1,(350,220),(320,180),.9)
        o=f.step(.1,obs,Pressure(.1))
        np.testing.assert_allclose(f.aim,[320,180])
        self.assertTrue(o.v or o.w)
        # 两个点共同平移，误差和控制都应完全一致。
        o2=f.step(.2,Observation(2,.2,(360,225),(330,185),.9),Pressure(.2))
        np.testing.assert_allclose([o.v,o.w],[o2.v,o2.w])

    def test_missing_nozzle_stops_without_lowering(self):
        f=GraspLogic(CFG,CAL,0)
        o=f.step(.1,Observation(1,.1,(350,220)),Pressure(.1))
        self.assertEqual(f.state,'ERROR');self.assertTrue(o.immediate_stop);self.assertEqual(o.servo,'neutral')

    def test_both_points_coincident_stable_frames_then_pump_stage(self):
        f=GraspLogic(CFG,CAL,0)
        for i in range(4):f.step(i*.1,Observation(i,i*.1,(315,165),(312,166),.9),Pressure(i*.1))
        self.assertEqual(f.state,'STOP_BASE')
        o=f.step(.5,Observation(5,.5,(315,165),(312,166),.9),Pressure(.5))
        self.assertTrue(o.pump_on);self.assertEqual(f.state,'PUMP_START');self.assertEqual(o.servo,'neutral')

    def test_rectangle_surface_center_ignores_internal_occlusion(self):
        mask=np.zeros((300,400),np.uint8)
        cv2.rectangle(mask,(100,80),(300,220),255,-1)
        cv2.rectangle(mask,(170,80),(230,140),0,-1)
        cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        np.testing.assert_allclose(surface_center(cs[0]),[200,150],atol=1)

    def test_relative_error_uses_both_coordinates(self):
        np.testing.assert_allclose(relative_feature(Observation(1,0,(320,240),(300,180))),[20,60])

    def test_blank_image_does_not_match_nozzle(self):
        detector=NozzleDetector(dict(mode='template',template_dir='data/nozzle_templates',search_roi=[220,100,190,130],min_score=.65))
        self.assertTrue(detector.templates)
        center,box,score=detector.detect(np.zeros((480,640,3),np.uint8))
        self.assertIsNone(center)

if __name__=='__main__':unittest.main()
