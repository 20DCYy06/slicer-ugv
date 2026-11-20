import json,tempfile,time,unittest
from pathlib import Path
import cv2,numpy as np
from two_point import NozzleDetector
from vision import PinkDetector
from logic import Observation
from nozzle_calibration import capture_sample
ROOT=Path(__file__).resolve().parent
CFG=json.loads((ROOT/'config.json').read_text())

class FixedOriginTests(unittest.TestCase):
    def test_origin_is_constant_even_without_visible_nozzle(self):
        detector=NozzleDetector(CFG['target']['nozzle'])
        self.assertEqual(detector.templates,[])
        for fill in (0,255):
            point,_,_=detector.detect(np.full((480,640,3),fill,np.uint8))
            self.assertEqual(list(point),CFG['target']['nozzle']['origin_px'])

    def test_new_camera_pink_block_is_detected_on_recorded_frame(self):
        image=cv2.imread(str(ROOT/'data/analysis/fixed_origin_current.jpg'))
        center,box,_=PinkDetector(CFG['target']).detect(image)
        self.assertIsNotNone(center)
        self.assertLess(np.linalg.norm(np.array(center)-[329,225]),12)

    def test_gray_floor_does_not_match_pink(self):
        self.assertIsNone(PinkDetector(CFG['target']).detect(np.full((480,640,3),150,np.uint8))[0])

    def test_failed_detection_still_saves_raw_photo_without_stale_green_point(self):
        with tempfile.TemporaryDirectory() as folder:
            now=time.monotonic();obs=Observation(2,now,None,(344,344),1)
            old=Observation(1,now-.1,(100,100))
            record,path=capture_sample(Path(folder),'face1',np.zeros((480,640,3),np.uint8),obs,[old],CFG['camera'])
            self.assertIsNone(record['object_center_px'])
            self.assertTrue((path/'image.jpg').exists())
            self.assertFalse((Path(folder)/'data/aim.json').exists())

    def test_saved_green_point_is_relative_to_red_origin(self):
        with tempfile.TemporaryDirectory() as folder:
            now=time.monotonic();obs=Observation(1,now,(330,225),(344,344),1)
            record,_=capture_sample(Path(folder),'face1',np.zeros((480,640,3),np.uint8),obs,[],CFG['camera'])
            self.assertEqual(record['object_relative_px'],[-14,-119])

if __name__=='__main__':unittest.main()
