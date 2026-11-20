"""同帧双点：吸嘴模板定位、平面矩形中心、两点误差。无执行器调用。"""
from pathlib import Path
import cv2
import numpy as np


class NozzleDetector:
    def __init__(self,cfg,config_dir=None):
        self.cfg=cfg
        self.templates=[]
        if cfg.get('mode')=='fixed':return
        folder=(config_dir or Path.cwd()) / cfg.get('template_dir','data/nozzle_templates')
        for p in sorted(folder.glob('*.png')):
            image=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
            if image is not None and image.std()>2:self.templates.append(image)

    def detect(self,image):
        if self.cfg.get('mode')=='fixed':
            u,v=self.cfg['origin_px']
            if not (0<=u<image.shape[1] and 0<=v<image.shape[0]):return None,None,0.0
            return (float(u),float(v)),(int(u)-12,int(v)-12,24,24),1.0
        x,y,w,h=self.cfg['search_roi']
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        crop=gray[y:y+h,x:x+w]
        best=None
        for template in self.templates:
            th,tw=template.shape
            if crop.shape[0]<th or crop.shape[1]<tw:continue
            scores=cv2.matchTemplate(crop,template,cv2.TM_CCOEFF_NORMED)
            _,score,_,location=cv2.minMaxLoc(scores)
            if best is None or score>best[0]:best=(score,location,tw,th)
        if best is None or best[0]<self.cfg['min_score']:return None,None,0.0
        score,(u,v),tw,th=best
        offset=self.cfg.get('center_offset_px',[0,0])
        return (x+u+tw/2+offset[0],y+v+th/2+offset[1]),(x+u,y+v,tw,th),score


def surface_center(contour):
    # 吸嘴遮挡会挖掉粉色轮廓的一部分；凸包补全外形，不以缺口质心做目标。
    hull=cv2.convexHull(contour)
    for ratio in (.015,.025,.04):
        polygon=cv2.approxPolyDP(hull,ratio*cv2.arcLength(hull,True),True)
        if len(polygon)==4:
            corners=polygon.reshape(4,2).astype(np.float32)
            middle=corners.mean(axis=0)
            corners=corners[np.argsort(np.arctan2(corners[:,1]-middle[1],corners[:,0]-middle[0]))]
            transform=cv2.getPerspectiveTransform(np.float32([[0,0],[1,0],[1,1],[0,1]]),corners)
            center=cv2.perspectiveTransform(np.float32([[[.5,.5]]]),transform)[0,0]
            return tuple(float(v) for v in center)
    moment=cv2.moments(hull)
    return moment['m10']/moment['m00'],moment['m01']/moment['m00']


def relative_feature(observation):
    if observation.center is None or observation.nozzle_center is None:
        raise ValueError('物体或吸嘴未识别，不能计算双点偏移')
    return np.asarray(observation.center,float)-np.asarray(observation.nozzle_center,float)
