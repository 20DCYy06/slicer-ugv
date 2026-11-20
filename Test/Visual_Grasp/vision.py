"""USB相机+粉色长方体轮廓中心。无需神经网络、深度流或手眼矩阵。"""
import threading
import logging
import time
import cv2
import numpy as np
from logic import Observation
from two_point import NozzleDetector, surface_center
from snapshot_move import surface_quad


class PinkDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.previous = None
        self.lost = False
        self.selection = None
        self.retain_loss = True
        self.quad = None

    def select(self, box):
        self.selection = box
        self.previous = None
        self.lost = False

    def detect(self, image):
        self.quad=None
        if self.lost and not self.retain_loss:
            self.previous = None
            self.lost = False
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower, upper = list(self.cfg['hsv_lower']), list(self.cfg['hsv_upper'])
        if lower[0] <= upper[0]:
            mask = cv2.inRange(hsv, np.asarray(lower,np.uint8), np.asarray(upper,np.uint8))
        else:
            # 粉色在USB自动白平衡下可能跨越红色H=0，支持160..179及0..35。
            mask = cv2.inRange(hsv,np.asarray(lower,np.uint8),np.asarray([179,*upper[1:]],np.uint8))
            mask |= cv2.inRange(hsv,np.asarray([0,*lower[1:]],np.uint8),np.asarray(upper,np.uint8))
        kernel = np.ones((3,3),np.uint8)
        if self.cfg.get('color_space')=='lab':
            # 新相机的淡粉色接近白色，饱和度低；a通道分离偏红粉色与灰绿地面。
            lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
            mask=cv2.inRange(lab,np.asarray(self.cfg['lab_lower'],np.uint8),np.asarray(self.cfg['lab_upper'],np.uint8))
        closing = np.ones((self.cfg.get('close_kernel_px',7),)*2,np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,closing)
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.cfg['min_area_px']:
                continue
            rect = cv2.minAreaRect(contour)
            _,(w,h),_ = rect
            if w*h < 1 or area/(w*h) < self.cfg['min_rect_fill']:
                continue
            # 用可见粉色面的面积质心，而不是角度敏感的旋转外接框中心。
            # 现场浅粉色边缘有噪声：此法无需叠加时间滤波即可稳定中心。
            moment = cv2.moments(contour)
            u,v = moment['m10']/moment['m00'], moment['m01']/moment['m00']
            if self.cfg.get('surface_center',False):u,v=surface_center(contour)
            if self.selection is not None:
                x,y,bw,bh = self.selection
                if not (x <= u <= x+bw and y <= v <= y+bh):
                    continue
            candidates.append((area, (u,v), cv2.boundingRect(contour),surface_quad(contour)))
        chosen = None
        if not self.lost and candidates:
            if self.previous is None:
                chosen = max(candidates,key=lambda c:c[0])
            else:
                chosen = min(candidates,key=lambda c:np.linalg.norm(np.asarray(c[1])-self.previous))
                if np.linalg.norm(np.asarray(chosen[1])-self.previous) > self.cfg['association_px']:
                    chosen = None
        self.selection = None
        if chosen is None:
            # 丢失已锁定目标后不自动换成另一块粉色物体；需重新拖框确认。
            if self.previous is not None:
                self.lost = True
            return None, None, mask
        self.previous = np.asarray(chosen[1])
        self.quad=chosen[3].tolist()
        return chosen[1], chosen[2], mask


class Camera:
    def __init__(self, config, target):
        self.cfg = config
        self.detector = PinkDetector(target)
        self.nozzle=NozzleDetector(target['nozzle']) if target.get('nozzle') else None
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.frame = None
        self.observation = Observation(0, 0, None)
        self.box = None
        self.roi_request = None
        self.active = False
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True, name='usb-camera')

    def start(self):
        self.thread.start()

    def select(self, box):
        with self.lock:
            self.roi_request = tuple(int(x) for x in box)

    def snapshot(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy(),
                    self.observation, self.box, self.error)

    def set_active(self, active):
        with self.lock:
            self.active = active

    def run(self):
        cv2.setNumThreads(1)
        while not self.quit.is_set():
            try:
                self.capture_once()
                return
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                    self.box = None
                    self.active = False
                    self.observation = Observation(self.observation.seq,time.monotonic(),None)
                logging.getLogger('visual_grasp').warning('相机连接失败：%s；只重连预览，不恢复旧动作',exc)
                if isinstance(exc,ValueError):
                    return
                if self.quit.wait(self.cfg.get('retry_delay_s',1.0)):
                    return

    def capture_once(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.cfg['device'], cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg.get('fourcc','YUYV')))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg['height'])
            cap.set(cv2.CAP_PROP_FPS, self.cfg.get('capture_fps',30))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                raise RuntimeError('无法打开相机；检查设备路径及是否被其它程序占用')
            rate=cap.get(cv2.CAP_PROP_FPS)
            code=int(cap.get(cv2.CAP_PROP_FOURCC))
            encoding=''.join(chr((code >> (8*i)) & 255) for i in range(4))
            logging.getLogger('visual_grasp').info('相机已打开：%s %.2ffps；识别上限 %.1fHz',encoding,rate,self.cfg['fps'])
            self.detector=PinkDetector(self.detector.cfg)
            seq = self.observation.seq
            last_processed = 0.0
            while not self.quit.is_set():
                # 持续取走驱动缓冲；不靠sleep限频，避免相机实际30Hz时堆积旧画面。
                ok = cap.grab()
                stamp = time.monotonic()
                if not ok:
                    if self.quit.is_set():
                        return
                    raise RuntimeError('相机取帧失败')
                if stamp-last_processed < 1/self.cfg['fps']:
                    continue
                ok, image = cap.retrieve()
                if not ok:
                    raise RuntimeError('相机解码失败')
                last_processed = stamp
                if image.shape[1::-1] != (self.cfg['width'], self.cfg['height']):
                    raise ValueError('相机实际分辨率与配置不一致')
                seq += 1
                with self.lock:
                    roi, self.roi_request = self.roi_request, None
                    self.detector.retain_loss = self.active
                if roi is not None:
                    self.detector.select(roi)
                center, box, _ = self.detector.detect(image)
                nozzle,nozzle_box,score=None,None,0.0
                if self.nozzle is not None:nozzle,nozzle_box,score=self.nozzle.detect(image)
                with self.lock:
                    self.frame, self.box = image, box
                    self.observation = Observation(seq, stamp, center,nozzle,score,nozzle_box,self.detector.quad)
                    self.error = None
        finally:
            if cap is not None:
                cap.release()

    def close(self, wait=True):
        self.quit.set()
        if wait and self.thread.ident is not None:
            self.thread.join(2)
