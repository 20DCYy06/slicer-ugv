"""视觉位移规划：通过已知 7×10 cm 平面换算底盘 XY，并由编码器与 IMU 跟踪。"""
import math
import cv2
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def surface_quad(contour):
    hull=cv2.convexHull(contour)
    points=None
    for ratio in (.015,.025,.04):
        candidate=cv2.approxPolyDP(hull,ratio*cv2.arcLength(hull,True),True)
        if len(candidate)==4:
            points=candidate.reshape(4,2).astype(np.float32);break
    if points is None:
        points=cv2.boxPoints(cv2.minAreaRect(hull))
    center=points.mean(axis=0)
    order=np.argsort(np.arctan2(points[:,1]-center[1],points[:,0]-center[0]))
    return points[order]


def make_plan(obs,camera,cfg):
    if obs.center is None or obs.nozzle_center is None or obs.surface_quad is None:
        raise ValueError('需要同一画面中的物体中心、固定吸嘴原点及物体四角')
    q=np.asarray(obs.surface_quad,np.float32)
    if q.shape!=(4,2) or not np.isfinite(q).all():raise ValueError('物体四角无效')
    edge0=(np.linalg.norm(q[1]-q[0])+np.linalg.norm(q[3]-q[2]))/2
    edge1=(np.linalg.norm(q[2]-q[1])+np.linalg.norm(q[0]-q[3]))/2
    # 当前可见面7×10cm：近俯视时较长的图像边对应10cm，另一边7cm。
    long,short=sorted(cfg['surface_size_m'],reverse=True)
    if not (math.isfinite(long) and math.isfinite(short) and short>0):raise ValueError('可见面尺寸必须为正数')
    u,v=(long,short) if edge0>=edge1 else (short,long)
    h=cv2.getPerspectiveTransform(q,np.float32([[0,0],[u,0],[u,v],[0,v]]))
    origin=np.asarray(obs.nozzle_center,float)
    directions={'down':np.array([0.,1.]),'up':np.array([0.,-1.]),
                'right':np.array([1.,0.]),'left':np.array([-1.,0.])}
    if cfg['forward_image_direction'] not in directions:raise ValueError('请设置底盘正线速度的画面方向')
    forward=directions[cfg['forward_image_direction']]
    left=np.array([forward[1],-forward[0]])
    pixel=np.float32([[origin,obs.center,origin+forward,origin+left]])
    mapped=cv2.perspectiveTransform(pixel,h)[0]
    delta=mapped[1]-mapped[0]
    f=mapped[2]-mapped[0];f=f/np.linalg.norm(f)
    l=np.array([-f[1],f[0]])
    if np.dot(l,mapped[3]-mapped[0])<0:l=-l
    xy=[float(np.dot(delta,f)),float(np.dot(delta,l))]
    if not np.isfinite(xy).all():raise ValueError('平面位移换算无效')
    return dict(mode='periodic_visual_feedback',frame_seq=obs.seq,
                red_px=list(obs.nozzle_center),green_px=list(obs.center),
                pixel_vector=(np.asarray(obs.center)-origin).tolist(),
                surface_quad_px=q.tolist(),surface_size_m=[long,short],
                forward_image_direction=cfg['forward_image_direction'],body_xy_m=xy,
                camera_device=camera['device'])


class FeedbackPose:
    """积分下位机反馈的编码器线速度和IMU角速度；不用命令速度估计进度。"""
    def __init__(self):
        self.x=self.y=self.yaw=0.;self.previous=None

    def update(self,mcu_us,v,w):
        if self.previous is not None:
            last,pv,pw=self.previous
            dt=((mcu_us-last)&0xffffffff)/1e6
            if dt>1:raise RuntimeError('底盘反馈时间戳中断/复位，停止本次移动')
            speed=(pv+v)/2;turn=(pw+w)/2
            middle=self.yaw+turn*dt/2
            self.x+=speed*dt*math.cos(middle);self.y+=speed*dt*math.sin(middle)
            self.yaw+=turn*dt
        self.previous=(mcu_us,v,w)

    def tuple(self):return self.x,self.y,self.yaw


class LockedMove:
    """目标只在构造时锁定。转向、沿目标线移动，再恢复初始朝向。

    恢复朝向可抵消未知的底盘中心→吸嘴水平杆臂，不需要猜测吸嘴安装距离。
    """
    def __init__(self,xy,pose,cfg):
        self.cfg=cfg;self.start=np.array(pose[:2],dtype=float,copy=True);self.yaw0=float(pose[2])
        x,y=xy;self.distance=math.hypot(x,y);angle=math.atan2(y,x)
        if abs(angle)>math.pi/2:
            angle=wrap(angle+math.pi);self.distance=-self.distance
        self.heading=self.yaw0+angle
        self.axis=np.array([math.cos(self.heading),math.sin(self.heading)])
        self.stage='DONE' if abs(self.distance)<=cfg['position_tolerance_m'] else 'TURN_OUT'

    def turn(self,error):
        value=max(-self.cfg['max_angular_rad_s'],min(self.cfg['max_angular_rad_s'],self.cfg['heading_gain']*error))
        return math.copysign(max(abs(value),self.cfg['min_turn_rad_s']),value)

    def step(self,pose):
        if self.stage=='DONE':return 0.,0.
        if self.stage in ('TURN_OUT','TURN_BACK'):
            target=self.heading if self.stage=='TURN_OUT' else self.yaw0
            error=wrap(target-pose[2])
            if abs(error)<=self.cfg['yaw_tolerance_rad']:
                self.stage='DRIVE' if self.stage=='TURN_OUT' else 'DONE'
                return 0.,0.
            return 0.,self.turn(error)
        progress=float(np.dot(np.asarray(pose[:2])-self.start,self.axis))
        remaining=self.distance-progress
        if abs(remaining)<=self.cfg['position_tolerance_m']:
            self.stage='TURN_BACK';return 0.,0.
        speed=min(self.cfg['max_linear_m_s'],max(self.cfg['min_linear_m_s'],abs(remaining)))
        yaw_error=wrap(self.heading-pose[2])
        w=max(-self.cfg['max_angular_rad_s'],min(self.cfg['max_angular_rad_s'],self.cfg['heading_gain']*yaw_error))
        return math.copysign(speed,remaining),w
