"""新底盘ROS适配核心；复用Test使用的Chassis，不另写通信/PWM控制器。"""
import math,secrets,struct,time

class Session:
    def __init__(self,car,c,wait_for_fault_clear=True):
        self.car,self.c,self.wait_fault=car,c,wait_for_fault_clear
        self.target=(0.,0.,-math.inf);self.held=False;self.retry_at=0.;self.recovery_error='';self.fault_flags=0
    def command(self,v,w,now):
        self.target=(v,w,now) if math.isfinite(v) and math.isfinite(w) else (0.,0.,-math.inf)
    def cycle(self,now):
        self.car.poll(0.)
        if now-self.car.last_rx>self.c['telemetry_timeout_s']:
            raise RuntimeError('MCU telemetry timeout; stop and close session')
        state=self.car.state;first=False;recovered=False
        if state['fault_latched']:
            if not self.wait_fault:raise RuntimeError('MCU fault latched')
            first=not self.held;self.held=True;self.target=(0.,0.,-math.inf)
            if first:self.fault_flags=state['flags']
            # HELLO仅在Motor OFF时清故障，不绕过下位机保护。
            if not state['motor_enabled'] and now>=self.retry_at:
                self.retry_at=now+1.
                self.car.speed(0.,0.)
                self.car.replies.clear();self.car.body=None;self.car.nonce=secrets.randbits(32)
                try:
                    self.car.connect();self.recovery_error=''
                except (RuntimeError,TimeoutError) as exc:self.recovery_error=str(exc)
                state=self.car.state
        if self.held:
            self.target=(0.,0.,-math.inf)
            if not state['fault_latched']:self.held=False;recovered=True
        v,w,stamp=self.target
        if self.held or now-stamp>=self.c['command_timeout_s']:v,w=0.,0.
        # 只保留原驱动的失联停车与对称速度上限；不添加转向补偿/二次包络。
        scale=max(1.,abs(v)/self.c['max_linear'],abs(w)/self.c['max_angular'])
        v,w=v/scale,w/scale
        self.car.speed(v,w)
        imu=self.car.body is not None and self.car.body['imu_flags']&7==7
        ready=bool(imu and state['configured'] and state['motor_enabled'] and state['motor_power_present'] and not state['fault_latched'])
        return v,w,ready,first,recovered

class EncoderTap:
    """旁路读取同一解析器已验证CRC的0x90；不打开第二个串口。"""
    def __init__(self,parser):self.parser=parser;self.sample=None
    def feed(self,data):
        frames=self.parser.feed(data)
        for kind,seq,payload in frames:
            if kind==0x90 and len(payload)==61:
                values=struct.unpack('<II4iBH4i4h4hH',payload)
                self.sample=(values[0],list(values[8:12]))
        return frames

class Odometry:
    """编码器累计平移+板载IMU偏航率；只发布/wheel/odom，EKF独占动态TF。"""
    def __init__(self,c):self.c=c;self.previous=None;self.x=self.y=self.yaw=0.
    def update(self,sample,body):
        if sample is None or sample==self.previous:return None
        previous=self.previous;self.previous=sample
        if previous is None:return None
        dt=((sample[0]-previous[0])&0xffffffff)*1e-6
        if not 0<dt<=1.:return None
        counts=[(((b-a+2**31)%2**32)-2**31)*s for a,b,s in zip(previous[1],sample[1],self.c['encoder_signs'])]
        distances=[n*math.pi*self.c['wheel_diameter']/self.c['counts_per_revolution'] for n in counts]
        ds=sum(distances)/4
        # 0x93中的wz是Test已验证的下位机IMU反馈；不使用命令角速度当反馈。
        wz=body['wz'] if body and body['imu_flags']&7==7 else ((distances[0]+distances[2]-distances[1]-distances[3])/2/(self.c['track_width']+self.c['wheelbase'])/dt)
        da=wz*dt;k=1. if abs(da)<1e-9 else math.sin(da/2)/(da/2)
        self.x+=ds*k*math.cos(self.yaw+da/2);self.y+=ds*k*math.sin(self.yaw+da/2)
        self.yaw=math.atan2(math.sin(self.yaw+da),math.cos(self.yaw+da))
        return self.x,self.y,self.yaw,ds/dt,wz
