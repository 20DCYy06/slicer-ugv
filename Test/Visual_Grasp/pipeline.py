"""可中断的动作线程入口；UI线程只负责显示，绝不等待串口ACK。"""
import json
from pathlib import Path
import time
import cv2
from devices import Motion, PressureLink


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temp.replace(path)


class Runtime:
    def __init__(self, cfg, root, camera, cancel, log):
        self.cfg, self.root, self.camera = cfg, root, camera
        self.cancel, self.log = cancel, log

    def observation(self):
        frame, obs, _, error = self.camera.snapshot()
        self.last_observation_frame=frame
        if error:
            raise RuntimeError(error)
        if obs.center is None or time.monotonic()-obs.stamp > self.cfg['alignment']['observation_max_age_s']:
            raise RuntimeError('目标丢失或相机画面过期；停止本次操作，请重新选框')
        if self.cfg['alignment'].get('two_point',False) and obs.nozzle_center is None:
            raise RuntimeError('吸嘴模板未匹配；检查红色吸嘴框或模板区域配置')
        return obs



    def pick(self, calibration=None):
        """仅静止时读取一次视觉位移，运动中不再次读取视觉目标。"""
        from snapshot_move import make_plan,LockedMove
        cfg=self.cfg['snapshot_move']
        if cfg['forward_image_direction'] not in ('up','down','left','right'):
            raise ValueError('请在config.json设置底盘正线速度的画面方向')
        motion=pump=None
        try:
            pcfg=json.loads((self.root/self.cfg['airgrip_config']).read_text())
            pcfg['status_poll_s']=self.cfg['grasp']['status_poll_s']
            pcfg['open_settle_s']=self.cfg['grasp']['open_settle_s']
            pump=PressureLink(pcfg,self.log);pump.start()
            motion=Motion(self.cfg,self.root,self.cancel,self.log)
            self.log('[STATE] WAIT_STATIONARY：下发零速并等待实际反馈停稳')
            pose0=motion.wait_stationary()
            self.camera.set_active(True)
            observation=self.observation()  # 唯一用于规划的图像观测
            plan=make_plan(observation,self.cfg['camera'],cfg)
            plan['start_feedback_pose']=list(pose0)
            save_json(self.root/'data/last_move_plan.json',plan)
            if self.last_observation_frame is not None:
                cv2.imwrite(str(self.root/'data/last_move_snapshot.jpg'),self.last_observation_frame)
            self.log('[SNAPSHOT_LOCKED] pixel=%s vision_xy=%s extra=%.3fm body_xy=(%.4f,%.4f)m；运动中不重算视觉目标'%
                     (plan['pixel_vector'],plan['vision_body_xy_m'],plan['approach_extra_m'],*plan['body_xy_m']))
            mover=LockedMove(plan['body_xy_m'],pose0,cfg)
            previous=None;last_log=0.
            while mover.stage!='DONE':
                pose,v_measured,w_measured=motion.feedback()
                v,w=mover.step(pose)
                if previous!=mover.stage:
                    self.log('[STATE] '+mover.stage);previous=mover.stage
                motion.tick(v,w)
                if time.monotonic()-last_log>.5:
                    self.log('[MOVE_FEEDBACK] xy=(%.4f,%.4f)m yaw=%.3f command=(%.3f,%.3f) measured=(%.3f,%.3f)'%
                             (*pose,v,w,v_measured,w_measured));last_log=time.monotonic()
            self.log('[STATE] STOP_BEFORE_GRASP')
            motion.wait_stationary()
            self.log('[STATE] PUMP_START：位移执行结束，开始吸取')
            pump.command(True)
            # 不控制升降，也不等待不存在的升降到位。确认本次开泵指令后即结束。
            while True:
                motion.tick(0.,0.)
                pressure=pump.snapshot()
                if time.monotonic()-pressure.stamp>self.cfg['grasp']['status_max_age_s']:
                    raise RuntimeError('气泵反馈过期，未确认开泵')
                if pressure.on_ack and pressure.pump:
                    self.log('[DONE] 位移完成，气泵已确认开启；吸附=%s，无自动升降'%pressure.attached)
                    return
        finally:
            try:
                if motion:motion.close()
            finally:
                if pump:pump.close()
                self.camera.set_active(False)


    def release(self):
        cfg = json.loads((self.root/self.cfg['airgrip_config']).read_text())
        # 释放是明确的人机操作，不打开底盘，也不发送升降命令。
        from airgrip.keyboard import AirGrip
        dev = AirGrip(cfg)
        try:
            if self.cancel.wait(self.cfg['grasp']['open_settle_s']):
                raise InterruptedError('用户中断')
            self.log('[RELEASE] '+str(dev.set_pump(False)))
        finally:
            dev.ser.close()
