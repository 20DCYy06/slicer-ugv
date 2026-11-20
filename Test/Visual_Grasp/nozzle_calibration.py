"""记录操作者确认的可吸取位置。只保存图像/像素数据，不导入任何执行器模块。"""
from pathlib import Path
import json
import time
import cv2
import numpy as np


def atomic_json(path, value):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    temp.replace(path)


def capture_sample(root, face_id, frame, observation, history, camera, max_age_s=.6):
    """只记录可吸取摆放，不把可见轮廓质心认定为吸嘴真实接触点。

    历史帧只用于减少静态测量噪声；不是由此求解相机内参、手眼或运动雅可比。
    不同面分别存储，禁止将不同高度的目标像素盲目平均。
    """
    now=time.monotonic()
    if face_id not in ('face1','face2','face3'):
        raise ValueError('请选择面1、面2或面3')
    if frame is None or now-observation.stamp>max_age_s:
        raise ValueError('没有新鲜相机画面，不能拍摄')
    seen={observation.seq:observation} if observation.center is not None else {}
    for item in history if observation.center is not None else []:
        if item.center is not None and 0<=observation.stamp-item.stamp<=.8:
            seen[item.seq]=item
    points=np.asarray([item.center for item in seen.values()],float).reshape(-1,2)
    aim=np.median(points,axis=0) if len(points) else None
    origin=getattr(observation,'nozzle_center',None)
    center=np.asarray((frame.shape[1]/2,frame.shape[0]/2))
    folder=Path(root)/'data/nozzle_samples'/face_id/str(time.time_ns())
    folder.mkdir(parents=True,exist_ok=False)
    if not cv2.imwrite(str(folder/'image.jpg'),frame):
        raise IOError('原图保存失败')
    marked=frame.copy()
    if aim is not None:cv2.drawMarker(marked,tuple(np.rint(aim).astype(int)),(0,255,0),cv2.MARKER_CROSS,24,2)
    if origin is not None:cv2.drawMarker(marked,tuple(np.rint(origin).astype(int)),(0,0,255),cv2.MARKER_CROSS,24,2)
    if not cv2.imwrite(str(folder/'marked.png'),marked):
        raise IOError('标记图保存失败')
    record=dict(face_id=face_id,confirmed_pickable_by_user=True,
                sample_kind='reachable_placement',nozzle_target_calibrated=False,
                recorded_unix_s=time.time(),camera=dict(camera),
                resolution=[frame.shape[1],frame.shape[0]],aim_px=None if aim is None else aim.tolist(),
                object_center_px=None if aim is None else aim.tolist(),origin_px=origin,
                object_relative_px=None if aim is None or origin is None else (aim-origin).tolist(),
                image_geometric_center_px=center.tolist(),
                offset_from_image_center_px=None if aim is None else (aim-center).tolist(),
                sample_count=len(points),center_std_px=None if not len(points) else points.std(axis=0).tolist(),
                centers_px=points.tolist(),image='image.jpg',marked='marked.png',
                actuator_commands_sent=False)
    atomic_json(folder/'sample.json',record)
    index=Path(root)/'data/nozzle_profiles.json'
    profiles=json.loads(index.read_text()) if index.exists() else {}
    profiles[face_id]=dict(object_center_px=record['object_center_px'],camera=dict(camera),
                           nozzle_target_calibrated=False,
                           sample=str((folder/'sample.json').relative_to(root)))
    atomic_json(index,profiles)
    return record,folder
