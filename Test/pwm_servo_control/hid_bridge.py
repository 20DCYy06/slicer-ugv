"""仅负责Hiwonder控制板HID I/O；图形界面不以root运行。"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import sys
import time
from protocol import hid_report, voltage_frame, voltage_mv, move_frame, load_config, speed_pulse, angle_pulse

HERE = Path(__file__).resolve().parent


def boards():
    result = []
    for p in Path('/sys/class/hidraw').glob('*'):
        try:
            props = dict(line.split('=', 1) for line in (p / 'device/uevent').read_text().splitlines() if '=' in line)
            if props.get('HID_ID', '').upper() == '0003:00000483:00005750' and 'Hiwonder' in props.get('HID_NAME', ''):
                result.append('/dev/' + p.name)
        except OSError:
            pass
    return sorted(result)


class Board:
    def __init__(self):
        candidates = boards()
        if len(candidates) != 1:
            raise RuntimeError('需要唯一的Hiwonder 0483:5750控制板，当前：' + str(candidates))
        self.path = candidates[0]
        self.fd = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            os.close(self.fd)
            raise RuntimeError('控制板已被另一个本程序实例占用')
        # 仅打开特定HID时需要权限；随后立即放弃root，避免以root运行UI。
        if os.geteuid() == 0 and 'SUDO_UID' in os.environ:
            os.setgroups([])
            os.setgid(int(os.environ['SUDO_GID']))
            os.setuid(int(os.environ['SUDO_UID']))

    def send(self, frame):
        report = hid_report(frame)
        if os.write(self.fd, report) != len(report):
            raise OSError('HID发送不完整')
        return frame.hex(' ')

    def voltage(self):
        # 只读查询，不发送MOVE或动作组指令。
        self.send(voltage_frame())
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            if select.select([self.fd], [], [], max(0, end - time.monotonic()))[0]:
                value = voltage_mv(os.read(self.fd, 256))
                if value is not None:
                    return value
        return None

    def close(self):
        os.close(self.fd)


def emit(**event):
    print(json.dumps(event, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='仅查询电压，不运动')
    args = parser.parse_args()
    cfg = load_config(HERE / 'config.json')
    board = Board()
    active8 = False
    try:
        mv = board.voltage()
        emit(event='connected', device=board.path, voltage_mv=mv, uid=os.geteuid())
        if args.check:
            if mv is None:
                raise RuntimeError('HID已打开但电压查询未响应，请检查板供电和USB连接')
            return
        for line in sys.stdin:
            try:
                req = json.loads(line)
                op = req['op']
                if op == 'quit':
                    break
                if op == 'voltage':
                    emit(event='voltage', voltage_mv=board.voltage())
                    continue
                if op == 'stop8':
                    channel, pulse = 8, cfg['channel8']['neutral_us']
                elif op == 'speed8':
                    channel, pulse = 8, speed_pulse(float(req['value']), cfg['channel8'])
                elif op == 'angle9':
                    channel, pulse = 9, angle_pulse(float(req['value']), cfg['channel9'])
                else:
                    raise ValueError('未知操作')
                # 连续旋转松开即输出中位，不等待300ms插值减速。
                duration = 0 if op == 'stop8' else cfg['move_time_ms']
                frame = move_frame(channel, pulse, duration)
                wire = board.send(frame)
                if channel == 8:
                    active8 = pulse != cfg['channel8']['neutral_us']
                emit(event='sent', channel=channel, pulse_us=pulse, frame=wire)
            except Exception as exc:
                emit(event='error', message=str(exc))
    finally:
        # EOF/正常关闭时仅停止已由本程序启动的8路；9路不自动改变位置。
        if active8:
            try:
                board.send(move_frame(8, cfg['channel8']['neutral_us'], 0))
                emit(event='stopped8')
            except OSError as exc:
                emit(event='error', message='8路停止发送失败：' + str(exc))
        board.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        emit(event='error', message=str(exc))
        sys.exit(1)
