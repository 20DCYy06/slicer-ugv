"""LSC-16 协议：55 55、长度、命令、参数；无校验字节。"""
import struct


def move_frame(channel, pulse_us, duration_ms):
    # 面板接口是 S0..S15，丝印8/9直接对应协议ID 8/9，不减1。
    if channel not in (8, 9):
        raise ValueError('此程序仅控制接口8和9')
    if not 500 <= pulse_us <= 2500:
        raise ValueError('PWM必须在500..2500微秒内')
    if not 0 <= duration_ms <= 30000:
        raise ValueError('运动时间必须在0..30000毫秒内')
    return b'\x55\x55\x08\x03\x01' + struct.pack('<HBH', duration_ms, channel, pulse_us)


def voltage_frame():
    return b'\x55\x55\x02\x0f'


def hid_report(frame):
    # 本机实测：无Report ID的64字节HID输出，hidraw写入前须加0号Report ID。
    if len(frame) > 64:
        raise ValueError('HID有效数据超过64字节')
    return b'\x00' + frame.ljust(64, b'\x00')


def voltage_mv(report):
    # HID输入没有上面的0号Report ID；只解析Length指定的数据，忽略尾部缓冲残留。
    i = report.find(b'\x55\x55\x04\x0f')
    return struct.unpack_from('<H', report, i + 4)[0] if i >= 0 and len(report) >= i + 6 else None


def speed_pulse(speed, cfg):
    if not -100 <= speed <= 100:
        raise ValueError('速度比例必须在-100..100')
    speed *= cfg.get('direction', 1)
    n = cfg['neutral_us']
    end = cfg['positive_us'] if speed >= 0 else cfg['negative_us']
    return round(n + abs(speed) / 100.0 * (end - n))


def angle_pulse(angle, cfg):
    a, b = cfg['angle_min_deg'], cfg['angle_max_deg']
    if not a <= angle <= b or b <= a:
        raise ValueError('角度超出配置范围')
    ratio = (angle - a) / (b - a)
    if cfg.get('direction', 1) == -1:
        ratio = 1 - ratio
    return round(cfg['min_us'] + ratio * (cfg['max_us'] - cfg['min_us']))


def load_config(path):
    import json
    with open(path, encoding='utf-8') as f:
        cfg = json.load(f)
    s, p = cfg['channel8'], cfg['channel9']
    for v in (s['negative_us'], s['neutral_us'], s['positive_us'], p['min_us'], p['max_us']):
        if not isinstance(v, int) or not 500 <= v <= 2500:
            raise ValueError('脉宽配置应为500..2500整数微秒')
    if not s['negative_us'] < s['neutral_us'] < s['positive_us']:
        raise ValueError('8路需要negative_us < neutral_us < positive_us')
    if p['min_us'] >= p['max_us'] or p['angle_min_deg'] >= p['angle_max_deg']:
        raise ValueError('9路最小值必须小于最大值')
    for ch in (s, p):
        if ch.get('direction', 1) not in (-1, 1):
            raise ValueError('direction只能为1或-1')
    move_frame(9, p['min_us'], cfg['move_time_ms'])
    return cfg
