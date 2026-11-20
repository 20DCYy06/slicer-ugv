"""从上游host_ros.py逐函数复用；无ROS/串口副作用。"""

def crc16_modbus(data_bytes):
    crc = 0xFFFF
    for b in data_bytes:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc_text(body):
    return "%04X" % crc16_modbus(body.encode("utf-8"))


def encode_frame(seq, cmd, payload=""):
    body = "%s|%s|%s" % (seq, cmd, payload)
    return "@%s|%s\n" % (body, crc_text(body))


def parse_frame(line):
    text = line.strip()
    if not text or not text.startswith("@"):
        return False, "BAD_PREFIX"

    parts = text[1:].split("|")
    if len(parts) != 4:
        return False, "BAD_FIELDS"

    seq, cmd, payload, crc = parts
    body = "%s|%s|%s" % (seq, cmd, payload)
    if crc.upper() != crc_text(body):
        return False, "BAD_CRC"

    return True, {"seq": seq, "cmd": cmd.upper(), "payload": payload}


def parse_kv(payload):
    result = {}
    if not payload:
        return result

    for item in payload.split(";"):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            result[item.lower()] = ""
            continue
        k, v = item.split("=", 1)
        result[k.strip().lower()] = v.strip()
    return result
