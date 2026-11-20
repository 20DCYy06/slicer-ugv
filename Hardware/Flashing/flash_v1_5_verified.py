#!/usr/bin/env python3
"""Install the v1.5 image on a C30D with a verified v1.4 sector 0."""
import argparse
import struct
import sys

import flash
from chassis import Chassis

VERSION = "v1.5"
BOARD = ("1a86", "55d4", "5CF0016377")
V14_SECTOR_SHA = "bd4138c6be3ddfe903535ef7d89869088271cb5fa26d2d37bc5cdfe7031643e1"
V15_IMAGE_SHA = "809b930773b0e7ac8c3c692aecc15ad52ae1f8ae8888d2677e9cf425a9ef2f51"


class VerifiedROM(flash.SectorZeroROM):
    def backup(self, directory, info, **kwargs):
        result = super().backup(directory, info, **kwargs)
        original = (directory / "flash_read_1.bin").read_bytes()
        candidate = read_candidate()
        target = candidate.ljust(flash.SECTOR, b"\xff")
        if flash.sha(original[:flash.SECTOR]) != V14_SECTOR_SHA and original[:flash.SECTOR] != target:
            raise RuntimeError("板上扇区 0 与已验证 v1.4 或本次 v1.5 不匹配，双备份已保存")
        return result


def read_candidate():
    path = flash.ROOT / "Hardware/Firmware" / VERSION / "src/build/c30d_body.bin"
    image = path.read_bytes()
    if flash.sha(image) != V15_IMAGE_SHA:
        raise RuntimeError("v1.5 镜像哈希与验收记录不匹配")
    sp, reset = struct.unpack_from("<II", image)
    if not 0x20000000 < sp <= 0x20020000 or not reset & 1 or not flash.BASE <= reset < flash.BASE + len(image):
        raise RuntimeError("应用向量表无效")
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--power-off-confirmed", action="store_true")
    if not parser.parse_args().power_off_confirmed:
        parser.error("需要确认 Motor OFF、物理断开 12V、仅 USB 供电")
    read_candidate()
    config = flash.load_config()
    _, expected = flash.device(config)
    if expected != BOARD:
        raise RuntimeError("当前底盘身份与替换板不匹配")
    car = Chassis(config)
    try:
        state = car.read_state()
        if state["motor_enabled"] or state["motor_power_present"] or any(state["pwm_ticks"]):
            raise RuntimeError("遥测未确认 Motor OFF、动力断开和四轮 PWM 为零")
    finally:
        car.close()
    print("PRE_FLASH_MOTOR_OFF_POWER_OFF_PWM_ZERO", flush=True)
    (flash.ROOT / "Hardware/Backups").mkdir(exist_ok=True)
    flash.SectorZeroROM = VerifiedROM
    sys.argv = [sys.argv[0], "--version", VERSION, "--flash", "--power-off-confirmed"]
    flash.main()


if __name__ == "__main__":
    main()
