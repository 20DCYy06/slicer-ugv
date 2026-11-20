"""只发送电压查询，比较HID中断输出与控制传输；不含MOVE命令。"""
import ctypes as c
import ctypes.util
import os
import select
import time
from hid_bridge import boards
from protocol import hid_report,voltage_frame,voltage_mv

paths=boards()
assert len(paths)==1,paths
fd=os.open(paths[0],os.O_RDWR|os.O_NONBLOCK)
lib=c.CDLL(ctypes.util.find_library('usb-1.0'))
lib.libusb_init.argtypes=[c.POINTER(c.c_void_p)]
lib.libusb_open_device_with_vid_pid.argtypes=[c.c_void_p,c.c_uint16,c.c_uint16]
lib.libusb_open_device_with_vid_pid.restype=c.c_void_p
lib.libusb_control_transfer.argtypes=[c.c_void_p,c.c_uint8,c.c_uint8,c.c_uint16,c.c_uint16,c.c_void_p,c.c_uint16,c.c_uint]
lib.libusb_close.argtypes=[c.c_void_p]
lib.libusb_exit.argtypes=[c.c_void_p]
ctx=c.c_void_p();assert lib.libusb_init(c.byref(ctx))==0
handle=lib.libusb_open_device_with_vid_pid(ctx,0x0483,0x5750)
try:
    try:print('INTERRUPT_WRITE_BYTES',os.write(fd,hid_report(voltage_frame())),flush=True)
    except OSError as e:print('INTERRUPT_ERROR',repr(e),flush=True)
    end=time.monotonic()+1
    while time.monotonic()<end:
        if select.select([fd],[],[],.1)[0]:
            data=os.read(fd,256);print('INTERRUPT_REPLY',data.hex(),voltage_mv(data),flush=True)
    if handle:
        data=c.create_string_buffer(voltage_frame().ljust(64,b'\0'))
        n=lib.libusb_control_transfer(handle,0x21,9,0x200,0,data,64,1000)
        print('CONTROL_SET_REPORT_RESULT',n,flush=True)
        end=time.monotonic()+1
        while time.monotonic()<end:
            if select.select([fd],[],[],.1)[0]:
                data=os.read(fd,256);print('CONTROL_REPLY',data.hex(),voltage_mv(data),flush=True)
    else:print('CONTROL_OPEN_FAILED',flush=True)
finally:
    if handle:lib.libusb_close(handle)
    lib.libusb_exit(ctx);os.close(fd)
