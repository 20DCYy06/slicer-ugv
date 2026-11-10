#!/usr/bin/env python3
"""在启动传感器栈前检查可执行文件及动态依赖，避免只打开空白 RViz。"""
import subprocess
import roslib.packages


def check_sensor_binaries():
    for package,node in [('livox_ros_driver2','livox_ros_driver2_node'),('fast_lio','fastlio_mapping')]:
        paths=roslib.packages.find_node(package,node)
        if not paths:
            raise RuntimeError('Missing %s/%s; run run_navigation.sh build (includes sensor packages)' % (package,node))
        result=subprocess.run(['ldd',paths[0]],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        if result.returncode or 'not found' in result.stdout:
            raise RuntimeError('Dependencies of %s are unavailable:\n%s' % (paths[0],result.stdout))
        print('SENSOR_EXECUTABLE_OK: '+paths[0],flush=True)


if __name__=='__main__':
    check_sensor_binaries()
