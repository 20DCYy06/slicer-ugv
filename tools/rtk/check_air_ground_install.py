#!/usr/bin/env python3
"""Check this interface from install only; never create ROS nodes or devices."""
import argparse
import importlib
import io
import os
from pathlib import Path
import runpy
import sys
import roslaunch
from check_noetic_install import package_path, require


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    prefix = parser.parse_args().prefix.resolve()
    for pkg in ('ugv_navigation', 'ugv_bringup', 'slicer_air_ground_msgs'):
        require(package_path(pkg) == prefix/'share'/pkg, 'package did not resolve from install: '+pkg)
    lib = prefix/'lib/ugv_navigation'
    sys.path.insert(0, str(lib))
    for name in ('air_ground_core', 'air_ground_executor', 'air_ground_wire',
                 'geographic', 'geographic_goal_core'):
        module = importlib.import_module(name)
        require(Path(module.__file__).resolve() == lib/(name+'.py'), 'wrong installed helper: '+name)
    for name in ('air_ground.py', 'air_ground_node.py', 'air_ground_rospy.py'):
        require(os.access(lib/name, os.X_OK), 'entry point not executable: '+name)
        runpy.run_path(str(lib/name), run_name='install_check_only')
    from slicer_air_ground_msgs.msg import Command
    require(prefix in Path(importlib.import_module('slicer_air_ground_msgs.msg').__file__).resolve().parents,
            'generated messages must come from install')
    msg = Command(protocol_version=1, server_id='INSTALL_CHECK_ONLY', sequence=1,
                  request_id='check', task_id='check', operation='PAUSE')
    buf = io.BytesIO(); msg.serialize(buf)
    require(Command().deserialize(buf.getvalue()) == msg, 'message round trip failed')
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(prefix/'share/ugv_bringup/launch/slicer_air_ground.launch'), config)
    require(not config.config_errors, 'launch parse errors')
    require([(n.package, n.type) for n in config.nodes] == [('ugv_navigation', 'air_ground_node.py')],
            'task launch must not start navigation, drivers or localization')
    require(set(config.params) == {'/air_ground_task/'+k for k in
                                  ('via_pass_radius_m','segment_timeout_s','cancel_timeout_s','action')},
            'task launch unexpectedly changes unrelated parameters')
    print('AIR_GROUND_INSTALL_OK command_md5='+Command._md5sum)


if __name__ == '__main__':
    main()
