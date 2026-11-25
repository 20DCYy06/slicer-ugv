#!/usr/bin/env python3
"""Filter self returns for the projected scan and optional lidar voxel map.

The original LIO cloud is untouched. The scan keeps the existing arm mask
before its separate chassis filter. The voxel observation also rejects
chassis returns in 3-D before they can mark or clear cells.
"""
import copy
import math

import numpy as np
import rospy
import tf
import yaml
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import quaternion_matrix


def filter_cloud(msg, translation, rotation, rear_x, front_x, half_width, min_z,
                 *, chassis_half_size=None):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields and fields[name].datatype == PointField.FLOAT32
               and fields[name].count == 1 for name in ('x', 'y', 'z')):
        raise ValueError('Cloud requires scalar FLOAT32 x/y/z fields')
    if msg.point_step < max(fields[name].offset + 4 for name in ('x', 'y', 'z')):
        raise ValueError('Invalid PointCloud2 field offsets')
    data = bytearray(msg.data)
    dtype = '>f4' if msg.is_bigendian else '<f4'
    shape = (msg.height, msg.width)
    strides = (msg.row_step, msg.point_step)
    xyz = [np.ndarray(shape, dtype=dtype, buffer=data, offset=fields[name].offset,
                      strides=strides) for name in ('x', 'y', 'z')]
    matrix = quaternion_matrix(rotation)
    # Transform from source cloud frame into base_link. The IMU/body frame is
    # not at the ground-projected chassis centre, so raw x/y/z cannot be masked.
    with np.errstate(invalid='ignore'):
        bx = matrix[0, 0]*xyz[0] + matrix[0, 1]*xyz[1] + matrix[0, 2]*xyz[2] + translation[0]
        by = matrix[1, 0]*xyz[0] + matrix[1, 1]*xyz[1] + matrix[1, 2]*xyz[2] + translation[1]
        bz = matrix[2, 0]*xyz[0] + matrix[2, 1]*xyz[1] + matrix[2, 2]*xyz[2] + translation[2]
        finite = np.isfinite(bx) & np.isfinite(by) & np.isfinite(bz)
        mask = (finite & (bx >= rear_x) & (bx <= front_x) &
                (np.abs(by) <= half_width) & (bz >= min_z))
        if chassis_half_size is not None:
            half_length, half_chassis_width = chassis_half_size
            chassis = (np.abs(bx) <= half_length) & (np.abs(by) <= half_chassis_width)
    voxel = None
    if chassis_half_size is not None:
        keep = finite & ~mask & ~chassis
        if np.any(keep):
            records = np.ndarray(shape, dtype=f'V{msg.point_step}', buffer=data, strides=strides)
            voxel = copy.copy(msg)
            voxel.height = 1
            voxel.width = int(np.count_nonzero(keep))
            voxel.row_step = voxel.width * msg.point_step
            voxel.data = records[keep].tobytes()
            voxel.is_dense = True
    for coord in xyz:
        coord[mask] = math.nan
    result = copy.copy(msg)
    result.data = bytes(data)
    result.is_dense = False
    return result, int(np.count_nonzero(mask)), voxel


def main():
    rospy.init_node('slicer_arm_cloud_filter')
    with open(rospy.get_param('~config_file'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    envelope = config['navigation']['travel_envelope']
    min_z = float(config['sensors']['arm_self_filter_min_z'])
    rear_x, front_x = float(envelope['rear_x']), float(envelope['front_x'])
    half_width = float(envelope['half_width'])
    if not (-1 < rear_x < 0 < front_x < 1 and 0 < half_width < 1 and 0 < min_z < 2):
        raise ValueError('Invalid arm self-filter envelope')
    listener = tf.TransformListener()
    pub = rospy.Publisher('/cloud_nav_filtered', PointCloud2, queue_size=1)
    voxel_pub, chassis_size = None, None
    if rospy.get_param('~publish_voxel', False):
        chassis = config['chassis']
        padding = float(config['sensors']['self_filter_padding'])
        chassis_size = tuple(float(chassis[key]) / 2 + padding for key in ('body_length', 'body_width'))
        if not (0 <= padding < .10 and all(0 < extent < 1 for extent in chassis_size)):
            raise ValueError('Invalid voxel chassis self-filter envelope')
        voxel_pub = rospy.Publisher('/cloud_nav_voxel', PointCloud2, queue_size=1)

    def callback(msg):
        try:
            translation, rotation = listener.lookupTransform(config['frames']['base'],
                                                              msg.header.frame_id, rospy.Time(0))
            filtered, count, voxel = filter_cloud(
                msg, translation, rotation, rear_x, front_x, half_width, min_z,
                chassis_half_size=chassis_size)
        except (tf.Exception, ValueError) as exc:
            # Never pass unfiltered cloud into navigation if TF/format is lost.
            rospy.logerr_throttle(5, 'Arm cloud filter unavailable: %s', exc)
            return
        rospy.loginfo_throttle(30, 'Arm self-filter removed %d high points from latest cloud', count)
        pub.publish(filtered)
        if voxel is not None:
            voxel_pub.publish(voxel)

    rospy.Subscriber('/cloud_registered_body', PointCloud2, callback, queue_size=1,
                     tcp_nodelay=True)
    rospy.spin()


if __name__ == '__main__':
    main()
