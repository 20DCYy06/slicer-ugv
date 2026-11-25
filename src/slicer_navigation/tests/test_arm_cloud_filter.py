#!/usr/bin/env python3
import math
from pathlib import Path
import struct
import sys
import unittest

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from arm_cloud_filter import filter_cloud


class ArmCloudFilterTests(unittest.TestCase):
    def test_only_high_points_inside_measured_arm_envelope_are_removed(self):
        points = [(0.10, 0.0, 0.10), (0.10, 0.0, 0.05),
                  (0.30, 0.0, 0.10), (0.10, 0.20, 0.10)]
        msg = PointCloud2()
        msg.height, msg.width = 1, len(points)
        msg.point_step, msg.row_step = 12, 12 * len(points)
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32,
                                 count=1) for i, name in enumerate(('x', 'y', 'z'))]
        msg.data = b''.join(struct.pack('<fff', *point) for point in points)
        output, removed, voxel = filter_cloud(msg, (.081, .023, .136), (0, 0, 0, 1),
                                          -.10, .22, .09, .23)
        self.assertEqual(removed, 1)
        self.assertIsNone(voxel)
        self.assertTrue(math.isnan(struct.unpack_from('<f', output.data)[0]))
        for index in (1, 2, 3):
            self.assertTrue(math.isfinite(struct.unpack_from('<f', output.data, index * 12)[0]))
        self.assertTrue(math.isfinite(struct.unpack_from('<f', msg.data)[0]))


    def test_voxel_copy_filters_self_and_nonfinite_but_keeps_ground_and_normal_scan(self):
        points = [(0.16, 0.0, 0.26), (0.10, 0.08, 0.15),
                  (0.22, 0.0, 0.07), (0.8, 0.0, 0.0), (math.nan, 0., 0.)]
        xyz = point_cloud2.create_cloud_xyz32(Header(frame_id='body'), points)
        scan, removed, voxel = filter_cloud(xyz, (0., 0., 0.), (0., 0., 0., 1.),
                                           -.10, .17, .11, .23, chassis_half_size=(.125, .113))
        self.assertEqual(removed, 1)
        self.assertEqual(scan.width, xyz.width)
        self.assertTrue(math.isfinite(struct.unpack_from('<f', scan.data, scan.point_step)[0]))
        self.assertEqual(voxel.header, xyz.header)
        self.assertEqual(voxel.width, 2)
        for measured, original in zip(point_cloud2.read_points(voxel), points[2:4]):
            for value, expected in zip(measured, original):
                self.assertAlmostEqual(value, expected)


if __name__ == '__main__':
    unittest.main()
