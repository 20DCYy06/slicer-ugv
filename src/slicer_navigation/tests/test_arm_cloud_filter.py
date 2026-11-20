#!/usr/bin/env python3
import math
from pathlib import Path
import struct
import sys
import unittest

from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from arm_cloud_filter import mask_arm_points


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
        output, removed = mask_arm_points(msg, (.081, .023, .136), (0, 0, 0, 1),
                                          -.10, .22, .09, .23)
        self.assertEqual(removed, 1)
        self.assertTrue(math.isnan(struct.unpack_from('<f', output.data)[0]))
        for index in (1, 2, 3):
            self.assertTrue(math.isfinite(struct.unpack_from('<f', output.data, index * 12)[0]))
        self.assertTrue(math.isfinite(struct.unpack_from('<f', msg.data)[0]))


if __name__ == '__main__':
    unittest.main()
