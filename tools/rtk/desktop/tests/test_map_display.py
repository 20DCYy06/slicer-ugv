"""Check display-only costmap conversion and aligned ROS map image loading."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

try:
    from nav_msgs.msg import OccupancyGrid
except ImportError as error:
    raise unittest.SkipTest('Source the Noetic overlay to test OccupancyGrid conversion') from error

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from map_display import costmap_occupancy, load_saved_map


class MapDisplayTests(unittest.TestCase):
    def test_native_costmap_keeps_lethal_and_free_but_cannot_resolve_inflation(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.info.width = 6
        grid.info.height = 1
        grid.info.resolution = 0.05
        grid.info.origin.position.x = -20
        grid.data = [-1, 0, 1, 98, 99, 100]

        occupancy = costmap_occupancy(grid)

        self.assertEqual(list(occupancy.data), [-1, 0, -1, -1, -1, 100])
        self.assertEqual(occupancy.header.frame_id, 'odom')
        self.assertEqual(occupancy.info.origin.position.x, -20)
        self.assertEqual(grid.data, [-1, 0, 1, 98, 99, 100])

    def test_saved_map_loads_relative_image_with_ros_origin_and_row_order(self):
        with TemporaryDirectory() as directory:
            base = Path(directory)
            (base / 'field.pgm').write_bytes(b'P5\n3 2\n255\n' + bytes([0, 255, 205, 205, 0, 255]))
            config = base / 'field.yaml'
            config.write_text('image: field.pgm\nresolution: 0.05\n'
                              'origin: [-3, 4, 1.5707963267948966]\nnegate: 0\n'
                              'occupied_thresh: 0.65\nfree_thresh: 0.196\n',
                              encoding='utf-8')
            grid = load_saved_map(config, 'map')

        self.assertEqual(grid.header.frame_id, 'map')
        self.assertEqual((grid.info.width, grid.info.height), (3, 2))
        self.assertAlmostEqual(grid.info.resolution, 0.05)
        self.assertEqual((grid.info.origin.position.x, grid.info.origin.position.y), (-3, 4))
        self.assertAlmostEqual(grid.info.origin.orientation.z, 0.7071067811865476)
        self.assertAlmostEqual(grid.info.origin.orientation.w, 0.7071067811865476)
        self.assertEqual(list(grid.data), [-1, 100, 0, 100, 0, -1])

    def test_saved_map_negate_and_unknown_threshold_boundaries(self):
        with TemporaryDirectory() as directory:
            base = Path(directory)
            (base / 'field.pgm').write_bytes(b'P5\n4 1\n255\n' + bytes([0, 51, 204, 255]))
            config = base / 'field.yaml'
            config.write_text('image: field.pgm\nresolution: 1\n'
                              'origin: [0, 0, 0]\nnegate: 1\n'
                              'occupied_thresh: 0.8\nfree_thresh: 0.2\n',
                              encoding='utf-8')
            grid = load_saved_map(config, 'odom')

        self.assertEqual(grid.header.frame_id, 'odom')
        self.assertEqual(list(grid.data), [0, -1, -1, 100])

    def test_saved_map_rejects_missing_image(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / 'field.yaml'
            config.write_text('image: missing.pgm\nresolution: 1\n'
                              'origin: [0, 0, 0]\nnegate: 0\n'
                              'occupied_thresh: 0.65\nfree_thresh: 0.196\n',
                              encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'missing.pgm'):
                load_saved_map(config, 'map')


if __name__ == '__main__':
    unittest.main()
