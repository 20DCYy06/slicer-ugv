"""Read-only occupancy map for the desktop RViz display."""
from copy import copy
import math
import os
from pathlib import Path
from threading import Lock

import numpy as np
from PyQt5.QtGui import QImage
import rospy
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
import yaml


DISPLAY_TOPIC = 'rtk_display_map'
COSTMAP_TOPIC = '/ugv/move_base/global_costmap/costmap'
Grid = numpy_msg(OccupancyGrid)
# Costmap2DPublisher uses 0 for free, 100 for lethal, -1 for unknown,
# and 1–99 for derived costs, including the inscribed footprint cost.
COSTMAP_TO_OCCUPANCY = bytearray([255] * 256)
COSTMAP_TO_OCCUPANCY[0] = 0
COSTMAP_TO_OCCUPANCY[100] = 100


def costmap_occupancy(costmap):
    samples = np.asarray(costmap.data, dtype=np.int8)
    cells = samples.tobytes().translate(COSTMAP_TO_OCCUPANCY)
    return Grid(header=copy(costmap.header), info=costmap.info,
                data=np.frombuffer(cells, dtype=np.int8))


def load_saved_map(path, frame):
    """Read a trinary grayscale ROS map YAML and its image."""
    path = Path(path).expanduser()
    metadata = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict) or metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('地图 YAML 需要使用 trinary 模式。')
    x, y, yaw = map(float, metadata['origin'])
    resolution = float(metadata['resolution'])
    occupied = float(metadata['occupied_thresh'])
    free = float(metadata['free_thresh'])
    negate = float(metadata['negate'])
    if (not frame or not all(map(math.isfinite, (x, y, yaw, resolution)))
            or resolution <= 0 or not 0 <= free < occupied <= 1 or negate not in (0, 1)):
        raise ValueError('地图坐标系、原点、分辨率或占据参数无效。')

    image_path = path.parent / metadata['image']
    image = QImage(str(image_path))
    if image.isNull() or not image.isGrayscale() or image.hasAlphaChannel():
        raise ValueError(f'无法读取不带透明度的灰度地图图像 {image_path}。')
    image = image.convertToFormat(QImage.Format_Grayscale8)
    width, height, stride = image.width(), image.height(), image.bytesPerLine()
    rows = np.frombuffer(image.constBits().asstring(height * stride), dtype=np.uint8)
    # Image rows start at the top. OccupancyGrid starts at the bottom-left.
    gray = rows.reshape(height, stride)[::-1, :width]
    probability = (gray if negate else 255 - gray) / 255.0
    cells = np.full((height, width), -1, dtype=np.int8)
    cells[probability > occupied] = 100
    cells[probability < free] = 0

    grid = Grid(data=cells.ravel())
    grid.header.frame_id = frame
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = x
    grid.info.origin.position.y = y
    grid.info.origin.orientation.z = math.sin(yaw / 2.0)
    grid.info.origin.orientation.w = math.cos(yaw / 2.0)
    return grid


class MapDisplay:
    """Prefer an explicit saved map, then /map, then observed RTK costmap cells."""

    def __init__(self):
        self.publisher = rospy.Publisher(DISPLAY_TOPIC, Grid, queue_size=1, latch=True)
        saved_map = os.environ.get('SLICER_RTK_MAP_YAML')
        if saved_map:
            frame = os.environ.get('SLICER_RTK_MAP_FRAME', 'map').strip().lstrip('/')
            try:
                grid = load_saved_map(saved_map, frame)
            except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
                rospy.logwarn('RViz 无法加载地图 %s，继续使用 ROS 地图输入。%s', saved_map, error)
            else:
                grid.header.stamp = grid.info.map_load_time = rospy.Time.now()
                self.publisher.publish(grid)
                rospy.loginfo('RViz 已加载地图 %s（%s）', saved_map, frame)
                return

        self._lock = Lock()
        self._has_live_map = False
        # The fixed 40 m field profile produces 640 kB full grids.
        self._map = rospy.Subscriber('/map', Grid, self._live_map, queue_size=1, buff_size=2**20)
        self._costmap = rospy.Subscriber(COSTMAP_TOPIC, Grid, self._live_costmap,
                                         queue_size=1, buff_size=2**20)

    def _live_map(self, grid):
        if not _complete_grid(grid):
            return
        with self._lock:
            self._has_live_map = True
            self.publisher.publish(Grid(header=copy(grid.header), info=grid.info,
                                        data=np.asarray(grid.data, dtype=np.int8)))

    def _live_costmap(self, grid):
        if not _complete_grid(grid):
            return
        with self._lock:
            if self._has_live_map and self._map.get_num_connections():
                return
            self._has_live_map = False
            self.publisher.publish(costmap_occupancy(grid))


def _complete_grid(grid):
    return (grid.info.width > 0 and grid.info.height > 0
            and math.isfinite(grid.info.resolution) and grid.info.resolution > 0
            and len(grid.data) == grid.info.width * grid.info.height)
