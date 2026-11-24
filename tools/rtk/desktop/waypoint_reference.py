"""A separate local-to-WGS84 transform for read-only desktop waypoints."""
from geographic_core import enu_between, enu_to_geodetic, fit_alignment, transform_xy


MIN_BASELINE_M = 2.
MAX_RESIDUAL_M = .10


class WaypointReferences:
    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None
        self.origin = None
        self.lever = None
        self.alignment = None

    def capture(self, odom, antenna_origin, lever):
        if self.alignment is not None:
            raise ValueError('clear the existing references before recording again')
        if self.anchor is None:
            self.anchor = dict(stamp=odom['stamp'], odom=odom, enu=(0., 0., 0.))
            self.origin = antenna_origin
            self.lever = tuple(lever)
            return 1
        if tuple(lever) != self.lever:
            raise ValueError('antenna offset changed between captures')
        station = dict(stamp=odom['stamp'], odom=odom,
                       enu=enu_between((0., 0., 0.), antenna_origin, self.origin))
        self.alignment = fit_alignment([self.anchor, station], self.lever,
                                       MIN_BASELINE_M, MAX_RESIDUAL_M)
        return 2

    def convert(self, x, y):
        if self.alignment is None:
            return None
        east, north = transform_xy(x, y, self.alignment['yaw'], *self.alignment['translation'])
        latitude, longitude, _ = enu_to_geodetic((east, north, 0.), self.origin)
        return dict(latitude=latitude, longitude=longitude)
