"""Bounded candidate search in a live, axis-aligned RTK odom costmap."""
import math


def candidate_goals(start, distances):
    x, y, yaw = start
    for distance in distances:
        for degrees in (0, 35, -35, 70, -70, 120, -120, 180):
            bearing = yaw + math.radians(degrees)
            yield x + distance * math.cos(bearing), y + distance * math.sin(bearing), bearing


def observed_cell(grid, x, y):
    info = grid.info
    resolution = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    if not all(math.isfinite(v) for v in (x, y, ox, oy, resolution)) or resolution <= 0.:
        return False
    if len(grid.data) != info.width * info.height:
        return False
    col, row = math.floor((x - ox) / resolution), math.floor((y - oy) / resolution)
    # 99 is inscribed under costmap_2d's OccupancyGrid translation.
    return (0 <= col < info.width and 0 <= row < info.height
            and 0 <= grid.data[row * info.width + col] < 99)


def observed_path(grid, points, budget):
    """Return path length only if it stays in known cells away from lethal costs."""
    if len(points) < 2:
        return None
    resolution = grid.info.resolution
    if not observed_cell(grid, *points[0]):
        return None
    length = 0.
    for (x, y), (nx, ny) in zip(points, points[1:]):
        segment = math.hypot(nx - x, ny - y)
        length += segment
        if not math.isfinite(length) or length > budget:
            return None
        divisions = max(1, math.ceil(segment / (resolution / 3.)))
        for step in range(divisions + 1):
            if not observed_cell(grid, x + (nx - x) * step / divisions,
                                 y + (ny - y) * step / divisions):
                return None
    return length if length > 0. else None
