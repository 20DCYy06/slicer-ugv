"""Bounded candidate search in a live, axis-aligned RTK odom costmap."""
import math


def candidate_goals(start, distances):
    x, y, yaw = start
    for distance in distances:
        for degrees in (0, 35, -35, 70, -70, 120, -120, 180):
            bearing = yaw + math.radians(degrees)
            yield x + distance * math.cos(bearing), y + distance * math.sin(bearing), bearing


def observed_cell(grid, x, y, max_cost=98):
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
            and 0 <= grid.data[row * info.width + col] <= max_cost)


def observed_path(grid, points, budget, max_cost=98):
    """Return path length only if it stays in known cells away from lethal costs."""
    if len(points) < 2:
        return None
    resolution = grid.info.resolution
    if not observed_cell(grid, *points[0], max_cost=max_cost):
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
                                 y + (ny - y) * step / divisions, max_cost=max_cost):
                return None
    return length if length > 0. else None


def straight_roundtrip_goal(start, requested, minimum, plan, known):
    """Select an observed forward corridor that the straight return can reuse."""
    if not all(math.isfinite(value) for value in (*start, requested, minimum)) or not 0. < minimum <= requested:
        return None
    x, y, yaw = start
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    # At most one attempt per quarter metre, plus the configured minimum.
    for step in range(math.ceil((requested - minimum) / .25) + 1):
        distance = max(requested - step * .25, minimum)
        goal = (x + distance * cos_yaw, y + distance * sin_yaw, yaw)
        # The return controller follows the start-to-goal line in reverse.
        if not known([start[:2], goal[:2]]):
            continue
        path = plan(goal)
        if len(path) < 2 or math.dist(path[-1], goal[:2]) > .15:
            continue
        points = [start[:2], *path, goal[:2]]
        along_max = 0.
        for px, py in points:
            dx, dy = px - x, py - y
            along = dx * cos_yaw + dy * sin_yaw
            across = -dx * sin_yaw + dy * cos_yaw
            # Leave room within the guarded return's .28 m lateral limit.
            if not math.isfinite(along) or not math.isfinite(across):
                break
            if abs(across) > .15 or along < along_max - .025 or along > distance + .025:
                break
            along_max = max(along, along)
        else:
            if known(points) and known([start[:2], goal[:2]]):
                return goal
    return None
