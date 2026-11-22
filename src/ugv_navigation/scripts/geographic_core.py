#!/usr/bin/env python3
"""Pure geometry for fixed WGS84 ENU navigation; no ROS or persistent state.

Yaw is counterclockwise radians from +x. An odom record describes base_link;
GNSS ENU describes the antenna. The horizontal lever is expressed in base_link.
Registration therefore rotates the lever in odom before fitting antenna points.
"""
import math
from statistics import median
from sys import float_info

from cup_gnss.enu import geodetic_to_ecef, WGS84_A, WGS84_E2


def _number(value, name):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(name + ' must be a finite number')
    return float(value)


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError('{} must contain {} numbers'.format(name, size))
    return [_number(item, name) for item in value]


def _stamp(value):
    value = _number(value, 'stamp')
    if value < 0.:
        raise ValueError('stamp must be nonnegative')
    return value


def _odom(record):
    if not isinstance(record, dict):
        raise ValueError('odom must be a record with stamp, x, y and yaw')
    try:
        return dict(stamp=_stamp(record['stamp']), x=_number(record['x'], 'odom x'),
                    y=_number(record['y'], 'odom y'), yaw=wrap(record['yaw']))
    except KeyError as error:
        raise ValueError('odom requires stamp, x, y and yaw') from error


def wrap(angle):
    """Normalize a finite angle to [-pi, pi)."""
    return (_number(angle, 'angle') + math.pi) % (2. * math.pi) - math.pi


def transform_xy(x, y, yaw, tx, ty):
    """Return R(yaw) * [x,y] + [tx,ty] in metres."""
    x, y = _number(x, 'x'), _number(y, 'y')
    yaw = wrap(yaw)
    tx, ty = _number(tx, 'translation x'), _number(ty, 'translation y')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return _vector([cosine * x - sine * y + tx, sine * x + cosine * y + ty],
                   2, 'transformed point')


def antenna_odom(odom_record, antenna_xy):
    """Return the antenna's horizontal position in odom, applying its lever once."""
    record = _odom(odom_record)
    x, y = _vector(antenna_xy, 2, 'antenna lever')
    return transform_xy(x, y, record['yaw'], record['x'], record['y'])


def base_from_antenna(enu, base_yaw, antenna_xy):
    """Return base horizontal ENU from a raw antenna fix and base ENU yaw."""
    east, north, _ = _vector(enu, 3, 'antenna ENU')
    lever_x, lever_y = _vector(antenna_xy, 2, 'antenna lever')
    rotated = transform_xy(lever_x, lever_y, base_yaw, 0., 0.)
    return _vector([east - rotated[0], north - rotated[1]], 2, 'base ENU')


def interpolate_odom(odoms, stamp, max_gap=.5):
    """Interpolate ordered base poses without extrapolation or crossing a gap.

    Exact samples remain usable without interpolation, including a single
    sample. Duplicate or decreasing stamps are rejected rather than reordered.
    Yaw follows the shortest arc. These timestamps retain the caller's clock
    semantics; this function cannot correct receive-time transport latency.
    """
    stamp = _stamp(stamp)
    max_gap = _number(max_gap, 'max_gap')
    if max_gap <= 0.:
        raise ValueError('max_gap must be positive')
    records = [_odom(item) for item in odoms]
    if not records:
        raise ValueError('no odom samples')
    if any(right['stamp'] <= left['stamp'] for left, right in zip(records, records[1:])):
        raise ValueError('odom stamps must strictly increase')
    if stamp < records[0]['stamp'] or stamp > records[-1]['stamp']:
        raise ValueError('requested stamp is outside odom coverage; extrapolation refused')
    for record in records:
        if record['stamp'] == stamp:
            return dict(record)
    for left, right in zip(records, records[1:]):
        if left['stamp'] < stamp < right['stamp']:
            interval = right['stamp'] - left['stamp']
            if interval > max_gap:
                raise ValueError('odom interpolation gap exceeds max_gap')
            fraction = (stamp - left['stamp']) / interval
            return _odom(dict(stamp=stamp,
                              x=left['x'] + fraction * (right['x'] - left['x']),
                              y=left['y'] + fraction * (right['y'] - left['y']),
                              yaw=left['yaw'] + fraction * wrap(right['yaw'] - left['yaw'])))
    raise ValueError('no odom interval covers requested stamp')


def _alignment_pair(item):
    if not isinstance(item, dict):
        raise ValueError('alignment pair must contain stamp, odom and enu')
    try:
        stamp, record = _stamp(item['stamp']), _odom(item['odom'])
        enu = _vector(item['enu'], 3, 'antenna ENU')
    except KeyError as error:
        raise ValueError('alignment pair requires stamp, odom and enu') from error
    if abs(stamp - record['stamp']) > 1e-6:
        raise ValueError('GNSS and odom stamps are not paired')
    return stamp, record, enu


def fit_alignment(pairs, antenna_xy, min_displacement=.75, max_residual=.5):
    """Fit odom antenna -> fixed ENU antenna using a 2-D rigid least squares fit.

    Each pair contains stamp, an interpolated odom record at that stamp and a
    three-component antenna ENU point. Times must strictly increase and agree
    within one microsecond. No scale, frame reset or clock offset is estimated.

    displacement_m is the smaller of the largest base odom displacement and
    antenna ENU displacement from their first samples. Requiring base motion
    prevents turn-in-place lever motion from qualifying as initialization.
    max_residual bounds horizontal RMSE. heading_std_rad is an initialization
    uncertainty assumption: max(5 degrees, atan2(RMSE, displacement)). The floor
    remains even for exact synthetic fits; it is not a receiver accuracy claim.
    """
    lever = _vector(antenna_xy, 2, 'antenna lever')
    minimum = _number(min_displacement, 'min_displacement')
    maximum = _number(max_residual, 'max_residual')
    if minimum <= 0. or maximum <= 0.:
        raise ValueError('min_displacement and max_residual must be positive')
    samples = []
    for item in pairs:
        stamp, record, enu = _alignment_pair(item)
        if samples and stamp <= samples[-1][0]:
            raise ValueError('alignment stamps must strictly increase')
        samples.append((stamp, record, antenna_odom(record, lever), enu[:2]))
    if len(samples) < 2:
        raise ValueError('at least two paired samples are required')

    first = samples[0]
    odom_displacement = max(math.hypot(item[1]['x'] - first[1]['x'],
                                       item[1]['y'] - first[1]['y']) for item in samples)
    enu_displacement = max(math.hypot(item[3][0] - first[3][0],
                                     item[3][1] - first[3][1]) for item in samples)
    displacement = min(odom_displacement, enu_displacement)
    if displacement < minimum:
        raise ValueError('insufficient base/antenna displacement for geographic heading')

    count = len(samples)
    mean_p = [sum(item[2][axis] for item in samples) / count for axis in (0, 1)]
    mean_q = [sum(item[3][axis] for item in samples) / count for axis in (0, 1)]
    dot, cross = 0., 0.
    for _, _, p, q in samples:
        px, py = p[0] - mean_p[0], p[1] - mean_p[1]
        qx, qy = q[0] - mean_q[0], q[1] - mean_q[1]
        dot += px * qx + py * qy
        cross += px * qy - py * qx
    _number(dot, 'alignment covariance')
    _number(cross, 'alignment covariance')
    if math.hypot(dot, cross) < 1e-12:
        raise ValueError('paired trajectories have no identifiable common heading')
    yaw = wrap(math.atan2(cross, dot))
    rotated_mean = transform_xy(*mean_p, yaw, 0., 0.)
    translation = [mean_q[axis] - rotated_mean[axis] for axis in (0, 1)]
    squared_error = 0.
    for _, _, p, q in samples:
        predicted = transform_xy(*p, yaw, *translation)
        squared_error += (predicted[0] - q[0]) ** 2 + (predicted[1] - q[1]) ** 2
    rmse = _number(math.sqrt(squared_error / count), 'alignment RMSE')
    if rmse > maximum:
        raise ValueError('alignment residual exceeds max_residual')
    return dict(yaw=yaw, translation=translation, rmse_m=rmse,
                displacement_m=displacement, span_s=samples[-1][0] - samples[0][0],
                samples=count, heading_std_rad=max(math.radians(5.), math.atan2(rmse, displacement)))


def _base_distance(left, right):
    first, second = left['odom'], right['odom']
    return math.hypot(first['x'] - second['x'], first['y'] - second['y'])


def _antenna_distance(left, right):
    return math.hypot(left['enu'][0] - right['enu'][0], left['enu'][1] - right['enu'][1])


def _baseline(samples):
    if not samples:
        return 0.
    first = samples[0]
    return min(max(_base_distance(first, item) for item in samples),
               max(_antenna_distance(first, item) for item in samples))


def fit_trajectory_alignment(pairs, antenna_xy, min_baseline_m=1.2, spacing_m=.20,
                             min_samples=6, max_gap_s=1.5, max_residual_m=.15,
                             max_heading_std_rad=.11, max_heading_delta_rad=.10,
                             resume_odom_m=.10, resume_enu_m=.20):
    """Fit the latest continuous trajectory using spatially distinct pairs.

    Report observed path length, an effective baseline and heading consistency
    separately. An explicit resume can bridge a time gap only at a coincident
    odom and GNSS anchor. A gap during collection starts a new trajectory.
    """
    config = (min_baseline_m, spacing_m, max_gap_s, max_residual_m, max_heading_std_rad,
              max_heading_delta_rad, resume_odom_m, resume_enu_m)
    if any(_number(value, 'automatic alignment threshold') <= 0. for value in config):
        raise ValueError('automatic alignment thresholds must be positive')
    if type(min_samples) is not int or min_samples < 5:
        raise ValueError('automatic alignment needs at least five distinct positions')

    selected, previous = [], None
    path_length, discontinuity = 0., False
    for item in pairs:
        stamp, record, enu = _alignment_pair(item)
        point = dict(stamp=stamp, odom=record, enu=enu)
        if previous is not None:
            elapsed = stamp - previous['stamp']
            if elapsed <= 0.:
                raise ValueError('alignment stamps must strictly increase')
            base_step = _base_distance(previous, point)
            if item.get('resumed'):
                contiguous = base_step <= resume_odom_m and _antenna_distance(previous, point) <= resume_enu_m
            else:
                contiguous = elapsed <= max_gap_s
            if contiguous:
                path_length += base_step
            else:
                selected.clear()
                path_length, discontinuity = 0., True
        previous = point
        # Returning to an already sampled area contributes no extra fit weight.
        if all(_base_distance(point, candidate) >= spacing_m-1e-9 for candidate in selected):
            selected.append(point)

    progress = dict(ready=False, reason='', samples=len(selected), rejected_samples=0,
                    baseline_m=_baseline(selected), path_length_m=path_length, residual_m=None,
                    heading_std_rad=None, heading_delta_rad=None, discontinuity=discontinuity)

    def pending(reason):
        return None, dict(progress, reason=reason)

    if len(selected) < min_samples or progress['baseline_m'] < min_baseline_m:
        reason = 'need_more_distinct_positions' if len(selected) < min_samples else 'insufficient_spatial_baseline'
        return pending('paired_data_gap; '+reason if discontinuity else reason)

    try:
        # An initial loose fit identifies isolated jumps before enforcing the
        # residual gate; multiple large residuals are grounds to wait.
        estimate = fit_alignment(selected, antenna_xy, min_baseline_m, float_info.max)
        residuals = []
        for sample in selected:
            predicted = transform_xy(*antenna_odom(sample['odom'], antenna_xy),
                                     estimate['yaw'], *estimate['translation'])
            residuals.append(math.hypot(predicted[0]-sample['enu'][0], predicted[1]-sample['enu'][1]))
        cutoff = max(2.*max_residual_m, 3.*median(residuals))
        inliers = [sample for sample, residual in zip(selected, residuals) if residual <= cutoff]
        rejected = len(selected) - len(inliers)
        progress.update(samples=len(inliers), rejected_samples=rejected, baseline_m=_baseline(inliers))
        if rejected > max(1, int(.15*len(selected))):
            return pending('too_many_alignment_outliers')
        if len(inliers) < min_samples or progress['baseline_m'] < min_baseline_m:
            return pending('insufficient_inlier_positions_or_baseline')

        # Overlap the early and late fits so a short straight track still has
        # enough separation. This detects conflicting heading estimates while
        # keeping the same least-squares problem and antenna reference.
        estimate = fit_alignment(inliers, antenna_xy, min_baseline_m, float_info.max)
        progress.update(residual_m=estimate['rmse_m'], heading_std_rad=estimate['heading_std_rad'])
        if estimate['rmse_m'] > max_residual_m:
            return pending('alignment_residual_too_large')
        if estimate['heading_std_rad'] > max_heading_std_rad:
            return pending('alignment_heading_uncertain')
        group_size = (2*len(inliers)+2)//3
        subfits = [fit_alignment(group, antenna_xy, .4*min_baseline_m, max_residual_m)
                   for group in (inliers[:group_size], inliers[-group_size:])]
        early, late = (fit['yaw'] for fit in subfits)
        delta = max(abs(wrap(early - late)), abs(wrap(early - estimate['yaw'])),
                    abs(wrap(late - estimate['yaw'])))
        progress['heading_delta_rad'] = delta
        if delta > max_heading_delta_rad:
            return pending('alignment_heading_inconsistent')
    except ValueError as error:
        return pending(str(error))
    progress['ready'] = True
    return dict(estimate, rejected_samples=rejected, heading_delta_rad=delta), progress


def _origin(origin):
    if not isinstance(origin, dict):
        raise ValueError('origin must specify WGS84 latitude, longitude and ellipsoid height')
    try:
        latitude = _number(origin['latitude'], 'origin latitude')
        longitude = _number(origin['longitude'], 'origin longitude')
        altitude = _number(origin['altitude_ellipsoid_m'], 'origin ellipsoid height')
    except KeyError as error:
        raise ValueError('origin requires latitude, longitude and altitude_ellipsoid_m') from error
    # Reuse the receiver package's WGS84 conversion and geographic range checks.
    xyz = geodetic_to_ecef(latitude, longitude, altitude)
    lat, lon = math.radians(latitude), math.radians(longitude)
    return xyz, math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)


def _enu_ecef(point_xyz, origin):
    east, north, up = _vector(point_xyz, 3, 'ENU point')
    source, slat, clat, slon, clon = _origin(origin)
    return [source[0]-slon*east-slat*clon*north+clat*clon*up,
            source[1]+clon*east-slat*slon*north+clat*slon*up,
            source[2]+clat*north+slat*up]


def enu_to_geodetic(point_xyz, origin):
    """Inverse of the existing WGS84 ENU geometry; no new height measurement.

    Caller supplies the Up convention. Capture retains antenna Up and only
    compensates the horizontal lever; the returned height is not base height.
    """
    x, y, z = _enu_ecef(point_xyz, origin)
    radius = math.hypot(x, y)
    if radius < 1e-9:
        polar_radius = WGS84_A*math.sqrt(1.-WGS84_E2)
        return (math.copysign(90., z), origin['longitude'], abs(z)-polar_radius)
    latitude = math.atan2(z, radius*(1.-WGS84_E2))
    for _ in range(12):
        normal = WGS84_A/math.sqrt(1.-WGS84_E2*math.sin(latitude)**2)
        updated = math.atan2(z+WGS84_E2*normal*math.sin(latitude), radius)
        if abs(updated-latitude) < 1e-15:
            latitude = updated
            break
        latitude = updated
    normal = WGS84_A/math.sqrt(1.-WGS84_E2*math.sin(latitude)**2)
    # Projection on the ellipsoid normal avoids p/cos(latitude) near the poles.
    height = radius*math.cos(latitude)+z*math.sin(latitude)-normal*(1.-WGS84_E2*math.sin(latitude)**2)
    return math.degrees(latitude), math.degrees(math.atan2(y, x)), height


def enu_between(point_xyz, source_origin, target_origin):
    """Transform a point between explicit WGS84 ENU frames via ECEF, in metres.

    Both origins use latitude/longitude in degrees and altitude_ellipsoid_m.
    Source and target axes may differ; an ENU point is not a simple origin
    translation. Datum and height-type declarations belong to the caller.
    """
    source = _enu_ecef(point_xyz, source_origin)
    target, t_slat, t_clat, t_slon, t_clon = _origin(target_origin)
    x, y, z = (source[i]-target[i] for i in range(3))
    return _vector([-t_slon * x + t_clon * y,
                    -t_slat * t_clon * x - t_slat * t_slon * y + t_clat * z,
                    t_clat * t_clon * x + t_clat * t_slon * y + t_slat * z],
                   3, 'target ENU point')
