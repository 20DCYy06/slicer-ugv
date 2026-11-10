"""Small observation policy; the estimator remains robot_localization."""
import math
from geographic_core import wrap


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def qualified_fix(fix, quality, now, max_age):
    if fix.get('stamp_ns') != quality.get('stamp_ns'):
        return 'quality_epoch_mismatch'
    if not finite(fix.get('stamp')) or not 0 <= now - fix['stamp'] <= max_age:
        return 'fix_stamp_stale_or_future'
    if str(quality.get('gga_quality')) != '4':
        return 'raw_quality_not_fixed'
    if str(quality.get('fresh_rtk_usable')).lower() != 'true' or str(quality.get('fix_valid')).lower() != 'true':
        return 'rtk_not_fresh_or_valid'
    if fix.get('status', -1) < 0:
        return 'invalid_navsat_fix'
    if fix.get('frame') != 'gnss_link':
        return 'expected_raw_gnss_antenna_frame'
    if not all(finite(fix.get(k)) for k in ('latitude', 'longitude', 'altitude')):
        return 'nonfinite_geodetic_or_ellipsoid_height'
    if not -90 <= fix['latitude'] <= 90 or not -180 <= fix['longitude'] <= 180:
        return 'geodetic_out_of_range'
    return ''


def horizontal_covariance(fix, assumed_std):
    covariance = fix.get('covariance', [])
    if fix.get('covariance_type', 0) != 0 and len(covariance) == 9:
        a, b, c, d = [covariance[i] for i in (0, 1, 3, 4)]
        if all(finite(v) for v in (a, b, c, d)) and a > 0 and d > 0 and abs(b-c) < 1e-8 and a*d > b*c:
            return [a, b, c, d], False
    return [assumed_std**2, 0., 0., assumed_std**2], True


def antenna_xy(site, source):
    configured = site.get('antenna_in_base_link_m', [None]*3)
    laser = source['sensors']['base_to_laser']['xyz']
    # User-confirmed antenna above the lidar. Reuse that existing horizontal
    # installation, while keeping explicit antenna coordinates authoritative.
    result = [laser[i] if configured[i] is None else configured[i] for i in (0, 1)]
    if not all(finite(v) for v in result):
        raise ValueError('invalid existing horizontal antenna installation')
    return list(map(float, result))


def odom_reset(previous, current):
    if previous is None:
        return ''
    if (previous.get('frame') != current.get('frame') or
            previous.get('child') != current.get('child')):
        return 'odom_frame_changed'
    dt = current['stamp'] - previous['stamp']
    if dt < 0:
        return 'odom_time_reversed'
    if dt == 0:
        return ('odom_pose_changed_at_repeated_stamp' if
                math.hypot(current['x']-previous['x'], current['y']-previous['y']) > .01 or
                abs(wrap(current['yaw']-previous['yaw'])) > .01 else '')
    if math.hypot(current['x']-previous['x'], current['y']-previous['y']) > max(.75, 2.*dt):
        return 'odom_position_reset'
    if abs(wrap(current['yaw']-previous['yaw'])) > max(.7, 3.*dt):
        return 'odom_heading_reset'
    return ''
