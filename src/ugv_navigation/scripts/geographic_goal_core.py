"""Pure geographic-goal conversion and observation-based arrival review.

All navigation targets describe base_link. Arrival uses the manager's already
compensated observation. Capture alone converts raw antenna fixes once; loading
the resulting WGS84 target never reapplies that horizontal lever.
"""
import json
import math

from cup_gnss.enu import geodetic_to_ecef, to_enu
from geographic_core import base_from_antenna, enu_to_geodetic, interpolate_odom
from geographic_state import qualified_fix


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def origin_record(record):
    if not isinstance(record, dict) or record.get('datum') != 'WGS84':
        raise ValueError('origin datum must explicitly be WGS84; no datum conversion is implemented')
    origin = record.get('origin')
    if not isinstance(origin, dict):
        raise ValueError('origin must contain latitude, longitude and altitude_ellipsoid_m')
    geodetic_to_ecef(origin.get('latitude'), origin.get('longitude'), origin.get('altitude_ellipsoid_m'))
    return origin


def status_problem(status, received, now, ros_now, generation=None, max_age=1.):
    if not isinstance(status, dict):
        return 'geographic status unavailable or malformed'
    if received is None or not 0. <= now-received <= max_age:
        return 'geographic status receive stale'
    stamp = status.get('status_stamp')
    if not finite(stamp) or not -.1 <= ros_now-stamp <= max_age:
        return 'geographic status stamp stale or invalid'
    if status.get('state') == 'invalid':
        return 'geographic registration invalid: ' + str(status.get('reason', ''))
    if status.get('registered') is not True or status.get('state') not in ('ready', 'degraded'):
        return 'geographic task is not registered: ' + str(status.get('reason', ''))
    if type(status.get('generation')) is not int:
        return 'geographic generation unavailable'
    if generation is not None and status['generation'] != generation:
        return 'geographic registration generation changed'
    return None


def registration_observation_problem(status, ros_now, max_fix_age):
    if type(status.get('registration_samples')) is not int or status['registration_samples'] < 1:
        return 'no qualified stationary GNSS sample; no initialization goal sent'
    stamp = status.get('qualified_stamp')
    if not finite(stamp) or not -.1 <= ros_now-stamp <= max_fix_age:
        return 'qualified registration GNSS sample stale; no initialization goal sent'
    return None


def load_wgs84_target(target_file):
    with target_file.open(encoding='utf-8') as stream:
        record = json.load(stream)
    if (not isinstance(record, dict) or record.get('kind') != 'slicer_geographic_target'
            or type(record.get('version')) is not int or record['version'] != 1
            or record.get('datum') != 'WGS84'
            or record.get('physical_reference') != 'base_link_horizontal'):
        raise ValueError('unsupported WGS84 base_link target file')
    lat, lon, height = (record.get(key) for key in ('latitude', 'longitude', 'altitude_ellipsoid_m'))
    geodetic_to_ecef(lat, lon, height)
    return dict(latitude=lat, longitude=lon, altitude_ellipsoid_m=height,
                height_semantics=record.get('height_semantics'))


def resolve_target(args, status, current_pose, file_intent=None):
    """Return one fixed map target with original input and explicit yaw policy."""
    origin = origin_record(status)
    original = {'datum': 'WGS84', 'yaw': args.yaw}
    target_file = getattr(args, 'target_file', None)
    if target_file is not None:
        record = file_intent if file_intent is not None else load_wgs84_target(target_file)
        lat, lon, height = (record[key] for key in ('latitude', 'longitude', 'altitude_ellipsoid_m'))
        xyz = list(to_enu(lat, lon, height, origin))
        original.update(kind='captured_wgs84', target_file=str(target_file), latitude=lat,
                        longitude=lon, altitude_ellipsoid_m=height,
                        height_semantics=record.get('height_semantics'))
        # WGS84 coordinates are portable; capture origin/generation are provenance,
        # never a requirement to reuse the old registration or compensate again.
        yaw = args.yaw
    elif args.latitude is not None:
        original.update(kind='geodetic', latitude=args.latitude, longitude=args.longitude,
                        altitude_ellipsoid_m=args.altitude_ellipsoid)
        xyz = list(to_enu(args.latitude, args.longitude, args.altitude_ellipsoid, origin))
        yaw = args.yaw
    else:
        original.update(kind='enu', east_m=args.east, north_m=args.north, up_m=args.up)
        xyz, yaw = [args.east, args.north, args.up], args.yaw
        if args.origin_file is not None:
            with args.origin_file.open(encoding='utf-8') as stream:
                source_record = json.load(stream)
            source = origin_record(source_record)
            # The interface declares true East/North/Up metres; arbitrary local
            # axes require conversion by the producer before using this CLI.
            if source_record.get('axes', 'ENU') != 'ENU' or source_record.get('units', 'm') != 'm':
                raise ValueError('ENU origin file axes must be ENU and units must be m')
            from geographic_core import enu_between
            original['source_origin'] = source_record
            xyz = list(enu_between(xyz, source, origin))
            if yaw is not None:
                tip = enu_between((args.east+math.cos(yaw), args.north+math.sin(yaw), args.up), source, origin)
                yaw = math.atan2(tip[1]-xyz[1], tip[0]-xyz[0])
        else:
            if not args.session_enu:
                raise ValueError('ENU target requires --origin-file or --session-enu')
            original['source_origin'] = {'datum': 'WGS84', 'origin': dict(origin), 'axes': 'ENU', 'units': 'm'}
    if yaw is None:
        dx, dy = xyz[0]-current_pose[0], xyz[1]-current_pose[1]
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else current_pose[2]
        policy = 'bearing from current base to target; retain current yaw when coincident'
    else:
        policy = 'explicit yaw counterclockwise from source ENU East, converted to task ENU'
    return dict(original=original, map_xyz=xyz, frame='map', yaw=yaw, yaw_policy=policy,
                physical_point='base_link', task_origin=dict(origin), datum='WGS84')


def capture_sample(fix, quality, global_poses, lever, origin, ros_now, max_fix_age, max_pose_gap=.5):
    """One same-epoch raw antenna/quality pair with time-matched map yaw."""
    problem = qualified_fix(fix, quality, ros_now, max_fix_age)
    if problem:
        raise ValueError(problem)
    pose = interpolate_odom(global_poses, fix['stamp'], max_pose_gap)
    antenna = list(to_enu(fix['latitude'], fix['longitude'], fix['altitude'], origin))
    horizontal = base_from_antenna(antenna, pose['yaw'], lever)
    return dict(stamp=fix['stamp'], stamp_ns=fix['stamp_ns'],
                base_enu=horizontal+[antenna[2]], yaw=pose['yaw'],
                antenna_height_m=fix['altitude'], session=quality.get('session'),
                gga_quality=4, fresh_rtk_usable=True)


def capture_target(samples, origin, stop, duration):
    """Summarize new stationary samples as a portable two-dimensional target."""
    start, end = stop.get('window_start_stamp'), stop.get('window_end_stamp')
    if stop.get('confirmed') is not True or not finite(start) or not finite(end):
        raise ValueError('capture requires a confirmed stationary window')
    samples = [row for row in samples if start <= row['stamp'] <= end]
    if len(samples) < 2:
        raise ValueError('capture needs at least two qualified samples in the stationary window')
    count = len(samples)
    mean = [math.fsum(row['base_enu'][i] for row in samples)/count for i in range(3)]
    latitude, longitude, height = enu_to_geodetic(mean, origin)
    variances = [math.fsum((row['base_enu'][i]-mean[i])**2 for row in samples)/count for i in (0, 1)]
    return dict(kind='slicer_geographic_target', version=1, datum='WGS84',
                latitude=latitude, longitude=longitude, altitude_ellipsoid_m=height,
                physical_reference='base_link_horizontal',
                height_semantics='2D horizontal base_link; mean GNSS antenna Up retained, not measured base_link height',
                capture=dict(start_stamp=min(row['stamp'] for row in samples),
                             end_stamp=max(row['stamp'] for row in samples),
                             requested_duration_s=duration, sample_count=count,
                             stationary_window=stop, source_origin=dict(origin),
                             mean_base_enu_m=mean,
                             horizontal_std_m=[math.sqrt(value) for value in variances],
                             horizontal_rms_m=math.sqrt(sum(variances)),
                             max_horizontal_radius_m=max(math.hypot(row['base_enu'][0]-mean[0], row['base_enu'][1]-mean[1]) for row in samples),
                             mean_antenna_ellipsoid_height_m=math.fsum(row['antenna_height_m'] for row in samples)/count,
                             quality=dict(gga_quality=4, fresh_rtk_usable=True,
                                          receiver_sessions=sorted({str(row['session']) for row in samples})),
                             timestamp_semantics='host reception mapped to ROS time; receiver internal latency is not corrected',
                             note='sample scatter is not surveyed absolute accuracy'))


def quiet_odom_tail(rows, now, ros_now, window=1., max_age=.5):
    result = dict(confirmed=False, source='/odom', window_s=window,
                  note='fresh continuous odometry quiet tail; physical stopping requires field observation')
    if not rows:
        return dict(result, reason='no fresh odometry quiet tail')
    latest = rows[-1]
    result['latest_stamp'] = latest['stamp']
    if not (0. <= now-latest['received'] <= max_age and -.1 <= ros_now-latest['stamp'] <= max_age):
        return dict(result, reason='odometry receipt or source stamp stale')
    tail = []
    for row in reversed(rows):
        if not all(finite(row.get(key)) for key in ('received', 'stamp', 'speed', 'angular_speed', 'x', 'y')):
            break
        if abs(row['speed']) > .02 or abs(row['angular_speed']) > .05:
            break
        if tail and not (0. < tail[-1]['received']-row['received'] <= max_age
                         and 0. < tail[-1]['stamp']-row['stamp'] <= max_age):
            break
        if math.hypot(row['x']-latest['x'], row['y']-latest['y']) > .03:
            break
        tail.append(row)
    duration = latest['received']-tail[-1]['received'] if tail else 0.
    source_duration = latest['stamp']-tail[-1]['stamp'] if tail else 0.
    return dict(result, confirmed=duration >= window-1e-6 and source_duration >= window-1e-6,
                observed_quiet_s=duration, source_quiet_s=source_duration,
                window_start_stamp=tail[-1]['stamp'] if tail else None,
                window_end_stamp=latest['stamp'] if tail else None,
                reason='' if duration >= window-1e-6 and source_duration >= window-1e-6
                else 'quiet tail too short, moving, discontinuous or repeated odometry')


def arrival_review(target, status, received, now, ros_now, generation, action_state, stop,
                   tolerance=.35, max_fix_age=2.5, status_max_age=1.):
    """Keep Action, software stopping and original-GNSS position as separate facts."""
    geographic = dict(confirmed=False, qualified=False, horizontal_residual_m=None,
                      tolerance_m=tolerance, reference='original qualified GNSS compensated once to base_link',
                      note='RTK also enters localization; this is not independent surveyed ground truth')
    problem = status_problem(status, received, now, ros_now, generation, status_max_age)
    if not problem and status.get('state') != 'ready':
        problem = 'geographic localization degraded: '+str(status.get('reason', 'RTK not ready'))
    start, end = stop.get('window_start_stamp'), stop.get('window_end_stamp')
    if not problem and (stop.get('confirmed') is not True or not finite(start) or
                        not finite(end) or start > end):
        problem = 'confirmed stopped window unavailable'
    geographic['stopped_window'] = dict(start_stamp=start, end_stamp=end)
    observation = status.get('latest_observation') if isinstance(status, dict) else None
    if not problem and isinstance(observation, dict):
        geographic.update(stamp=observation.get('stamp'), gga_quality=observation.get('gga_quality'),
                          covariance_assumed=observation.get('covariance_assumed'))
        stamp, xy = observation.get('stamp'), observation.get('base_xy')
        if (observation.get('fresh_rtk_usable') is not True or observation.get('gga_quality') != 4
                or not finite(stamp) or not -.1 <= ros_now-stamp <= max_fix_age
                or not isinstance(xy, (list, tuple)) or len(xy) != 2 or not all(finite(x) for x in xy)):
            problem = 'fresh qualified original RTK base observation unavailable'
        elif not start <= stamp <= end:
            problem = 'qualified RTK does not belong to this stopped window'
        else:
            residual = math.hypot(xy[0]-target['map_xyz'][0], xy[1]-target['map_xyz'][1])
            geographic.update(qualified=True, base_xy=list(xy), horizontal_residual_m=residual,
                              confirmed=action_state == 3 and stop.get('confirmed') is True and residual <= tolerance)
            if not geographic['confirmed']:
                problem = 'Action success, quiet odometry and geographic tolerance must all hold'
    elif not problem:
        problem = 'qualified original GNSS observation unavailable'
    geographic['reason'] = problem or ''
    return dict(target=target, generation=generation, evaluated_stamp=ros_now,
                move_base={'state': action_state}, stop=stop, geographic=geographic)
