"""ROS-independent validation and mapping for the public air-ground V1 wire.

Coordinates describe horizontal base_link, never an antenna or aircraft point.
No conversion, lever correction, ROS master, or navigation runs in this module.
"""
import json
import math
from pathlib import Path

PROTOCOL_VERSION = 1
COMMAND_TOPIC = '/air_ground/command'
STATE_TOPIC = '/air_ground/state'
OPERATIONS = ('SET_TASK', 'REPLACE_VIA_POINTS', 'PAUSE', 'RESUME', 'CANCEL')
UINT64_MAX = 2**64 - 1


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def normalize_point(point, allow_yaw=True):
    if not isinstance(point, dict):
        raise ValueError('WGS84 point must be an object')
    latitude, longitude = point.get('latitude'), point.get('longitude')
    if not finite(latitude) or not -90. <= latitude <= 90.:
        raise ValueError('WGS84 latitude must be finite degrees in [-90, 90]')
    if not finite(longitude) or not -180. <= longitude <= 180.:
        raise ValueError('WGS84 longitude must be finite degrees in [-180, 180]')
    height, yaw = point.get('altitude_ellipsoid_m'), point.get('yaw')
    if height is not None and not finite(height):
        raise ValueError('altitude_ellipsoid_m must be finite ellipsoid metres or null')
    if yaw is not None and (not allow_yaw or not finite(yaw)):
        raise ValueError('yaw must be finite and is permitted only on the final goal')
    return dict(latitude=float(latitude), longitude=float(longitude),
                altitude_ellipsoid_m=None if height is None else float(height),
                yaw=None if yaw is None else float(yaw))


def load_target(path, yaw=None):
    """Decode the existing geographic capture schema without recompensation."""
    with Path(path).open(encoding='utf-8') as stream:
        record = json.load(stream)
    if (not isinstance(record, dict) or record.get('kind') != 'slicer_geographic_target'
            or type(record.get('version')) is not int or record['version'] != 1
            or record.get('datum') != 'WGS84'
            or record.get('physical_reference') != 'base_link_horizontal'):
        raise ValueError('unsupported WGS84 base_link target file')
    if not finite(record.get('altitude_ellipsoid_m')):
        raise ValueError('captured target requires finite altitude_ellipsoid_m')
    return normalize_point(dict(record, yaw=yaw))


def normalize_command(command):
    if not isinstance(command, dict):
        raise ValueError('command must be an object')
    if type(command.get('protocol_version')) is not int or command['protocol_version'] != PROTOCOL_VERSION:
        raise ValueError('unsupported protocol_version')
    for name in ('server_id', 'request_id', 'task_id'):
        if not isinstance(command.get(name), str) or not command[name].strip():
            raise ValueError(name + ' must be a nonempty string')
    sequence = command.get('sequence')
    if type(sequence) is not int or not 1 <= sequence <= UINT64_MAX:
        raise ValueError('sequence must be a positive uint64')
    operation = command.get('operation')
    if operation not in OPERATIONS:
        raise ValueError('unsupported operation')
    via = command.get('via_points', [])
    if not isinstance(via, list):
        raise ValueError('via_points must be an ordered array')
    if operation not in ('SET_TASK', 'REPLACE_VIA_POINTS') and via:
        raise ValueError(operation + ' does not take via_points')
    final = command.get('final_goal')
    if operation == 'SET_TASK':
        final = normalize_point(final)
    elif final is not None:
        raise ValueError(operation + ' does not take final_goal')
    return dict(protocol_version=PROTOCOL_VERSION, server_id=command['server_id'],
                sequence=sequence, request_id=command['request_id'], task_id=command['task_id'],
                operation=operation, final_goal=final,
                via_points=[normalize_point(point, allow_yaw=False) for point in via])


def _point_to_dict(message):
    if type(message.has_altitude) is not bool or type(message.has_yaw) is not bool:
        raise ValueError('point optional presence flags must be booleans')
    return dict(latitude=message.latitude, longitude=message.longitude,
                altitude_ellipsoid_m=message.altitude_ellipsoid_m if message.has_altitude else None,
                yaw=message.yaw if message.has_yaw else None)


def command_to_dict(message):
    """Strictly validate a formal Command; receivers use decode_command."""
    final = message.final_goal
    if message.operation != 'SET_TASK' and (
            final.latitude != 0. or final.longitude != 0. or final.has_altitude
            or final.altitude_ellipsoid_m != 0. or final.has_yaw or final.yaw != 0.):
        raise ValueError(message.operation + ' does not take final_goal')
    return normalize_command(dict(protocol_version=message.protocol_version, server_id=message.server_id,
                                  sequence=message.sequence, request_id=message.request_id,
                                  task_id=message.task_id, operation=message.operation,
                                  final_goal=_point_to_dict(final) if message.operation == 'SET_TASK' else None,
                                  via_points=[_point_to_dict(point) for point in message.via_points]))


def decode_command(message):
    """Structurally decode for receiver identity checks before semantic validation.

    Keep invalid numeric values, versions and unused payload so TaskCore can
    remember their rejection and reject changed content under the same ID.
    Numeric values behind false optional flags carry no semantic content.
    """
    def point_value(point):
        return dict(latitude=point.latitude, longitude=point.longitude,
                    altitude_ellipsoid_m=point.altitude_ellipsoid_m if point.has_altitude else None,
                    yaw=point.yaw if point.has_yaw else None)

    final = point_value(message.final_goal)
    if (message.operation != 'SET_TASK' and final['latitude'] == 0.
            and final['longitude'] == 0. and final['altitude_ellipsoid_m'] is None
            and final['yaw'] is None):
        final = None
    return dict(protocol_version=message.protocol_version, server_id=message.server_id,
                sequence=message.sequence, request_id=message.request_id, task_id=message.task_id,
                operation=message.operation, final_goal=final,
                via_points=[point_value(point) for point in message.via_points])


def make_command(command, command_type=None, point_type=None):
    """Construct slicer_air_ground_msgs/Command; imports ROS messages on use."""
    value = normalize_command(command)
    if command_type is None or point_type is None:
        from slicer_air_ground_msgs.msg import Command, GeoPoint
        command_type, point_type = Command, GeoPoint

    def point_message(point):
        result = point_type()
        result.latitude, result.longitude = point['latitude'], point['longitude']
        result.has_altitude = point['altitude_ellipsoid_m'] is not None
        result.altitude_ellipsoid_m = point['altitude_ellipsoid_m'] if result.has_altitude else 0.
        result.has_yaw = point['yaw'] is not None
        result.yaw = point['yaw'] if result.has_yaw else 0.
        return result

    result = command_type()
    for name in ('protocol_version', 'server_id', 'sequence', 'request_id', 'task_id', 'operation'):
        setattr(result, name, value[name])
    if value['final_goal'] is not None:
        result.final_goal = point_message(value['final_goal'])
    result.via_points = [point_message(point) for point in value['via_points']]
    return result


def state_problem(state, received, now, ros_now, max_age=2.):
    """Validate both local receive freshness and the receiver's ROS epoch stamp."""
    if not isinstance(state, dict):
        return 'air-ground state unavailable or malformed'
    if received is None or not 0. <= now - received <= max_age:
        return 'air-ground state receive stale'
    if type(state.get('protocol_version')) is not int or state['protocol_version'] != PROTOCOL_VERSION:
        return 'unsupported air-ground protocol_version'
    if not isinstance(state.get('server_id'), str) or not state['server_id'].strip():
        return 'air-ground server_id unavailable'
    sequence = state.get('last_sequence')
    if sequence is not None and (type(sequence) is not int or not 0 <= sequence <= UINT64_MAX):
        return 'air-ground last_sequence invalid'
    stamp = state.get('updated_stamp')
    if not finite(stamp) or not 0. <= ros_now - stamp <= max_age:
        return 'air-ground updated_stamp stale or in the future; check ROS clocks'
    return None
