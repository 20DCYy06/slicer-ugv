#!/usr/bin/env python3
"""Record a built workspace's release metadata using files and local state only."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import tempfile
from urllib.parse import urlsplit

import yaml


APT_FILENAME = 'apt-packages.tsv'
MANIFEST_FILENAME = 'release.json'
HARDWARE_PATH = 'deps/slicer_ugv_hardware'


def command(*args, env=None):
    return subprocess.check_output(args, env=env, stderr=subprocess.PIPE)


def git(repository, *args):
    env = dict(os.environ)
    for key in ('GIT_DIR', 'GIT_COMMON_DIR', 'GIT_WORK_TREE', 'GIT_OBJECT_DIRECTORY',
                'GIT_ALTERNATE_OBJECT_DIRECTORIES', 'GIT_INDEX_FILE'):
        env.pop(key, None)
    return command('git', '--no-optional-locks', '-C', str(repository), *args, env=env)


def checked_url(value):
    if not isinstance(value, str) or not value:
        raise ValueError('Missing Git remote URL')
    parsed = urlsplit(value)
    if parsed.password or parsed.query or parsed.fragment or (parsed.scheme in ('http', 'https') and parsed.username):
        raise ValueError('Release Git URLs must not contain credentials or query values')
    return value


def checkout(repository):
    if Path(git(repository, 'rev-parse', '--show-toplevel').decode().strip()) != repository:
        raise ValueError(f'{repository} is not the root of a checked-out repository')
    try:
        git(repository, 'diff', '--quiet', 'HEAD', '--')
    except subprocess.CalledProcessError as error:
        raise ValueError(f'Tracked source differs from Git HEAD: {repository}') from error
    return {
        'sha': git(repository, 'rev-parse', '--verify', 'HEAD').decode().strip(),
        'url': checked_url(git(repository, 'remote', 'get-url', 'origin').decode().strip()),
    }


def dependency_pins(workspace, vehicle_path):
    raw = (vehicle_path / 'dependencies.repos').read_bytes()
    if raw != git(vehicle_path, 'show', 'HEAD:dependencies.repos'):
        raise ValueError('dependencies.repos differs from the checked-out vehicle commit')
    repositories = yaml.safe_load(raw)['repositories']
    if not isinstance(repositories, dict) or HARDWARE_PATH not in repositories:
        raise ValueError('dependencies.repos must include the hardware checkout')
    dependencies = {}
    for relative, pin in sorted(repositories.items()):
        relative_path = Path(relative)
        path = workspace / relative_path
        if (relative_path.is_absolute() or path.resolve(strict=True) != path
                or workspace not in path.parents):
            raise ValueError(f'Dependency must be a checkout inside the workspace: {relative}')
        locked = {'sha': pin['version'], 'url': checked_url(pin['url'])}
        if pin.get('type') != 'git' or not re.fullmatch(r'[0-9a-f]{40}', locked['sha']):
            raise ValueError(f'Dependency needs a full Git SHA: {relative}')
        if checkout(path) != locked:
            raise ValueError(f'Dependency checkout or origin does not match pin: {relative}')
        dependencies[relative] = locked
    return dependencies


def digest_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def configuration_digests(workspace):
    configs = {}
    for relative in ('runtime/no_rtk', 'runtime/rtk_config'):
        directory = workspace / relative
        if directory.resolve(strict=True) != directory or not directory.is_dir():
            raise ValueError(f'Missing local runtime configuration directory: {relative}')
        files = sorted(path for path in directory.rglob('*') if path.suffix in ('.yaml', '.yml', '.json'))
        if not files:
            raise ValueError(f'No generated configuration files in {relative}')
        for path in files:
            if path.resolve(strict=True) != path or not path.is_file():
                raise ValueError(f'Redirected runtime configuration: {path}')
            configs[path.relative_to(workspace).as_posix()] = digest_file(path)
    return configs


def firmware_metadata(workspace, image, trusted_record):
    hardware = workspace / HARDWARE_PATH
    ledger = hardware / 'last_install.json'
    historical = json.loads(trusted_record.read_text()) if trusted_record else None
    baseline = json.loads(ledger.read_text()) if ledger.is_file() else {}
    version = (historical or baseline).get('version')
    if not isinstance(version, str) or not re.fullmatch(r'v[A-Za-z0-9._-]+', version):
        version = None
    if image is None:
        if version is None:
            raise ValueError('Provide --firmware-image when the hardware version is unavailable')
        image = hardware / 'Firmware' / version / 'bin/firmware.bin'
    image = image.resolve(strict=True)
    image.relative_to(hardware)
    sha = digest_file(image)
    expected_image = historical.get('image_sha256') if historical is not None else None
    if historical is not None and expected_image != sha:
        raise ValueError('Firmware image does not match the supplied historical record')
    if historical is None and baseline.get('image_sha256') not in (None, sha):
        version = None
    record = {'image': image.relative_to(workspace).as_posix(), 'image_sha256': sha}
    if version:
        record['version'] = version
    if historical is not None:
        readback = historical['readback_sha256']
        if not re.fullmatch(r'[0-9a-f]{64}', readback):
            raise ValueError('Historical firmware readback SHA-256 is invalid')
        verified = historical.get('verified') is True
        expected = historical.get('expected_sha256')
        if verified and expected is not None and expected != readback:
            raise ValueError('Historical firmware readback and expected SHA-256 disagree')
        record['historical_readback'] = {'sha256': readback, 'recorded_verified': verified}
    return record


def build_environment(workspace, sdk, build_command):
    os_release = {}
    for line in Path('/etc/os-release').read_text().splitlines():
        key, _, value = line.partition('=')
        if key in ('ID', 'VERSION_ID', 'PRETTY_NAME'):
            os_release[key] = value.strip('"')
    toolchain = {}
    cache_path = workspace / 'build/CMakeCache.txt'
    cache = {}
    if cache_path.is_file():
        for line in cache_path.read_text().splitlines():
            key, _, value = line.partition('=')
            if key in ('CMAKE_C_COMPILER:FILEPATH', 'CMAKE_CXX_COMPILER:FILEPATH',
                       'CMAKE_HOME_DIRECTORY:INTERNAL', 'CMAKE_INSTALL_PREFIX:PATH',
                       'LIVOX_SDK2_ROOT:PATH', 'SLICER_HARDWARE_CONFIG:FILEPATH'):
                cache[key] = value
        for key, expected in (
            ('CMAKE_HOME_DIRECTORY:INTERNAL', workspace / 'src'),
            ('CMAKE_INSTALL_PREFIX:PATH', workspace / 'install'),
            ('LIVOX_SDK2_ROOT:PATH', sdk),
        ):
            if key in cache and Path(cache[key]).resolve() != expected:
                raise ValueError(f'CMake cache path is inconsistent with this release: {key}')
    for name, path in (
        ('cmake', 'cmake'), ('make', 'make'), ('cc', cache.get('CMAKE_C_COMPILER:FILEPATH', 'cc')),
        ('cxx', cache.get('CMAKE_CXX_COMPILER:FILEPATH', 'c++')),
    ):
        toolchain[name] = command(path, '--version').decode(errors='replace').splitlines()[0]
    toolchain['python'] = platform.python_version()
    config = cache.get('SLICER_HARDWARE_CONFIG:FILEPATH')
    hardware_input = ({'path': config, 'sha256': digest_file(Path(config))} if config else None)
    return {
        'command': build_command, 'os': os_release, 'architecture': platform.machine(),
        'toolchain': toolchain, 'hardware_config_input': hardware_input,
    }


def apt_packages():
    fmt = '${db:Status-Abbrev}\t${binary:Package}\t${Version}\t${Architecture}\n'
    installed = command('dpkg-query', '-W', '-f=' + fmt).splitlines()
    selected = sorted(row.split(b'\t', 1)[1] for row in installed if row[1:2] == b'i')
    if not selected:
        raise ValueError('dpkg-query did not report installed packages')
    return b'package\tversion\tarchitecture\n' + b'\n'.join(selected) + b'\n'


def verification_status(values):
    statuses = {}
    for entry in values:
        step, separator, result = entry.partition('=')
        if (not separator or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', step)
                or result not in ('passed', 'failed', 'not-run') or step in statuses):
            raise ValueError('Use unique --verification NAME=passed|failed|not-run values')
        statuses[step] = result
    return statuses


def record_release(options):
    workspace = options.workspace.resolve(strict=True)
    vehicle_path = workspace / 'src/slicer-ugv'
    install = workspace / 'install/setup.bash'
    if not install.is_file() or install.resolve() != install or vehicle_path.resolve() != vehicle_path:
        raise ValueError('Release needs its own vehicle checkout and install/setup.bash')
    if not options.build_command.strip():
        raise ValueError('Provide the actual --build-command')
    publish_targets = [workspace / name for name in (APT_FILENAME, MANIFEST_FILENAME)]
    if any(os.path.lexists(path) for path in publish_targets):
        raise ValueError(f'{APT_FILENAME} or {MANIFEST_FILENAME} already exists in {workspace}')
    sdk = (options.sdk_prefix or workspace / 'sdk/install').resolve(strict=True)
    sdk.relative_to(workspace)
    if not sdk.is_dir():
        raise ValueError('SDK prefix must be an installed directory inside the workspace')
    statuses = verification_status(options.verification)
    dependencies = dependency_pins(workspace, vehicle_path)
    apt = apt_packages()
    metadata = {
        'schema_version': 1,
        'release_id': workspace.name,
        'workspace': str(workspace),
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'vehicle': checkout(vehicle_path),
        'dependencies': dependencies,
        'dependency_manifest': {
            'file': 'src/slicer-ugv/dependencies.repos',
            'sha256': digest_file(vehicle_path / 'dependencies.repos'),
        },
        'build': build_environment(workspace, sdk, options.build_command),
        'apt_packages': {'file': APT_FILENAME, 'sha256': hashlib.sha256(apt).hexdigest()},
        'runtime_config_sha256': configuration_digests(workspace),
        'sdk': {'prefix': str(sdk)},
        'firmware': firmware_metadata(workspace, options.firmware_image, options.firmware_record),
        'verification': statuses,
    }
    # Publish the completed JSON only after its companion inventory exists.
    with tempfile.TemporaryDirectory(dir=workspace) as temporary:
        inventory = Path(temporary) / APT_FILENAME
        manifest = Path(temporary) / MANIFEST_FILENAME
        inventory.write_bytes(apt)
        manifest.write_text(json.dumps(metadata, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        os.link(inventory, publish_targets[0])
        try:
            os.link(manifest, publish_targets[1])
        except OSError:
            publish_targets[0].unlink()
            raise
    return publish_targets[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--build-command', required=True, help='Command actually used to build the release')
    parser.add_argument('--sdk-prefix', type=Path, help='Selected SDK install path, default W/sdk/install')
    parser.add_argument('--firmware-image', type=Path, help='Expected firmware image inside the hardware checkout')
    parser.add_argument('--firmware-record', type=Path, help='Trusted historical ledger for this firmware image')
    parser.add_argument('--verification', action='append', default=[], metavar='NAME=STATUS')
    try:
        manifest = record_release(parser.parse_args())
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError, yaml.YAMLError) as error:
        parser.exit(1, f'Release record: {error}\n')
    print(manifest)


if __name__ == '__main__':
    main()
