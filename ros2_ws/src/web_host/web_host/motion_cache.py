"""Content-addressed offline conversion results; partial/corrupt entries are misses."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile


FILES = ('motion.npz', 'preview.json', 'report.json', 'robot.urdf')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def result_key(job, mapping):
    directory = job['directory']
    inputs = {name: digest(directory / name) for name in ('source.bvh', 'robot.urdf')}
    for name in ('web_host.motion', 'web_host.motion_cache', 'web_host.motion_simulation', 'web_host.motion_source',
                 'web_host.shoulder_retarget', 'bvh_broadcaster.bvh',
                 'bvh_broadcaster.generic_retarget', 'bvh_broadcaster.retarget',
                 'robot_transmission.coupled_limits', 'robot_transmission.mujoco_positions'):
        inputs[name] = digest(Path(importlib.util.find_spec(name).origin))
    import numpy
    import scipy
    import pinocchio
    inputs['runtime'] = [numpy.__version__, scipy.__version__, pinocchio.__version__]
    simulation = job.get('simulation')
    if simulation:
        from .motion_simulation import load_mujoco
        inputs['mujoco_version'] = load_mujoco().__version__
        model = simulation['directory']
        for path in sorted((model / 'mujoco').glob('*.xml')):
            inputs[str(path.relative_to(model))] = digest(path)
        inputs['standalone'] = digest(model / 'urdf' / f'{model.name}.standalone.urdf')
        inputs['constraints'] = simulation['config_sha256']
    selected = {key: value for key, value in mapping.items() if key != 'targets'}
    if 'targets' in mapping:
        selected['targets'] = sorted(({'target': entry['target'], 'source': entry['source']}
                                     for entry in mapping['targets'] if entry.get('source')),
                                    key=lambda entry: entry['target'])
    identity = {'inputs': inputs, 'mapping': selected, 'generic': job.get('generic', False),
                'up_axis': job['up_axis'], 'scale': job['scale']}
    return hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()


class ResultCache:
    def __init__(self, directory):
        self.directory = Path(directory)

    def load(self, key, destination):
        path = self.directory / key
        try:
            manifest_path = path / 'manifest.json'
            if manifest_path.stat().st_size > 16384:
                return False
            manifest = json.loads(manifest_path.read_text())
            if manifest['key'] != key or set(manifest['files']) != set(FILES):
                return False
            for name in FILES:
                file = path / name
                if file.stat().st_size > 128 * 1024 * 1024 or digest(file) != manifest['files'][name]:
                    return False
            for name in FILES:
                shutil.copyfile(path / name, destination / name)
            return True
        except (OSError, ValueError, KeyError, TypeError):
            return False

    def save(self, key, source):
        self.directory.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.writing-', dir=self.directory) as temporary:
            pending = Path(temporary) / key
            pending.mkdir()
            hashes = {}
            for name in FILES:
                shutil.copyfile(source / name, pending / name)
                hashes[name] = digest(pending / name)
            (pending / 'manifest.json').write_text(json.dumps({'key': key, 'files': hashes}))
            destination = self.directory / key
            # Existing entries are disposable cache data, including a failed earlier write.
            if destination.exists():
                shutil.rmtree(destination)
            os.replace(pending, destination)
