"""Read-only BVH saved-state inspection using the normal conversion identities."""

import copy
import hashlib
import json
from pathlib import Path
from urllib.parse import urlsplit

from .motion import MotionJobs
from .motion_cache import FILES, digest, result_key

_CATALOG_CACHE_LIMIT = 8


class _CatalogJobs(MotionJobs):
    def start(self, key, mapping, remember=False):
        # Catalog discovery prepares mappings but never schedules conversion.
        return self.status(key)


def _catalog_fingerprint(jobs, request, names):
    hasher = hashlib.sha256()
    urdf_text = request.get('urdf_text')
    urdf_hash = hashlib.sha256(urdf_text.encode('utf-8')).hexdigest() if isinstance(urdf_text, str) else ''
    req_summary = {
        'model': request.get('model'),
        'retargeting': request.get('retargeting'),
        'up_axis': request.get('up_axis'),
        'scale': request.get('scale'),
        'urdf_sha256': urdf_hash,
    }
    hasher.update(json.dumps(req_summary, sort_keys=True).encode('utf-8'))

    if jobs.actions_dir is not None:
        for name in names:
            hasher.update(b'\0bvh\0' + name.encode('utf-8'))
            try:
                hasher.update(digest(jobs.actions_dir / name).encode('ascii'))
            except OSError:
                hasher.update(b'missing')

    if jobs.profiles_dir is not None and jobs.profiles_dir.is_dir():
        for path in sorted(jobs.profiles_dir.glob('*.json')):
            hasher.update(b'\0prof\0' + path.name.encode('utf-8'))
            try:
                hasher.update(digest(path).encode('ascii'))
            except OSError:
                hasher.update(b'missing')

    if jobs.cache is not None and jobs.cache.directory.is_dir():
        for entry in sorted(jobs.cache.directory.iterdir()):
            if not entry.is_dir():
                continue
            hasher.update(b'\0cache\0' + entry.name.encode('utf-8'))
            manifest = entry / 'manifest.json'
            try:
                hasher.update(manifest.read_bytes())
            except OSError:
                hasher.update(b'no-manifest')
            for fname in FILES:
                f = entry / fname
                try:
                    st = f.stat()
                    hasher.update(f'{fname}:{st.st_size}:{st.st_mtime_ns}'.encode('utf-8'))
                except OSError:
                    hasher.update(f'{fname}:missing'.encode('utf-8'))

    model_name = request.get('model')
    if isinstance(model_name, str) and model_name and jobs.models_root is not None:
        model_dir = Path(jobs.models_root) / model_name
        if model_dir.is_dir():
            sim_cfg = model_dir / 'config' / 'simulation_limits.yaml'
            if sim_cfg.is_file():
                try:
                    hasher.update(b'\0simcfg\0' + sim_cfg.read_bytes())
                except OSError:
                    pass
            mujoco_dir = model_dir / 'mujoco'
            if mujoco_dir.is_dir():
                for xml in sorted(mujoco_dir.glob('*.xml')):
                    try:
                        hasher.update(b'\0xml\0' + xml.name.encode('utf-8') + xml.read_bytes())
                    except OSError:
                        pass

    return hasher.hexdigest()


def source_statuses(jobs, request):
    request = {key: value for key, value in request.items() if key != 'bvh_text'}
    names = jobs.sources()
    if jobs.cache is None:
        return [{'name': name, 'saved': False} for name in names]

    fingerprint = _catalog_fingerprint(jobs, request, names)
    with jobs.lock:
        cache = getattr(jobs, '_catalog_cache', None)
        if cache is None:
            cache = {}
            jobs._catalog_cache = cache
        cached = cache.get(fingerprint)
        if cached is not None:
            return copy.deepcopy(cached)

    inspector = _CatalogJobs(jobs.actions_dir, jobs.profiles_dir, jobs.models_root,
                             jobs.cache.directory)
    digest_cache = {}
    try:
        sources = []
        for name in names:
            try:
                status = inspector.create({**request, 'bvh_source': name,
                                           'review_mapping': False})
                job = inspector.jobs[status['id']]
                mapping = status['mapping']
                saved = bool(status['automatic_mapping'] and mapping
                             and inspector.cache.has(result_key(job, mapping, digest_cache=digest_cache)))
                sources.append({'name': name, 'saved': saved})
            except ImportError:
                raise
            except Exception as exc:
                sources.append({'name': name, 'saved': False, 'error': str(exc)})
    finally:
        inspector.close()

    with jobs.lock:
        cache = getattr(jobs, '_catalog_cache', None)
        if cache is None:
            cache = {}
            jobs._catalog_cache = cache
        cache.pop(fingerprint, None)
        cache[fingerprint] = copy.deepcopy(sources)
        while len(cache) > _CATALOG_CACHE_LIMIT:
            cache.pop(next(iter(cache)))
    return sources


def handle_motion_catalog(handler, jobs):
    if urlsplit(handler.path).path != '/api/motion/sources/status':
        return False

    def send(code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        handler.send_response(code)
        handler.send_header('Content-Type', 'application/json; charset=utf-8')
        handler.send_header('Content-Length', str(len(body)))
        handler.send_header('Cache-Control', 'no-store')
        handler.end_headers()
        handler.wfile.write(body)

    try:
        if handler.command != 'POST':
            send(405, {'error': '请提交目标模型和转换选项'})
            return True
        from .bff.security import same_origin
        origin = handler.headers.get('Origin')
        if origin is not None and not same_origin(origin, handler.headers.get('Host', '')):
            send(403, {'error': '只接受同源动作请求'})
            return True
        if handler.headers.get_content_type() != 'application/json':
            send(415, {'error': '动作请求必须为 application/json'})
            return True
        length = int(handler.headers.get('Content-Length', '0'))
        if not 0 < length <= 24 * 1024 * 1024:
            send(413, {'error': '模型文件不能超过 24 MB'})
            return True
        request = json.loads(handler.rfile.read(length))
        if not isinstance(request, dict):
            raise ValueError('动作请求必须是 JSON 对象')
        # Library identity always comes from the source tree.
        request.pop('bvh_text', None)
        send(200, {'sources': source_statuses(jobs, request)})
    except ImportError:
        send(503, {'error': '转换组件未就绪，请构建 bvh_broadcaster 并加载 ROS 工作区环境'})
    except Exception as exc:
        send(400, {'error': str(exc)})
    return True
