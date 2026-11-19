"""Read-only BVH saved-state inspection using the normal conversion identities."""

import json
from urllib.parse import urlsplit

from .motion import MotionJobs
from .motion_cache import result_key


class _CatalogJobs(MotionJobs):
    def start(self, key, mapping, remember=False):
        # Catalog discovery prepares mappings but never schedules conversion.
        return self.status(key)


def source_statuses(jobs, request):
    request = {key: value for key, value in request.items() if key != 'bvh_text'}
    names = jobs.sources()
    if jobs.cache is None:
        return [{'name': name, 'saved': False} for name in names]
    inspector = _CatalogJobs(jobs.actions_dir, jobs.profiles_dir, jobs.models_root,
                             jobs.cache.directory)
    try:
        sources = []
        for name in names:
            try:
                status = inspector.create({**request, 'bvh_source': name,
                                           'review_mapping': False})
                job = inspector.jobs[status['id']]
                mapping = status['mapping']
                saved = bool(status['automatic_mapping'] and mapping
                             and inspector.cache.load(result_key(job, mapping), job['directory']))
                sources.append({'name': name, 'saved': saved})
            except ImportError:
                raise
            except Exception as exc:
                sources.append({'name': name, 'saved': False, 'error': str(exc)})
        return sources
    finally:
        inspector.close()


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
        origin = handler.headers.get('Origin')
        if origin and urlsplit(origin).netloc != handler.headers.get('Host'):
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
