"""浏览器测试素材与临时动作任务服务；没有 ROS 节点或设备依赖。"""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from urllib.parse import unquote, urlsplit

package = Path(__file__).resolve().parents[2]
model_root = package.parent / 'robot_description'
sys.path.insert(0, str(package))
from web_host.motion import MotionJobs, handle_motion
motion_jobs = MotionJobs(package / 'action_library')


class Handler(SimpleHTTPRequestHandler):
    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(package / 'web'), **kwargs)

    def translate_path(self, path):
        requested = unquote(urlsplit(path).path)
        if requested.startswith('/robot_sim/'):
            target = (model_root / requested[len('/robot_sim/'):]).resolve()
            if model_root.resolve() in target.parents:
                return str(target)
        return super().translate_path(path)

    def do_GET(self):
        request = urlsplit(self.path)
        if request.path == '/api/models':
            models = sorted(p.name for p in (model_root / 'models').iterdir()
                            if p.is_dir() and (p / 'urdf' / f'{p.name}.urdf').is_file())
            body = json.dumps({'models': models}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if request.path in ('/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/workbench/' + ('?' + request.query if request.query else ''))
            self.end_headers()
            return
        if not handle_motion(self, motion_jobs):
            super().do_GET()

    def do_POST(self):
        if not handle_motion(self, motion_jobs):
            self.send_error(404)

    def log_message(self, *args):
        pass


try:
    ThreadingHTTPServer(('127.0.0.1', 8092), Handler).serve_forever()
finally:
    motion_jobs.close()
