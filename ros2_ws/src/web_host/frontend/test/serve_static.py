"""浏览器测试只读静态素材服务；没有 ROS 节点或设备依赖。"""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

package = Path(__file__).resolve().parents[2]
model_root = package.parent / 'robot_sim'


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

    def log_message(self, *args):
        pass


ThreadingHTTPServer(('127.0.0.1', 8092), Handler).serve_forever()
