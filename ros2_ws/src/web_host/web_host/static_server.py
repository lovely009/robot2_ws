import json
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import rclpy
from rclpy.node import Node

from .agent_manager import AgentProcessManager, enumerate_serial_ports


_MAX_API_BODY_BYTES = 4096


def main(args=None):
    rclpy.init(args=args)
    node = Node('web_static_server')
    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8080)
    node.declare_parameter('web_root', '')
    # 额外挂载：把某个包的 share 目录挂到一个 URL 前缀下。臂视图要取 URDF 与
    # 约 20 MB 的 STL 网格——那批量既不进 web/、也不该进包，而 setuptools 的
    # symlink-install 又处理不了**目录**符号链接（会把整目录当文件 copy，报
    # "not a regular file"），所以走挂载。空 = 不挂载：静态页面照常，
    # 臂视图自己会报加载失败并说明原因。
    node.declare_parameter('model_url_prefix', '')
    node.declare_parameter('model_dir', '')
    host = node.get_parameter('host').value
    port = int(node.get_parameter('port').value)
    web_root = node.get_parameter('web_root').value
    if not web_root:
        web_root = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
    web_root = os.path.abspath(web_root)

    prefix = node.get_parameter('model_url_prefix').value.rstrip('/')
    model_dir = node.get_parameter('model_dir').value
    if model_dir and os.path.isdir(model_dir):
        model_dir = os.path.abspath(model_dir)
    else:
        if model_dir:
            node.get_logger().warn(
                'model_dir 不存在，臂视图几何将 404：%s' % model_dir)
        model_dir = None
    if not model_dir:
        prefix = ''
    node.get_logger().info(
        '模型挂载：%s -> %s' % (prefix or '(无)', model_dir or '(无)'))

    agent_manager = AgentProcessManager()

    class Handler(SimpleHTTPRequestHandler):

        def __init__(self, *a, **kw):
            super().__init__(*a, directory=web_root, **kw)

        def translate_path(self, path):
            # 命中挂载前缀时临时换根，其余一律走 web_root。每个请求都是新的
            # Handler 实例，改 self.directory 不会串到别的线程。
            if prefix and (path == prefix or path.startswith(prefix + '/')):
                saved, self.directory = self.directory, model_dir
                try:
                    return SimpleHTTPRequestHandler.translate_path(
                        self, path[len(prefix):] or '/')
                finally:
                    self.directory = saved
            return SimpleHTTPRequestHandler.translate_path(self, path)

        def _send_json(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/serial-ports':
                ports = []
                for port in enumerate_serial_ports():
                    ports.append({
                        **port,
                        'accessible': os.access(
                            port['path'], os.R_OK | os.W_OK),
                    })
                self._send_json(200, {'ports': ports})
                return
            if path == '/api/agent/status':
                self._send_json(200, agent_manager.status())
                return
            super().do_GET()

        def do_POST(self):
            path = urlsplit(self.path).path
            if path == '/api/agent/stop':
                try:
                    self._send_json(200, agent_manager.stop())
                except OSError as exc:
                    self._send_json(
                        500, {'error': 'could not stop Agent: %s' % exc})
                return
            if path != '/api/agent/start':
                self._send_json(404, {'error': 'unknown API endpoint'})
                return

            try:
                content_length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                self._send_json(400, {'error': 'invalid Content-Length'})
                return
            if content_length < 0 or content_length > _MAX_API_BODY_BYTES:
                self._send_json(413, {'error': 'request body too large'})
                return
            try:
                request = json.loads(self.rfile.read(content_length))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json(400, {'error': 'invalid JSON request'})
                return
            device = (request.get('device')
                      if isinstance(request, dict) else None)
            if not isinstance(device, str) or not device:
                self._send_json(400, {'error': 'device is required'})
                return

            try:
                self._send_json(200, agent_manager.start(device))
            except ValueError as exc:
                self._send_json(400, {'error': str(exc)})
            except PermissionError as exc:
                self._send_json(403, {'error': str(exc)})
            except RuntimeError as exc:
                self._send_json(409, {
                    'error': str(exc), 'status': agent_manager.status(),
                })
            except OSError as exc:
                self._send_json(
                    500, {'error': 'could not start Agent: %s' % exc})

        def log_message(self, fmt, *args):
            node.get_logger().info(
                '%s %s' % (self.address_string(), fmt % args))

    server = ThreadingHTTPServer((host, port), Handler)
    node.get_logger().info(
        'Serving %s at http://%s:%d' % (web_root, host, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        agent_manager.stop()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
