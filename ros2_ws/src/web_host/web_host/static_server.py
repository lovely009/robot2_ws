"""页面资产、运行模式、模型与动作文件域；设备访问统一经 MCU。

HTTP 服务不打开 CAN 或串口。运动意图通过受限 rosbridge 入口进入整机监督器。
"""

import json
import os
import signal
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

import rclpy
from rclpy.node import Node

from .actions import actions_directory, list_actions, read_action, save_action
from .models import (enumerate_arm_models, model_transmissions,
                     motor_library)
from .motion import MotionJobs, handle_motion


# 动作库导入：单个 JSON 载荷可达数百 KB（motion.json 2868 点）
_MAX_UPLOAD_BYTES = 8 * 1024 * 1024


def _same_origin(origin, host):
    """比较本 HTTP 服务的源；默认端口等价，空或畸形 Origin 不被信任。"""
    try:
        source = urlsplit(origin)
        target = urlsplit('http://' + host)
        return (
            source.scheme == target.scheme == 'http'
            and source.hostname is not None
            and source.hostname == target.hostname
            and (source.port if source.port is not None else 80)
            == (target.port if target.port is not None else 80)
            and source.username is None and source.password is None
            and target.username is None and target.password is None
            and not source.path and not source.query and not source.fragment
            and not target.path and not target.query and not target.fragment
        )
    except (TypeError, ValueError):
        return False


def main(args=None):
    rclpy.init(args=args)
    # 主循环为 serve_forever，不消费 rclpy 信号；SIGINT/SIGTERM 均转为
    # KeyboardInterrupt，确保执行 finally 清理，不能用 SIG_DFL 直接终止。
    signal.signal(signal.SIGINT, signal.default_int_handler)

    def _sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _sigterm)
    node = Node('web_static_server')
    node.declare_parameter('host', '127.0.0.1')
    node.declare_parameter('port', 8080)
    node.declare_parameter('web_root', '')
    node.declare_parameter('control_mode', 'bench')
    node.declare_parameter('robot_model', '')
    node.declare_parameter('rosbridge_port', 0)
    control_mode = node.get_parameter('control_mode').value
    if control_mode not in ('bench', 'readonly'):
        raise ValueError('control_mode must be bench or readonly')
    runtime_config = {'control_mode': control_mode,
                      'robot_model': node.get_parameter('robot_model').value}
    if node.get_parameter('rosbridge_port').value:
        runtime_config['rosbridge_port'] = int(node.get_parameter('rosbridge_port').value)
    # 额外挂载：把模型包根（含 models/）挂到一个 URL 前缀下。臂视图要取 URDF 与
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
    # 模型枚举根 = 挂载源的 models/ 子目录（model_dir 语义：模型包根，可指源码
    # 目录）。取不到 → None，models.py 的枚举/映射回落到 robot_description 安装树。
    models_dir = None
    if model_dir and os.path.isdir(os.path.join(model_dir, 'models')):
        models_dir = os.path.join(model_dir, 'models')
    node.get_logger().info(
        '模型挂载：%s -> %s；模型枚举根：%s'
        % (prefix or '(无)', model_dir or '(无)', models_dir or '(robot_description 安装树)'))

    # 本次服务启动的标识：写进 cookie，用来判断"这个浏览器还欠不欠一次缓存清理"
    # （见 Handler.end_headers 的长注释）。用启动时刻，每次重启都不同。
    build_token = str(int(time.time()))
    from pathlib import Path
    motion_jobs = MotionJobs(actions_dir=actions_directory(web_root),
                            profiles_dir=Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
                            / 'robot2/motion_mappings', models_root=models_dir)

    class Handler(SimpleHTTPRequestHandler):

        # 这些后缀是"页面本身"，一个旧的就是一次链接期失败；其余（模型几何）允许
        # 走重验证，别为了省事让 70 MB 的 STL 每次重下。
        _NO_STORE_SUFFIXES = ('.html', '.js', '.mjs', '.css', '.json')

        def __init__(self, *a, **kw):
            super().__init__(*a, directory=web_root, **kw)

        def _cookie(self, name):
            """取请求里某个 cookie 的值（没有则 None）。只服务于 web_ver 这一个自设标记。"""
            for part in self.headers.get('Cookie', '').split(';'):
                key, sep, value = part.strip().partition('=')
                if sep and key == name:
                    return value
            return None

        def end_headers(self):
            # 页面资产用 no-store，避免跨版本模块混用；该头无法清除浏览器已有缓存。
            # 文档响应以 cookie 记录每个浏览器的服务版本，重启后各清一次 HTTP 缓存，
            # 避免被探活请求消耗清理机会，或反复清掉大型 STL 缓存。
            # Clear-Site-Data 要求安全上下文（localhost 可用，局域网 IP 不可用）。
            # 模型几何用 no-cache 重验证，Last-Modified 命中时返回 304，无需重传。
            buffer = getattr(self, '_headers_buffer', None) or []
            path = urlsplit(self.path).path.lower()
            if b' 200 ' in b''.join(buffer[:1]) and (
                    path == '/' or path.endswith('.html')):
                if self._cookie('web_ver') != build_token:
                    self.send_header('Clear-Site-Data', '"cache"')
                    self.send_header(
                        'Set-Cookie',
                        f'web_ver={build_token}; Path=/; SameSite=Lax')
            if b'Cache-Control' not in b''.join(buffer):
                # `/` 也是文档（映射到 index.html）：契约测试按 id 比对的就是它，
                # 它必须与那批 JS 同代，所以一并按"不复用"处理。
                if path == '/' or path.endswith(self._NO_STORE_SUFFIXES):
                    self.send_header('Cache-Control', 'no-store')
                else:
                    self.send_header('Cache-Control', 'no-cache')
            super().end_headers()

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
            if handle_motion(self, motion_jobs):
                return
            path = urlsplit(self.path).path
            if path == '/api/runtime':
                self._send_json(200, runtime_config)
                return
            if path in ('/', '/index.html') and os.path.isfile(os.path.join(web_root, 'workbench', 'index.html')):
                query = urlsplit(self.path).query
                self.send_response(302)
                self.send_header('Location', '/workbench/' + ('?' + query if query else ''))
                self.end_headers()
                return
            if path == '/api/models':
                # 臂视图可切换的模型（有 urdf/<名>.urdf 的目录）
                self._send_json(
                    200, {'models': enumerate_arm_models(models_dir)})
                return
            if path == '/api/motor-specs':
                # 型号库（减速比等）：设备页据此把关节侧量折算成设备端原值
                # （cnt / mA / 电机端 RPM）。取不到返回空表，页面不显示该列。
                self._send_json(200, {'motor_types': motor_library()})
                return
            if path == '/api/model-transmissions':
                # 传动组是关节与电机的权威关联。
                requested = (parse_qs(
                    urlsplit(self.path).query).get('model') or [''])[0]
                try:
                    payload = model_transmissions(requested, models_dir)
                except ValueError as exc:
                    # 明确 404 + 原因：前端据此告警，而不是悄悄退化成错映射
                    self._send_json(404, {'error': str(exc)})
                    return
                self._send_json(200, payload)
                return
            if path == '/api/actions':
                # 动作库列表（动作屏「动作库播放」的数据源）
                self._send_json(200, {'actions': list_actions(web_root)})
                return
            if path.startswith('/api/actions/'):
                # 单个动作原文（保留文件原格式，前端自行解析）
                name = unquote(path[len('/api/actions/'):])
                try:
                    text = read_action(web_root, name)
                except FileNotFoundError:
                    self._send_json(404, {'error': '动作不存在'})
                    return
                except ValueError as exc:
                    self._send_json(400, {'error': str(exc)})
                    return
                body = text.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_GET()

        def do_POST(self):
            if handle_motion(self, motion_jobs):
                return
            path = urlsplit(self.path).path
            if path != '/api/actions':
                self._send_json(404, {'error': 'unknown API endpoint'})
                return
            # 所有写接口共用门禁：跨源网页不能通过 text/plain 简单 POST 绕过
            # 浏览器预检，悄悄覆盖动作库。无 Origin 的 JSON 命令行客户端仍可使用。
            origin = self.headers.get('Origin')
            if origin is not None and not _same_origin(
                    origin, self.headers.get('Host', '')):
                self._send_json(403, {'error': '只接受同源请求'})
                return
            if self.headers.get_content_type() != 'application/json':
                self._send_json(415, {'error': '请求必须为 application/json'})
                return

            try:
                content_length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                self._send_json(400, {'error': 'invalid Content-Length'})
                return
            if content_length < 0 or content_length > _MAX_UPLOAD_BYTES:
                self._send_json(413, {'error': 'request body too large'})
                return
            try:
                request = json.loads(self.rfile.read(content_length))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json(400, {'error': 'invalid JSON request'})
                return

            if path == '/api/actions':
                # 导入动作到内置库；同名默认拒绝，显式 overwrite 才覆盖
                name = request.get('name') if isinstance(request, dict) else None
                text = request.get('text') if isinstance(request, dict) else None
                overwrite = bool(
                    request.get('overwrite')) if isinstance(request, dict) else False
                try:
                    entry = save_action(web_root, name, text, overwrite=overwrite)
                except FileExistsError:
                    self._send_json(409, {
                        'error': '同名动作已存在；确认覆盖后重试',
                        'exists': True,
                    })
                    return
                except ValueError as exc:
                    self._send_json(400, {'error': str(exc)})
                    return
                except OSError as exc:
                    self._send_json(
                        500, {'error': '动作库写入失败：%s' % exc})
                    return
                self._send_json(200, {'ok': True, 'action': entry})
                return

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
        motion_jobs.close()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
