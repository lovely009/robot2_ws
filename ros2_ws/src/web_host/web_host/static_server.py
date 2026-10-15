"""web_host 的静态文件服务：页面资产、模型几何挂载与文件域 API。

进程管理（仿真 / Agent / 电机后端）不在这里——那是 bench_manager 的 /bench/*
服务；本进程只做只读文件与静态资产，前端经 rosbridge + 本 HTTP 服务两个通用面
使用。
"""

import json
import os
import signal
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

import rclpy
from rclpy.node import Node

from .actions import list_actions, read_action, save_action
from .models import (enumerate_arm_models, motor_library,
                     simulation_model_joints)


# 动作库导入：单个 JSON 载荷可达数百 KB（motion.json 2868 点）
_MAX_UPLOAD_BYTES = 8 * 1024 * 1024


def main(args=None):
    rclpy.init(args=args)
    # rclpy.init 会装它自己的 SIGINT handler（期待 rclpy.spin 消费）；本进程的
    # 主循环是 serve_forever，不恢复默认 handler 的话 SIGINT 被吞——ros2 launch
    # 得等满 5+10 秒升级到 SIGKILL，finally 里的清理全部跳过。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = Node('web_static_server')
    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8080)
    node.declare_parameter('web_root', '')
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
    # 目录）。取不到 → None，models.py 的枚举/映射回落到 robot_sim 安装树。
    models_dir = None
    if model_dir and os.path.isdir(os.path.join(model_dir, 'models')):
        models_dir = os.path.join(model_dir, 'models')
    node.get_logger().info(
        '模型挂载：%s -> %s；模型枚举根：%s'
        % (prefix or '(无)', model_dir or '(无)', models_dir or '(robot_sim 安装树)'))

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
            if path == '/api/model-joints':
                # 臂视图的 motor_id → 关节名 映射**从模型配置取**（单一来源），
                # 不再由前端硬编码。此前前端写死 half_body_robot 的 18 台，
                # 换模型就静默失效（关节角恒 0、小球点不中，像"没注册"）。
                requested = (parse_qs(
                    urlsplit(self.path).query).get('model') or [''])[0]
                try:
                    joints = simulation_model_joints(requested, models_dir)
                except ValueError as exc:
                    # 明确 404 + 原因：前端据此告警，而不是悄悄退化成错映射
                    self._send_json(404, {'error': str(exc)})
                    return
                self._send_json(200, {
                    'model': requested,
                    # JSON 对象的键只能是字符串；前端解析回 int
                    'joints': {str(mid): joint
                               for mid, joint in joints.items()},
                })
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
            path = urlsplit(self.path).path
            if path not in {'/api/actions'}:
                self._send_json(404, {'error': 'unknown API endpoint'})
                return

            try:
                content_length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                self._send_json(400, {'error': 'invalid Content-Length'})
                return
            limit = _MAX_UPLOAD_BYTES
            if content_length < 0 or content_length > limit:
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
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
