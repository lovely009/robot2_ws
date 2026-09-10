import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('web_static_server')
    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8080)
    node.declare_parameter('web_root', '')
    host = node.get_parameter('host').value
    port = int(node.get_parameter('port').value)
    web_root = node.get_parameter('web_root').value
    if not web_root:
        web_root = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
    web_root = os.path.abspath(web_root)

    class Handler(SimpleHTTPRequestHandler):

        def __init__(self, *a, **kw):
            super().__init__(*a, directory=web_root, **kw)

        def log_message(self, fmt, *args):
            node.get_logger().info('%s %s' % (self.address_string(), fmt % args))

    server = ThreadingHTTPServer((host, port), Handler)
    node.get_logger().info('Serving %s at http://%s:%d' % (web_root, host, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
