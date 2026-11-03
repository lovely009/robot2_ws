def main(args=None):
    # 文件域工具可独立导入；只有启动 HTTP/ROS 服务时才加载运行期依赖。
    from .static_server import main as serve
    return serve(args=args)

__all__ = ['main']
