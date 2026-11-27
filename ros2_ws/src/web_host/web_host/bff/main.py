"""CLI and ROS 2 entry point for the single-port FastAPI + rclpy BFF server."""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import uvicorn

from .app import create_app


def _default_web_root() -> Path:
    return Path(__file__).resolve().parents[2] / 'web'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Run the Robot Workbench BFF server.')
    parser.add_argument('--host', default=os.environ.get('WEB_HOST', '0.0.0.0'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('WEB_PORT', '8080')))
    parser.add_argument('--web-root', default=str(_default_web_root()))
    parser.add_argument('--model-dir', default='')
    parser.add_argument('--model-url-prefix', default='/robot_sim')
    parser.add_argument('--control-mode', choices=('bench', 'readonly'), default='bench')
    parser.add_argument('--robot-model', default='')
    parser.add_argument('--pin', default=None)
    parser.add_argument('--lease-ttl', type=float, default=3.0)
    parser.add_argument('--broadcast-hz', type=float, default=30.0)
    args, _unknown = parser.parse_known_args(argv)

    app = create_app(
        web_root=args.web_root,
        model_dir=args.model_dir or None,
        model_url_prefix=args.model_url_prefix,
        control_mode=args.control_mode,
        robot_model=args.robot_model,
        lease_ttl_s=args.lease_ttl,
        pin=args.pin,
        broadcast_hz=args.broadcast_hz,
    )
    uvicorn.run(app, host=args.host, port=args.port, log_level='info')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
