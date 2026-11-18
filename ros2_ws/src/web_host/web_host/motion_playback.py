"""全分辨率动作轨迹接口；只导出转换结果的数值，不创建 ROS 数据。"""

import io
import json
from urllib.parse import urlsplit

_MAX_DURATION_MILLISECONDS = 2 ** 32


def _load_trajectory(raw):
    """Read and validate motion.npz as JSON-compatible playback data."""
    import numpy as np

    with np.load(io.BytesIO(raw), allow_pickle=False) as motion:
        names = motion['joint_names'].tolist()
        positions = np.asarray(motion['positions'])
        frame_time = motion['frame_time'].item()

    if not names or any(type(name) is not str or not name for name in names):
        raise ValueError('关节名称必须是非空字符串')
    if len(set(names)) != len(names):
        raise ValueError('关节名称不能重复')
    if positions.ndim != 2 or len(positions) < 1 or positions.shape[1] != len(names):
        raise ValueError('关节位置数据形状无效')
    if not np.isfinite(positions).all():
        raise ValueError('关节位置数据必须有限')
    if isinstance(frame_time, bool) or not np.isfinite(frame_time) or frame_time <= 0:
        raise ValueError('帧时间必须为正有限值')
    total_ms = (len(positions) - 1) * frame_time * 1000
    if not np.isfinite(total_ms) or total_ms >= _MAX_DURATION_MILLISECONDS:
        raise ValueError('轨迹总时长超过 ROS duration 毫秒表示范围')
    return {'joint_names': names, 'positions': positions.tolist(),
            'frame_time': float(frame_time)}


def handle_motion_playback(handler, jobs):
    """Mount GET /api/motion/jobs/<id>/trajectory for completed jobs."""
    parts = urlsplit(handler.path).path.strip('/').split('/')
    if handler.command != 'GET' or len(parts) != 5 or parts[:3] != ['api', 'motion', 'jobs'] \
            or parts[4] != 'trajectory':
        return False

    def send(status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        handler.send_response(status)
        handler.send_header('Content-Type', 'application/json; charset=utf-8')
        handler.send_header('Content-Length', str(len(body)))
        handler.send_header('Cache-Control', 'no-store')
        handler.end_headers()
        handler.wfile.write(body)

    try:
        raw = jobs.result(parts[3], 'motion.npz')
        send(200, _load_trajectory(raw))
    except FileNotFoundError as exc:
        send(404, {'error': str(exc)})
    except Exception as exc:
        send(400, {'error': str(exc)})
    return True
