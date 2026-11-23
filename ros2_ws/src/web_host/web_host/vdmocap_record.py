"""录制一段 DreamsCapStudio 身体广播并提交给工作台动作库。"""

import argparse
from contextlib import contextmanager
from ipaddress import IPv4Address
import json
import math
from pathlib import Path
import runpy
import sys
import tempfile
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from .vdmocap_import import IMPORT_PATH


DEFAULT_SDK_DIR = Path.home() / '文档/DataRead_Python_Linux_SDK/DataRead_Python_Demo'


def sdk_frame(data):
    return {'index': data.frameIndex, 'frequency': data.frequency,
            'positions': [list(point) for point in data.position_body],
            'quaternions': [list(q) for q in data.quaternion_body]}


def read_frames(sdk, peer, seconds):
    from .vdmocap import MAX_FRAMES

    data = sdk.MocapData()
    frames = []
    previous = None
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if sdk.udp_recv_mocap_data(*peer, data) and data.isUpdate and data.frameIndex != previous:
            index, rate = data.frameIndex, data.frequency
            if rate <= 0 or seconds * rate > MAX_FRAMES:
                raise ValueError(f'采样率 {rate} Hz 无效或录制长度超过 {MAX_FRAMES} 帧')
            if previous is not None:
                step = (index - previous) % 2**32
                if step > 1 + rate // 4 or rate != frames[-1]['frequency']:
                    if len(frames) >= 2:
                        print('播放跳帧、变速或回到开头，保留跳变前的连续片段。', flush=True)
                        break
                    # A player paused at the end may rewind when Play is pressed.
                    frames.clear()
            frames.append(sdk_frame(data))
            previous = index
            if len(frames) >= 2:
                if len(frames) == 2:
                    print(f'已收到连续帧，正在按 {rate} Hz 录制。', flush=True)
                if (index - frames[0]['index']) % 2**32 + 1 >= max(2, round(seconds * rate)):
                    break
                deadline = time.monotonic() + 2
        time.sleep(.002)
    if len(frames) < 2:
        raise ValueError('等待 30 秒仍未收到连续帧，请确认广播地址并播放动作')
    return frames


@contextmanager
def body_connection(sdk_dir, ip, port):
    # Use the vendor demo's joint order and T pose for the very same SDK connection.
    demo_path = sdk_dir.resolve() / 'main.py'
    sys.path.insert(0, str(demo_path.parent))
    try:
        demo = runpy.run_path(str(demo_path))
        import vdmocapsdk_dataread as sdk
    finally:
        sys.path.pop(0)
    rest = demo['global_initial_position_body']
    peer = (0, str(ip), port)
    if not sdk.udp_open(0, 0):
        raise OSError('无法打开本地 UDP 接收端口')
    try:
        if not sdk.udp_set_position_in_initial_tpose(*peer, sdk.WorldSpace.WS_Geo, rest,
                demo['global_initial_position_hand_right'], demo['global_initial_position_hand_left']):
            raise OSError('SDK 拒绝设置地理坐标系 T 姿')
        if not sdk.udp_send_request_connect(*peer):
            raise OSError(f'无法连接 {ip}:{port}/UDP')
        yield sdk, peer, demo['NAMES_JOINT_BODY'], rest
    finally:
        try:
            sdk.udp_remove(*peer)
        finally:
            sdk.udp_close(0)


def record(sdk_dir, ip, port, seconds):
    with body_connection(sdk_dir, ip, port) as (sdk, peer, names, rest):
        print(f'等待 {ip}:{port}/UDP 的播放帧…', flush=True)
        frames = read_frames(sdk, peer, seconds)
    return {'world_space': 'WS_Geo', 'joints': names, 'rest': rest, 'frames': frames}


def submit(url, name, model, capture):
    payload = json.dumps({'name': name, 'model': model, 'capture': capture},
                         separators=(',', ':'), allow_nan=False).encode()
    request = Request(url.rstrip('/') + IMPORT_PATH, data=payload,
                      headers={'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=1800) as response:
            return json.load(response)
    except HTTPError as exc:
        body = exc.read().decode(errors='replace')
        try:
            body = json.loads(body)['error']
        except (KeyError, TypeError, ValueError):
            pass
        raise OSError(f'HTTP {exc.code}：{body}') from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='动作库中的 BVH 文件名')
    parser.add_argument('--model', default='full_body_robot', help='工作台模型名')
    parser.add_argument('--url', default='http://127.0.0.1:8080', help='工作台 HTTP 地址')
    parser.add_argument('--input', type=Path, help='提交已有的录制 JSON')
    parser.add_argument('--sdk-dir', type=Path, default=DEFAULT_SDK_DIR)
    parser.add_argument('--ip', type=IPv4Address, default='192.168.1.9')
    parser.add_argument('--port', type=int, default=7000)
    parser.add_argument('--seconds', type=float, default=5, help='从播放开始录制的秒数')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('端口须为 1–65535，录制秒数须为正数')
    try:
        capture = (json.loads(args.input.read_text(encoding='utf-8')) if args.input
                   else record(args.sdk_dir, args.ip, args.port, args.seconds))
        try:
            result = submit(args.url, args.name, args.model, capture)
        except (OSError, ValueError):
            if not args.input:
                with tempfile.NamedTemporaryFile(mode='w', prefix='vdmocap-', suffix='.json',
                                                 delete=False, encoding='utf-8') as backup:
                    json.dump(capture, backup, separators=(',', ':'))
                print(f'录制已保留，可用 --input {backup.name} 重试。', file=sys.stderr)
            raise
    except (OSError, ImportError, ValueError) as exc:
        parser.exit(1, f'{exc}\n')
    print(f"已保存 {result['name']}，模型 {result['model']}，{result['frames']} 帧 / "
          f"{result['frequency']} Hz，匹配 {result['matched_targets']} 个部位，"
          f"{result['joint_count']} 个机器人关节。")
