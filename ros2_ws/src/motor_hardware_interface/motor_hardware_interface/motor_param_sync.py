"""3 模板 + 37 电机差异参数解析、回读对账与一键同步（ExecuteConfig + W25 缓存）。"""

from dataclasses import dataclass
import argparse
import math
from pathlib import Path
import sys
import yaml

EXPECTED_MOTOR_COUNT = 37
MAX_BATCH = 16
MOTOR_ID_NONE = 0xFFFF

FIELD_INDEX = {
    'pos_min': 0,
    'pos_max': 1,
    'vel_limit': 2,
    'torque_limit': 3,
    'limit_protect': 4,
    'direction': 5,
    't_max': 6,
    'node_id': 7,
}
COMPARE_FIELDS = (
    'pos_min', 'pos_max', 'vel_limit', 'torque_limit',
    'limit_protect', 'direction', 't_max', 'node_id',
)
REQUIRED_TEMPLATE_FIELDS = (
    'pos_min', 'pos_max', 'vel_limit', 'torque_limit',
    'limit_protect', 'direction', 't_max',
)


@dataclass(frozen=True)
class ParamDiff:
    motor_id: int
    field: str
    expected: float
    actual: float


def load_motor_params(yaml_path):
    """解析 3 份型号模板 + 37 台电机差异覆盖，返回 {motor_id: resolved_dict}。"""
    data = yaml.safe_load(Path(yaml_path).read_text(encoding='utf-8')) or {}
    templates = data.get('templates') or {}
    motors = data.get('motors') or []
    if not isinstance(templates, dict) or not isinstance(motors, list):
        raise ValueError('motor_params.yaml 必须包含 templates 字典和 motors 列表')
    for tpl_name, tpl in templates.items():
        missing = [k for k in REQUIRED_TEMPLATE_FIELDS if k not in (tpl or {})]
        if missing:
            raise ValueError(f'模板 {tpl_name} 缺少字段: {missing}')

    resolved = {}
    for entry in motors:
        mid = int(entry['id'])
        if mid in resolved:
            raise ValueError(f'重复的 motor_id: {mid}')
        tpl_name = entry['template']
        if tpl_name not in templates:
            raise ValueError(f'未知模板 {tpl_name} (motor_id={mid})')
        merged = dict(templates[tpl_name])
        merged.update(entry.get('overrides') or {})
        merged['id'] = mid
        merged['joint'] = str(entry.get('joint', f'motor_{mid}'))
        merged['template'] = tpl_name
        merged.setdefault('node_id', mid)
        merged['limit_protect'] = bool(merged['limit_protect'])
        merged['direction'] = int(merged['direction'])
        merged['node_id'] = int(merged['node_id'])
        resolved[mid] = merged

    if len(resolved) != EXPECTED_MOTOR_COUNT or set(resolved.keys()) != set(range(1, EXPECTED_MOTOR_COUNT + 1)):
        raise ValueError(f'必须完整定义 1..{EXPECTED_MOTOR_COUNT} 共 {EXPECTED_MOTOR_COUNT} 台电机')
    return resolved


def _entry_actual_value(entry, field):
    """从 DeviceParamEntry 读取设备真值（优先 dev_*，未回读时退回 rec_*）。"""
    if field == 'limit_protect':
        dev_raw = getattr(entry, 'dev_safety', math.nan)
        rec_raw = getattr(entry, 'rec_safety', math.nan)
        raw = dev_raw if math.isfinite(dev_raw) else rec_raw
        if not math.isfinite(raw):
            return math.nan
        return 1.0 if int(round(raw)) == 0 else 0.0
    dev_val = float(getattr(entry, f'dev_{field}', math.nan))
    if math.isfinite(dev_val):
        return dev_val
    return float(getattr(entry, f'rec_{field}', math.nan))


def diff_device_info(desired, device_info_msg, tol=1e-3):
    """比对目标参数表与 MotorDeviceInfo，返回差异列表。"""
    actual_by_id = {int(e.motor_id): e for e in getattr(device_info_msg, 'params', [])}
    diffs = []
    for mid in sorted(desired):
        target = desired[mid]
        entry = actual_by_id.get(mid)
        if entry is None:
            diffs.append(ParamDiff(mid, 'missing', 1.0, 0.0))
            continue
        for field in COMPARE_FIELDS:
            expected = float(1 if target[field] else 0) if field == 'limit_protect' else float(target[field])
            actual = _entry_actual_value(entry, field)
            if not math.isfinite(actual) or abs(expected - actual) > tol:
                diffs.append(ParamDiff(mid, field, expected, actual))
    return diffs


def build_param_set_batches(desired, diffs, session_id, start_request_id=1, max_batch=MAX_BATCH):
    """将差异展开为稀疏 (motor_id, field, value) 目标，生成不超过 max_batch(16) 条/批的 ConfigRequest(PARAM_SET)。"""
    from builtin_interfaces.msg import Time
    from can_bridge_msgs.msg import ConfigRequest, ParamSetTarget
    from std_msgs.msg import Header

    fields_by_motor = {}
    for d in diffs:
        if d.field == 'missing':
            fields_by_motor[d.motor_id] = list(COMPARE_FIELDS)
        elif d.field in FIELD_INDEX:
            fields_by_motor.setdefault(d.motor_id, [])
            if d.field not in fields_by_motor[d.motor_id]:
                fields_by_motor[d.motor_id].append(d.field)

    targets = []
    for mid in sorted(fields_by_motor):
        cfg = desired[mid]
        for field in fields_by_motor[mid]:
            val = float(1.0 if cfg[field] else 0.0) if field == 'limit_protect' else float(cfg[field])
            targets.append(ParamSetTarget(
                motor_id=mid,
                field=FIELD_INDEX[field],
                value=val,
            ))

    batches = []
    req_id = int(start_request_id)
    for idx in range(0, len(targets), max_batch):
        chunk = targets[idx:idx + max_batch]
        batches.append(ConfigRequest(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=''),
            session_id=int(session_id),
            request_id=req_id,
            kind=ConfigRequest.CONFIG_KIND_PARAM_SET,
            count=len(chunk),
            targets=chunk,
            command=0,
            motor_id=MOTOR_ID_NONE,
        ))
        req_id += 1
    return batches


def build_system_command_request(command, session_id, request_id, motor_id=MOTOR_ID_NONE):
    """构造单条 ConfigRequest(SYSTEM_COMMAND)。"""
    from builtin_interfaces.msg import Time
    from can_bridge_msgs.msg import ConfigRequest
    from std_msgs.msg import Header

    return ConfigRequest(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=''),
        session_id=int(session_id),
        request_id=int(request_id),
        kind=ConfigRequest.CONFIG_KIND_SYSTEM_COMMAND,
        count=0,
        targets=[],
        command=int(command),
        motor_id=int(motor_id),
    )


def _call_sync(node, client, request, timeout_s=5.0):
    import rclpy
    fut = client.call_async(request)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
    if not fut.done() or fut.result() is None:
        raise TimeoutError(f'服务调用超时: {client.srv_name}')
    return fut.result()


def main(argv=None):  # pragma: no cover - ROS 2 CLI
    parser = argparse.ArgumentParser(description='37 轴参数模板对账与 W25 一键同步工具')
    parser.add_argument('--config', required=True, help='motor_params.yaml 路径')
    parser.add_argument('--check', action='store_true', help='仅回读 /motor/get_device_info 并对账')
    parser.add_argument('--sync', action='store_true', help='下发差异参数并执行同步')
    parser.add_argument('--save-device', action='store_true', help='同步后对变更电机发送 COMMAND_SAVE_DEVICE')
    parser.add_argument('--sync-w25', action='store_true', help='同步后发送 COMMAND_SYNC_W25 落盘 W25Q64')
    args = parser.parse_args(argv)
    if not args.check and not args.sync:
        args.check = True

    desired = load_motor_params(args.config)

    import rclpy
    from rclpy.node import Node
    from can_bridge_msgs.msg import SystemCommand
    from can_bridge_msgs.srv import ExecuteConfig, GetDeviceInfo

    rclpy.init()
    node = Node('motor_param_sync')
    try:
        info_cli = node.create_client(GetDeviceInfo, '/motor/get_device_info')
        cfg_cli = node.create_client(ExecuteConfig, '/motor/execute_config')
        if not info_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/motor/get_device_info 服务不可用')
        info_res = _call_sync(node, info_cli, GetDeviceInfo.Request(motor_id=0))
        info = info_res.info
        diffs = diff_device_info(desired, info)
        if not diffs:
            print(f'[OK] 37 台电机参数与 {args.config} 完全一致 (session_id={info.session_id})')
            return 0
        print(f'[DIFF] 发现 {len(diffs)} 处参数差异:')
        for d in diffs:
            print(f'  motor {d.motor_id:2d} {d.field}: expected={d.expected} actual={d.actual}')
        if not args.sync:
            return 2

        if not cfg_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/motor/execute_config 服务不可用')
        req_id = 1
        batches = build_param_set_batches(desired, diffs, info.session_id, start_request_id=req_id)
        for batch in batches:
            res = _call_sync(node, cfg_cli, ExecuteConfig.Request(request=batch, timeout_ms=5000))
            print(f'[PARAM_SET] req={batch.request_id} count={batch.count} status={res.result.status} err={res.result.error_code} ok={res.result.completed_targets}')
            req_id += 1
        if args.save_device:
            for mid in sorted({d.motor_id for d in diffs}):
                req = build_system_command_request(SystemCommand.COMMAND_SAVE_DEVICE, info.session_id, req_id, mid)
                res = _call_sync(node, cfg_cli, ExecuteConfig.Request(request=req, timeout_ms=5000))
                print(f'[SAVE_DEVICE] motor={mid} status={res.result.status}')
                req_id += 1
        if args.sync_w25:
            req = build_system_command_request(SystemCommand.COMMAND_SYNC_W25, info.session_id, req_id, MOTOR_ID_NONE)
            res = _call_sync(node, cfg_cli, ExecuteConfig.Request(request=req, timeout_ms=5000))
            print(f'[SYNC_W25] status={res.result.status} err={res.result.error_code} ok={res.result.completed_targets}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
