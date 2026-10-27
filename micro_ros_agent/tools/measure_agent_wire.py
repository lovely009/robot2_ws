#!/usr/bin/env python3
"""生成最大合法实例，测量真实 CDR；需 source 本工作区 Agent install。"""
import json
from pathlib import Path
import re

from rclpy.serialization import serialize_message
from rosidl_parser.definition import (
    AbstractSequence, AbstractString, Array, BasicType, NamespacedType,
)
from rosidl_runtime_py.utilities import get_message


def maximum_message(cls):
    message = cls()
    for field, slot in zip(message.get_fields_and_field_types(), message.SLOT_TYPES):
        if isinstance(slot, AbstractSequence):
            if not slot.has_maximum_size():
                raise ValueError(f'{cls.__name__}.{field}: 无界序列')
            setattr(message, field, [_value(slot.value_type)
                                    for _ in range(slot.maximum_size)])
        elif isinstance(slot, Array):
            setattr(message, field, [_value(slot.value_type) for _ in range(slot.size)])
        elif isinstance(slot, (NamespacedType, AbstractString)):
            setattr(message, field, _value(slot))
    return message


def _value(slot):
    if isinstance(slot, NamespacedType):
        return maximum_message(get_message('/'.join((*slot.namespaces, slot.name))))
    if isinstance(slot, AbstractString):
        # 标准 Header 的 frame_id 虽无界，固件只使用固定 imu_link。
        return 'x' * slot.maximum_size if slot.has_maximum_size() else 'imu_link'
    if isinstance(slot, BasicType):
        if slot.typename == 'boolean':
            return False
        if slot.typename in ('float', 'double', 'long double'):
            return 0.0
        return 0
    raise ValueError(f'不支持的类型: {slot}')


def wire_budget():
    root = Path(__file__).resolve().parents[2]
    library = root / 'micro_ros_firmware/.pio/libdeps/stm32h723vgt6/micro_ros_platformio'
    config = (library / 'libmicroros/include/uxr/client/config.h').read_text()
    mtu = int(re.search(r'#define\s+UXR_CONFIG_CUSTOM_TRANSPORT_MTU\s+(\d+)', config)[1])
    meta = json.loads((root / 'micro_ros_firmware/microros_user.meta').read_text())
    args = meta['names']['rmw_microxrcedds']['cmake-args']
    history = int(next(a.split('=')[-1] for a in args
                       if a.startswith('-DRMW_UXRCE_STREAM_HISTORY=')))
    # 可靠通道按每片扣 12 B 保守预算，不把全部历史当净 CDR 载荷。
    return mtu - 12, (mtu - 12) * history


def measurements():
    types = ('AgentCommand', 'AgentMotion', 'AgentTrajectoryRequest',
             'AgentConfigRequest', 'AgentStatus', 'AgentInfo')
    values = {name: len(serialize_message(maximum_message(
        get_message('can_bridge_msgs/msg/' + name)))) for name in types}
    for name in ('Imu', 'Temperature'):
        values[name] = len(serialize_message(maximum_message(
            get_message('sensor_msgs/msg/' + name))))
    return values


if __name__ == '__main__':
    print(json.dumps({'cdr_bytes': measurements(),
                      'budgets': dict(zip(('best_effort', 'reliable'), wire_budget()))},
                     indent=2))
