"""DDS fixture around production native firmware; never opens USB/CAN.

Boot inventory and commissioned calibration are fictional test facts. Motion,
policy/lease barriers, CAN encoding/routing and plant feedback use native code.
This is software-in-the-loop evidence, not physical timing acceptance.
"""
import ctypes as C
import time
import json
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from can_bridge_msgs.msg import (AgentCommand, AgentConfigRequest, AgentConfigResult,
    AgentDeviceInfo, AgentInfo, AgentMotion, AgentMotorStatus, AgentStatus, AgentWireContract)

N = 37


class NativeDevice(Node):
    def __init__(self, library, context, *, models=None, buses=None):
        super().__init__('native_firmware_sil_only', context=context)
        self._native_lock=threading.RLock()
        self.lib = C.CDLL(str(library))
        self.lib.sil_advance.argtypes = [C.c_uint32]
        self.lib.sil_lease.argtypes = [C.c_uint64, C.c_uint32, C.c_uint32, C.c_int]
        self.lib.sil_command.argtypes = [C.c_uint64, C.c_uint32, C.c_uint32, C.c_uint,
            C.POINTER(C.c_uint8), C.POINTER(C.c_int16), C.POINTER(C.c_int16), C.POINTER(C.c_int16)]
        self.lib.sil_policy.argtypes = [C.c_uint64, C.c_uint32, C.c_uint32, C.c_uint32, C.c_uint,
            C.POINTER(C.c_uint16), C.POINTER(C.c_uint8), C.POINTER(C.c_float), C.POINTER(C.c_float)]
        self.lib.sil_snapshot.argtypes = [C.POINTER(C.c_float), C.POINTER(C.c_uint32), C.POINTER(C.c_uint64)]
        self.lib.sil_enabled_mask.restype = C.c_uint64
        self.models = models if models is not None else ['cerb_r48_101'] * N
        self.count = len(self.models)
        self.buses = buses if buses is not None else [i % 3 for i in range(self.count)]
        if (not 0 < self.count <= N or len(self.buses) != self.count
                or any(not isinstance(bus, int) or not 0 <= bus < 3 for bus in self.buses)):
            raise ValueError('invalid native motor inventory')
        self.lib.sil_init()
        if models is not None:
            self.lib.sil_init_model.argtypes = [C.c_uint, C.POINTER(C.c_char_p), C.POINTER(C.c_uint8)]
            if not self.lib.sil_init_model(self.count, (C.c_char_p*self.count)(*[name.encode() for name in self.models]),
                    (C.c_uint8*self.count)(*self.buses)):
                raise ValueError('unsupported model or bus in native inventory')
        self.values, self.states, self.head = (C.c_float*(self.count*3))(), (C.c_uint32*(self.count*10))(), (C.c_uint64*8)()
        self.started = time.monotonic()
        self.sequence = 0
        self.commands, self.results, self.policies = [], [], []
        self._acknowledged=set()
        self.receipts=[]
        self.trace=[]
        self._advance_at=time.monotonic_ns()
        self.drop_commands = False
        self.drop_authority = False
        self.suppress_status = False
        self.grants=[]
        self.configs=[]
        self.status_pub = self.create_publisher(AgentStatus, AgentWireContract.TOPIC_STATUS, 10)
        self.info_pub = self.create_publisher(AgentInfo, AgentWireContract.TOPIC_INFO, 10)
        stream = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.motion_pub = self.create_publisher(AgentMotion, AgentWireContract.TOPIC_MOTION, stream)
        self.create_subscription(AgentCommand, AgentWireContract.TOPIC_COMMAND, self.command, stream)
        self.create_subscription(AgentConfigRequest, AgentWireContract.TOPIC_CONFIG, self.config, 10)
        self.create_timer(.001, self.advance)
        self.create_timer(.02, self.publish_state)

    def advance(self):
        current=time.monotonic_ns()
        if current-self._advance_at > 3_000_000:
            self.record('native_service_gap', gap_ms=(current-self._advance_at)/1e6)
        self._advance_at=current
        with self._native_lock:
            self.lib.sil_advance(1000+int((time.monotonic()-self.started)*1000))
            self.lib.sil_snapshot(self.values, self.states, self.head)

    def enabled_mask(self):
        with self._native_lock: return self.lib.sil_enabled_mask()

    def positions(self):
        with self._native_lock: return [float(self.values[i*3]) for i in range(self.count)]

    def record(self, event, **fields):
        if len(self.trace)<50000:
            self.trace.append(dict(event=event,mono_ns=time.monotonic_ns(),wall_ns=time.time_ns(),**fields))

    def save_trace(self, path):
        path.write_text(''.join(json.dumps(row)+'\n' for row in self.trace))

    def command(self, message):
        if message.schema_version!=AgentWireContract.SCHEMA_VERSION: return
        if message.mode==AgentCommand.STOP_ALL:
            if not message.count and not any((message.motor_ids,message.position_q,message.velocity_q,message.effort_q)):
                self.lib.sil_stop()
            return
        if message.session_id!=100: return
        if message.mode not in (AgentCommand.SNAPSHOT,AgentCommand.AUTHORITY): return
        self.advance()
        if message.mode == AgentCommand.AUTHORITY:
            self.grants.append(message)
            self.record('wire_authority_rx', sequence=message.lease_sequence, active=message.lease_active,
                tick=int(self.head[7]),deadline=message.lease_valid_until_tick_ms)
            if not self.drop_authority:
                with self._native_lock:
                    self.lib.sil_lease(message.lease_id, message.lease_sequence,
                        message.lease_valid_until_tick_ms, int(message.lease_active))
            return
        self.commands.append(message)
        if self.drop_commands: return
        accepted=self.lib.sil_command(message.lease_id, message.sequence, message.valid_until_tick_ms, message.count,
            (C.c_uint8*message.count)(*message.motor_ids), (C.c_int16*message.count)(*message.position_q),
            (C.c_int16*message.count)(*message.velocity_q), (C.c_int16*message.count)(*message.effort_q))
        self.receipts.append((int(self.head[7]),message.sequence,message.count,message.valid_until_tick_ms,int(accepted)))
        self.record('wire_command_rx',sequence=message.sequence,count=message.count,
            tick=int(self.head[7]),deadline=message.valid_until_tick_ms,accepted=bool(accepted))

    def config(self, message):
        if message.schema_version!=AgentWireContract.SCHEMA_VERSION: return
        if message.operation!=AgentConfigRequest.STOP_ALL and message.session_id!=100: return
        self.configs.append(message)
        self.advance()
        if message.operation == AgentConfigRequest.STOP_ALL:
            self.lib.sil_stop()
        elif message.operation == AgentConfigRequest.SET_POLICY:
            self.policies.append(message)
            count = len(message.policies)
            # Match production agent_request_deadline: transport consumes the
            # original issued time, never a fresh full TTL from receipt.
            age=max(0,time.time_ns()-message.issued_stamp)
            remaining=(message.ttl_ms*1000000-age)//1000000
            accepted = remaining>0 and self.lib.sil_policy(message.lease_id, message.request_id, message.policy_generation,
                int(self.head[7])+remaining, count,
                (C.c_uint16*count)(*[p.motor_id for p in message.policies]),
                (C.c_uint8*count)(*[p.enabled for p in message.policies]),
                (C.c_float*count)(*[p.kp for p in message.policies]),
                (C.c_float*count)(*[p.kd for p in message.policies]))
            # ACK is derived later from the native policy generation, never
            # from merely receiving the DDS transaction.
            if not accepted: self.get_logger().warning('native gate rejected policy')
        elif message.operation == AgentConfigRequest.QUERY_INFO:
            page = AgentInfo(session_id=100, clock_valid=True, query_id=message.query_id,
                snapshot_stamp=self.get_clock().now().nanoseconds, snapshot_tick_ms=int(self.head[7]),
                revision=1, cursor=message.cursor, next_cursor=min(self.count,message.cursor+16))
            page.complete = page.next_cursor == self.count
            for i in range(page.cursor,page.next_cursor):
                item = AgentDeviceInfo(motor_id=i+1, node_id=i+1, bus_index=self.buses[i], protocol=0 if self.models[i].startswith('vaxor_') else 1,
                    model_name=self.models[i], online=True, device_valid=True, record_valid=True)
                item.parameters.motor_id=i+1
                item.parameters.dev_init_state=2
                item.parameters.dev_safety=0.
                page.devices.append(item)
            self.info_pub.publish(page)
        elif message.operation == AgentConfigRequest.ACK_RESULTS:
            self._acknowledged.update(message.ack_request_ids)
            self.results = [r for r in self.results if r.request_id not in message.ack_request_ids]

    def publish_state(self):
        self.advance()
        self.sequence += 1
        stamp, tick = self.get_clock().now().nanoseconds, int(self.head[7])
        status = AgentStatus(session_id=100, clock_valid=True, snapshot_stamp=stamp,
            snapshot_tick_ms=tick, sequence=self.sequence, info_revision=1,
            command_sequence=int(self.head[4]), required_policy_generation=int(self.head[5]),
            stop_generation=int(self.head[6]), lease_id=int(self.head[0]),
            lease_sequence=int(self.head[1]), lease_valid_until_tick_ms=int(self.head[2]),
            lease_active=bool(self.head[3]))
        status.lease_valid=status.lease_active
        for i in range(self.count):
            s=self.states[i*10:(i+1)*10]
            status.motors.append(AgentMotorStatus(motor_id=i+1, bus_index=self.buses[i],
                online=bool(s[0]), status_flags=int(bool(s[0])), policy_enabled=bool(s[1]),
                kernel_enabled=bool(s[2]), lifecycle=s[3], execution_state=s[4],
                policy_generation=s[6], unified_fault_code=s[7]))
        for p in self.policies:
            if (p.request_id not in self._acknowledged and p.request_id not in [r.request_id for r in self.results] and
                    all(self.states[(t.motor_id-1)*10+6] == p.policy_generation for t in p.policies)):
                self.results.append(AgentConfigResult(request_id=p.request_id,
                    operation=p.operation, state=AgentConfigResult.SUCCEEDED,
                    completed_motor_mask=sum(1<<(t.motor_id-1) for t in p.policies)))
        status.results=self.results[-8:]
        if not self.suppress_status: self.status_pub.publish(status)
        self.record('native_status_suppressed' if self.suppress_status else 'native_status_tx',source_ns=stamp,tick=tick,sequence=self.sequence,
            authorized=bool(self.head[3]),command_sequence=int(self.head[4]),
            policy=bool(self.states[1]),motion=bool(self.states[2]),execution=int(self.states[4]))
        for group, start in enumerate(range(0,self.count,32)):
            ids=list(range(start,min(start+32,self.count)))
            frame = AgentMotion(session_id=100, snapshot_stamp=stamp, snapshot_tick_ms=tick,
                sequence=self.sequence, group_index=group, group_count=(self.count+31)//32, count=len(ids),
                motor_ids=[i+1 for i in ids],
                position_q=[max(-32767,min(32767,round(self.values[i*3]*32767/12.5))) for i in ids],
                velocity_q=[max(-32767,min(32767,round(self.values[i*3+1]*32767/64))) for i in ids],
                effort_q=[max(-32767,min(32767,round(self.values[i*3+2]*32767/64))) for i in ids],
                state_age_ms=[self.states[i*10+5] for i in ids], target_source=[self.states[i*10+8] for i in ids])
            self.motion_pub.publish(frame)
