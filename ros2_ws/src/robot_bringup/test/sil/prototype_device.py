"""Local interactive prototype using the real native firmware kernel and CAN codecs.
Virtual time pauses across host stalls; this is deliberately not a wall-clock timing test.
"""
import ctypes as C
import json
import math
import time
from pathlib import Path
import sys

from native_device import NativeDevice
from robot_transmission import Model, Axis
from can_bridge_msgs.msg import (MotorPolicyArray, MotorCommandArray, MotorControlStateArray,
    MotorControlState, MotorMotionStateArray, MotorMotionStateTarget,
    MotorTopology, MotorTopologyEntry, MotorStatusArray, MotorStatusTarget)
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from trajectory_msgs.msg import JointTrajectory


class PrototypeDevice(NativeDevice):
    def __init__(self, library, context, driver):
        import yaml
        hardware=yaml.safe_load(Path(driver).read_text())['motor_hardware']
        motors=sorted(hardware['motors'].values(),key=lambda m:m['id'])
        super().__init__(library,context,models=[m['type'] for m in motors],buses=[0]*len(motors))
        self.model=Model.load(str(driver))
        self.names=list(self.model.joint_names)
        self.ids=dict(self.model.actuator_ids)
        self.targets=[0.]*self.count
        self.plan=None
        self._proto_tick=1000
        self._proto_at=time.monotonic()
        self._lease_id=100000001
        self._lease_seq=self._command_seq=self._generation=0
        self._pending=[]
        self._mode='idle'
        self._last_error=''
        self.lib.sil_advance(1001)
        self._proto_tick=1001
        self._send_native([],0)
        self.joints_pub=self.create_publisher(JointState,'/joint_states',10)
        self.route_pub=self.create_publisher(String,'/robot_transmission/status',10)
        self.host_pub=self.create_publisher(String,'/control/status',10)
        self.control_pub=self.create_publisher(MotorControlStateArray,'/motor/control_states',10)
        self.motion_display_pub=self.create_publisher(MotorMotionStateArray,'/motor/motion_states',10)
        self.topology_pub=self.create_publisher(MotorTopology,'/motor/topology',10)
        self.health_pub=self.create_publisher(MotorStatusArray,'/motor/status_array',10)
        self.create_subscription(MotorPolicyArray,'/motor/policy_array',self.policy,10)
        self.create_subscription(MotorCommandArray,'/control/setpoint',self.setpoint,10)
        self.create_subscription(JointTrajectory,'/arm_controller/joint_trajectory',self.trajectory,10)
        self.create_subscription(Empty,'/control/stop',self.stop,10)

    def command(self, _message):
        # The prototype owns its local kernel clock; no transport lease is used.
        pass

    def _send_native(self, values, lease=None):
        count=len(values)
        self._command_seq+=1
        q=lambda value:max(-32767,min(32767,round(value*32767/12.5)))
        accepted=self.lib.sil_command(self._lease_id if lease is None else lease,self._command_seq,
            self._proto_tick+30,count,(C.c_uint8*count)(*range(1,count+1)),
            (C.c_int16*count)(*[q(v) for v in values]),(C.c_int16*count)(),(C.c_int16*count)())
        return accepted

    def advance(self):
        if not hasattr(self,'_proto_tick'): return
        now=time.monotonic()
        dt=max(0,min(10,int((now-self._proto_at)*1000)))
        if dt==0: return
        self._proto_at=now
        with self._native_lock:
            self._lease_seq+=1
            self.lib.sil_lease(self._lease_id,self._lease_seq,self._proto_tick+200,1)
            if self._pending:
                request,generation,targets=self._pending[0]
                ids=[t.motor_id for t in targets]
                if self.lib.sil_policy(self._lease_id,request,generation,self._proto_tick+100,len(ids),
                    (C.c_uint16*len(ids))(*ids),(C.c_uint8*len(ids))(*[t.enabled for t in targets]),
                    (C.c_float*len(ids))(*[t.kp for t in targets]),(C.c_float*len(ids))(*[t.kd for t in targets])):
                    self._pending.pop(0)
            if self.plan:
                started,duration,initial,destination=self.plan
                ratio=min(1.,(now-started)/duration)
                values={name:Axis(initial[name]+(destination[name]-initial[name])*ratio) for name in self.names}
                actuators=self.model.joint_to_actuator(values)
                self.targets=[actuators[name].position for name,mid in sorted(self.ids.items(),key=lambda item:item[1])]
                if ratio>=1: self.plan=None;self._mode='holding'
            self._send_native(self.targets)
            self._proto_tick+=dt
            self.lib.sil_advance(self._proto_tick)
            self.lib.sil_snapshot(self.values,self.states,self.head)

    def policy(self,msg):
        try:
            targets=list(msg.targets)
            if len(targets)!=msg.count or not targets or len({t.motor_id for t in targets})!=len(targets):
                raise ValueError('invalid policy targets')
            if any(not 1<=t.motor_id<=self.count or not all(math.isfinite(v) and v>=0 for v in [t.kp,t.kd]) for t in targets):
                raise ValueError('invalid motor or gains')
            self._generation=max(self._generation+1,int(msg.policy_generation))
            self._pending.append((int(msg.request_id),self._generation,targets))
            self._last_error=''
        except ValueError as e: self._last_error=str(e)

    def setpoint(self,msg):
        for t in msg.targets:
            if not 1<=t.motor_id<=self.count or not math.isfinite(t.position):
                self._last_error='invalid motor target';return
        self.plan=None
        for t in msg.targets: self.targets[t.motor_id-1]=t.position
        self._mode='manual'

    def measured_joints(self):
        return self.model.actuator_to_joint({name:Axis(float(self.values[(mid-1)*3])) for name,mid in self.ids.items()})

    def trajectory(self,msg):
        try:
            if not msg.points or not msg.joint_names or len(set(msg.joint_names))!=len(msg.joint_names):
                raise ValueError('empty or duplicate joint target')
            if not set(msg.joint_names)<=set(self.names): raise ValueError('unknown joint')
            point=msg.points[-1]
            duration=point.time_from_start.sec+point.time_from_start.nanosec/1e9
            if len(point.positions)!=len(msg.joint_names) or not .01<=duration<=60 or not all(map(math.isfinite,point.positions)):
                raise ValueError('invalid target or duration')
            current=self.measured_joints()
            initial={name:current[name].position for name in self.names}
            destination=dict(initial)
            destination.update(zip(msg.joint_names,point.positions))
            self.plan=(time.monotonic(),duration,initial,destination)
            self._mode='moving';self._last_error=''
        except ValueError as e: self._last_error=str(e)

    def stop(self,_msg):
        self.plan=None;self._pending.clear()
        with self._native_lock:
            self.lib.sil_stop();self._lease_id+=1
            self.targets=[float(self.values[i*3]) for i in range(self.count)]
        self._mode='stopped';self._last_error=''

    def publish_state(self):
        self.advance()
        self.sequence+=1
        if not hasattr(self,'joints_pub'): return
        stamp=time.time_ns()
        control=MotorControlStateArray(schema_version=2,sync_time_ns=stamp,session_id=100,
            sequence=self.sequence,tick_ms=self._proto_tick)
        motion=MotorMotionStateArray(sync_time_ns=stamp,session_id=100,seq=self.sequence,tick_ms=self._proto_tick)
        for i in range(self.count):
            s=self.states[i*10:(i+1)*10]
            control.states.append(MotorControlState(motor_id=i+1,online=bool(s[0]),
                control_lifecycle=int(s[3]),execution_state=int(s[4]),policy_enabled=bool(s[1]),
                motion_enabled=bool(s[2]),state_age_ms=int(s[5]),policy_generation=int(s[6]),unified_fault_code=int(s[7])))
            motion.states.append(MotorMotionStateTarget(motor_id=i+1,position=float(self.values[i*3]),
                velocity=float(self.values[i*3+1]),effort=float(self.values[i*3+2]),state_age_ms=int(s[5]),target_source=int(s[8])))
        self.control_pub.publish(control);self.motion_display_pub.publish(motion)
        if self.sequence%25==0:
            self.topology_pub.publish(MotorTopology(sync_time_ns=stamp,session_id=100,schema_version=2,
                entries=[MotorTopologyEntry(motor_id=i+1,bus_index=0,protocol=0 if name.startswith('vaxor_') else 1,
                    model_name=name,online=True) for i,name in enumerate(self.models)]))
            self.health_pub.publish(MotorStatusArray(sync_time_ns=stamp,statuses=[
                MotorStatusTarget(motor_id=i+1,online=bool(self.states[i*10]),bus_index=0,
                    lifecycle=4 if self.states[i*10+1] else 3) for i in range(self.count)]))
        measured=self.measured_joints()
        joint=JointState(name=self.names,position=[measured[n].position for n in self.names])
        joint.header.stamp=self.get_clock().now().to_msg()
        self.joints_pub.publish(joint)
        if self.sequence%5==0:
            route={'model':'half_body_robot','joint_names':self.names,'trajectory_enabled':True,
                'mode':self._mode,'output_topic':'native_firmware_prototype','feedback_fresh':True,
                'last_error':self._last_error,'prototype':True}
            self.route_pub.publish(String(data=json.dumps(route)))
            self.host_pub.publish(String(data=json.dumps(route)))
