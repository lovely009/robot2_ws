"""Shared, device-free fixture for the unified production host/MCU path.

The virtual MCU has its own executor; supervisor, bridge and JTC run in their
production processes. Test control tokens never leave this process or logs.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
from unittest.mock import patch

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

from can_bridge_msgs.msg import ConfigRequest, MotorCommandArray, MotorControlStateArray, MotorPolicyArray, MotorPolicyTarget
from robot_calibration.validation import compute_fingerprint
from robot_system_msgs.msg import CalibrationState, LeasedCommand, LeasedPolicy, LeasedTrajectory, RobotState
from robot_system_msgs.srv import AcquireControl, AuthorizeMotion, RenewControl, SetArmed
from native_device import NativeDevice, N


def wait(predicate, seconds=15):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.005)
    assert predicate(), 'condition timed out'


def stop(process):
    if process.poll() is not None: return
    os.killpg(process.pid,signal.SIGCONT)
    os.killpg(process.pid,signal.SIGINT)
    try: process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)


class ManagedNativeFixture:
    def __init__(self, folder, *, domain=191, driver_file=None, urdf_file=None, model_name='sil37'):
        library=os.environ.get('ROBOT2_SIL_LIBRARY')
        if not library: pytest.skip('explicit production native SIL library required')
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.driver,self.urdf=self.folder/'driver.yaml',self.folder/'fixture.urdf'
        specs=Path(get_package_share_directory('robot_description'))/'config/motor.yaml'
        self.driver.write_text(yaml.safe_dump({'motor_hardware':{
            'motor_specs_file':str(specs),
            'buses':{f'b{i}':{'transport':'socketcan','interface':f'sil_never_open_{i}'} for i in range(3)},
            'motors':{f'a{i+1}':{'id':i+1,'joint':f'j{i+1}','type':'cerb_r48_101','bus':f'b{i%3}'} for i in range(N)}}}))
        self.urdf.write_text('<robot name="sil37"><link name="base"/>'+''.join(
            f'<link name="l{i}"><inertial><origin xyz="1 0 0"/><mass value="0.1"/>'
            '<inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>'
            f'<joint name="j{i}" type="revolute"><parent link="base"/><child link="l{i}"/>'
            '<axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint>'
            for i in range(1,N+1))+'</robot>')
        self.model_name = model_name
        self.robot_id = model_name + "-fictional"
        if (driver_file is None) != (urdf_file is None):
            raise ValueError('driver_file and urdf_file must be provided together')
        if driver_file is not None:
            self.driver.write_text(Path(driver_file).read_text())
            self.urdf.write_text(Path(urdf_file).read_text())
        from robot_transmission import Model
        self.model = Model.load(str(self.driver))
        doc = yaml.safe_load(self.driver.read_text())['motor_hardware']
        motors = sorted(doc['motors'].values(), key=lambda m: m['id'])
        self.motor_ids = [m['id'] for m in motors]
        if self.motor_ids != list(range(1, len(motors)+1)):
            raise ValueError('native SIL requires contiguous motor IDs beginning at 1')
        bus_names = sorted(doc['buses'])
        self.bus_indices = [bus_names.index(m['bus']) for m in motors]
        self.model_names = [m['type'] for m in motors]
        self.joint_names = list(self.model.joint_names)
        self.gains = [m.get('command', {'kp':30.,'kd':1.}) for m in motors]
        self.environment=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
        self.affinity_before=os.sched_getaffinity(0)
        requested=os.environ.get('ROBOT2_SIL_CPUS','')
        self.cpus=[int(cpu) for cpu in requested.split(',')] if requested else []
        if self.cpus:
            if len(self.cpus)!=4 or len(set(self.cpus))!=4 or not set(self.cpus)<=self.affinity_before:
                raise ValueError('ROBOT2_SIL_CPUS must name four distinct allowed CPUs')
            os.sched_setaffinity(0,{self.cpus[0]})
        self.context=Context()
        # RMW reads localhost-only at context/participant creation. Match the
        # production child processes without changing later test environments.
        with patch.dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1'):
            rclpy.init(args=[],context=self.context,domain_id=domain)
            self.device=NativeDevice(library,self.context, models=self.model_names, buses=self.bus_indices)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.device)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True)
        self.processes={};self.logs=[];self.latest=None;self.host={};self.controls=[]
        self.private_commands=[];self.raw_commands=[];self.reports=[];self.sequence=0;self.lease=None
        self._renew_future=None
        self.reliable=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        self.latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        observer=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.calibration=self.device.create_publisher(CalibrationState,'/robot/calibration_state',self.latched)
        self.device.create_subscription(RobotState,'/robot/state',self._state,self.latched)
        self.device.create_subscription(String,'/control/status',self._host_status,self.latched)
        self.device.create_subscription(JointState,'/host_control/joint_states',
            lambda m:self.host.update(feedback_at=time.monotonic()),observer)
        for topic,collection in (('/motor/host_command_array',self.raw_commands),
                ('/robot/managed/motor/command_array',self.private_commands)):
            self.device.create_subscription(MotorCommandArray,topic,
                lambda m,name=topic,items=collection:self._command(name,m,items),observer)
        self.device.create_subscription(MotorControlStateArray,'/motor/control_states',self._control,observer)
        self.device.create_subscription(ConfigRequest,'/robot/managed/maintenance/config_request',self.reports.append,self.reliable)
        self.command_pub=self.device.create_publisher(LeasedCommand,'/robot/control/command',self.reliable)
        self.policy_pub=self.device.create_publisher(LeasedPolicy,'/robot/control/policy',self.reliable)
        self.trajectory_pub=self.device.create_publisher(LeasedTrajectory,'/robot/control/trajectory',self.reliable)
        self.action=ActionClient(self.device,FollowJointTrajectory,'/robot/follow_joint_trajectory')
        self.renew_client=self.device.create_client(RenewControl,'/robot/renew_control')
        self.renew_timer=self.device.create_timer(.3,self._renew)
        self.device.create_timer(.2,self._calibrate)
        self.fingerprint=compute_fingerprint(str(self.driver),str(self.urdf))

    def _command(self,topic,message,collection):
        collection.append(message)
        self.device.record(topic,source_ns=message.sync_time_ns,sequence=message.sequence,count=message.count)

    def _state(self,message):
        self.latest=message

    def _host_status(self,message):
        self.host.update(json.loads(message.data))
        self.device.record('host_status',mode=self.host.get('mode'),controllers_ready=self.host.get('controllers_ready'))

    def _control(self,message):
        self.controls.append(message)
        self.device.record('control_state_rx',source_ns=message.sync_time_ns,sequence=message.sequence,
            tick=message.tick_ms,authorized=message.lease_active)

    def _calibrate(self):
        report=CalibrationState(robot_id=self.robot_id,model_name=self.model_name,
            calibration_id='fictional-software-fixture-only',config_hash=self.fingerprint,
            valid=True,commissioned=True,motor_ids=self.motor_ids,bus_indices=self.bus_indices,
            protocols=[0 if name.startswith('vaxor_') else 1 for name in self.model_names],
            model_names=self.model_names,joint_names=self.joint_names)
        report.header.stamp=self.device.get_clock().now().to_msg();self.calibration.publish(report)

    def _renew(self):
        if self.lease and (self._renew_future is None or self._renew_future.done()):
            self._renew_future=self.renew_client.call_async(RenewControl.Request(
                lease_id=self.lease.lease_id,epoch=self.lease.epoch,lease_ms=2000))

    def start(self):
        self.thread.start()
        bridge=Path(get_package_prefix('micro_ros_bridge'))/'lib/micro_ros_bridge/micro_ros_bridge_node'
        supervisor=Path(get_package_prefix('robot_supervisor'))/'lib/robot_supervisor/robot_supervisor'
        launch=Path(get_package_share_directory('motor_hardware_interface'))/'launch/host_trajectory.launch.py'
        commands={
            'supervisor':[str(supervisor),'--ros-args','-p',f'driver_file:={self.driver}','-p',f'urdf_file:={self.urdf}',
                '-p',f'robot_id:={self.robot_id}','-p',f'model_name:={self.model_name}','-p','allow_trajectories:=true'],
            'bridge':[str(bridge)],
            'control':['ros2','launch',str(launch),f'motor_config_file:={self.driver}',f'urdf_file:={self.urdf}',f'model:={self.model_name}','start_controllers:=true'],
        }
        for index,(name,command) in enumerate(commands.items(),start=1):
            if self.cpus: command=['taskset','-c',str(self.cpus[index]),*command]
            log=open(self.folder/f'{name}.log','w');self.logs.append(log)
            self.processes[name]=subprocess.Popen(command,env=self.environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        wait(lambda:self.latest is not None and self.latest.ready and self.action.server_is_ready()
            and self.host.get('controllers_ready') and time.monotonic()-self.host.get('feedback_at',-100)<.2,30)
        self.device.record('fixture_phase',phase='ready')
        return self

    def call(self,kind,name,request,*,success=True):
        client=self.device.create_client(kind,'/robot/'+name)
        try:
            wait(client.service_is_ready);future=client.call_async(request);wait(future.done)
            response=future.result()
            if success: assert response.success,getattr(response,'reason',getattr(response,'message',''))
            return response
        finally:self.device.destroy_client(client)

    def acquire(self):
        self.lease=self.call(AcquireControl,'acquire_control',AcquireControl.Request(owner='native-integration-test',mode=3,lease_ms=2000))
        self.sequence=0
        return self.lease

    def arm(self,*,wait_ack=True):
        self.call(SetArmed,'set_armed',SetArmed.Request(lease_id=self.lease.lease_id,epoch=self.lease.epoch,armed=True))
        if wait_ack:wait(lambda:self.latest.mcu_authorized)

    def envelope(self,kind,payload):
        self.sequence+=1
        cls={'command':LeasedCommand,'policy':LeasedPolicy,'trajectory':LeasedTrajectory}[kind]
        message=cls(lease_id=self.lease.lease_id,epoch=self.lease.epoch,session_id=self.lease.session_id,
            sequence=self.sequence,**{kind:payload})
        message.header.stamp=self.device.get_clock().now().to_msg()
        return message

    def enable(self):
        self.policy_pub.publish(self.envelope('policy',MotorPolicyArray(count=len(self.motor_ids),targets=[
            MotorPolicyTarget(motor_id=mid,enabled=True,kp=float(gain['kp']),kd=float(gain['kd']))
            for mid,gain in zip(self.motor_ids,self.gains)])))
        wait(lambda:bool(self.controls) and all(s.policy_enabled for s in self.controls[-1].states)
            and all(self.device.states[i*10+1] for i in range(len(self.motor_ids))))

    def goal(self,target,seconds,*,bind=True,feedback=None,points=2):
        identifier=UUID(uuid=list(uuid.uuid4().bytes))
        if bind:
            self.sequence+=1
            request=AuthorizeMotion.Request(lease_id=self.lease.lease_id,epoch=self.lease.epoch,
                session_id=self.lease.session_id,sequence=self.sequence,goal_id=identifier.uuid)
            request.header.stamp=self.device.get_clock().now().to_msg()
            self.call(AuthorizeMotion,'authorize_motion',request)
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=self.joint_names
        from robot_transmission import Axis
        measured = self.device.positions()
        joints = self.model.actuator_to_joint({name:Axis(measured[mid-1]) for name,mid in self.model.actuator_ids.items()})
        initial=[joints[name].position for name in self.joint_names]
        for index in range(points):
            amount=index/(points-1);time_ns=round(seconds*amount*1e9)
            point=JointTrajectoryPoint(positions=[p+(target-p)*amount for p in initial],velocities=[0.]*len(self.joint_names))
            point.time_from_start.sec=time_ns//1000000000;point.time_from_start.nanosec=time_ns%1000000000
            goal.trajectory.points.append(point)
        self.device.record('fixture_phase',phase='send_goal',bound=bind)
        sent=self.action.send_goal_async(goal,goal_uuid=identifier,feedback_callback=feedback)
        wait(sent.done)
        return sent.result()

    def close(self):
        self.renew_timer.cancel()
        self.device.save_trace(self.folder/'timeline.jsonl')
        for process in reversed(list(self.processes.values())):stop(process)
        for log in self.logs:log.close()
        self.executor.shutdown();self.thread.join(timeout=2)
        self.action.destroy();self.device.destroy_node();self.context.shutdown()
        os.sched_setaffinity(0,self.affinity_before)

    def __enter__(self):
        try:return self.start()
        except BaseException:self.close();raise

    def __exit__(self,*_):self.close()
