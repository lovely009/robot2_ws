"""37 fictional axes: actual supervised JTC -> bridge -> native MCU -> CAN plant.

Run explicitly with ROBOT2_SIL_LIBRARY pointing at the host-built native target.
No real device, calibration record or physical transport is used.
"""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import yaml
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID
from can_bridge_msgs.msg import MotorPolicyArray, MotorPolicyTarget
from can_bridge_msgs.msg import MotorMotionStateArray
from robot_system_msgs.msg import CalibrationState, LeasedPolicy, RobotState
from robot_system_msgs.srv import AcquireControl, RenewControl, SetArmed, AuthorizeMotion
from robot_supervisor.node import SupervisorNode
from robot_calibration.validation import compute_fingerprint
from native_device import NativeDevice, N


def wait(predicate, timeout=15):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate(): return
        time.sleep(.01)
    assert predicate(), 'condition timed out'


def test_supervised_37_axis_native_firmware_chain(tmp_path, monkeypatch):
    library=os.environ.get('ROBOT2_SIL_LIBRARY')
    if not library: pytest.skip('explicit native SIL library required')
    monkeypatch.setenv('ROS_DOMAIN_ID','191')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    driver, urdf=tmp_path/'driver.yaml',tmp_path/'fixture.urdf'
    specs=Path(get_package_share_directory('robot_description'))/'config/motor.yaml'
    driver.write_text(yaml.safe_dump({'motor_hardware': {
        'motor_specs_file':str(specs),
        'buses':{f'b{i}':{'transport':'socketcan','interface':f'sil_never_open_{i}'} for i in range(3)},
        'motors':{f'a{i+1}':{'id':i+1,'joint':f'j{i+1}','type':'cerb_r48_101','bus':f'b{i%3}'} for i in range(N)}}}))
    urdf.write_text('<robot name="sil37"><link name="base"/>'+''.join(
        f'<link name="l{i}"/><joint name="j{i}" type="revolute"><parent link="base"/>'
        f'<child link="l{i}"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint>'
        for i in range(1,N+1))+'</robot>')
    context=Context();rclpy.init(context=context)
    device=NativeDevice(library,context)
    supervisor=SupervisorNode(context=context,parameter_overrides=[
        Parameter('driver_file',value=str(driver)),Parameter('urdf_file',value=str(urdf)),
        Parameter('robot_id',value='sil37-fictional'),Parameter('model_name',value='sil37'),
        Parameter('allow_trajectories',value=True)])
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(device);executor.add_node(supervisor)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    calibration=device.create_publisher(CalibrationState,'/robot/calibration_state',latched)
    state={}
    motion_history=[]
    device.create_subscription(MotorMotionStateArray,'/motor/motion_states',
        lambda m:motion_history.append((time.monotonic(),
            (device.get_clock().now().nanoseconds-m.sync_time_ns)/1e6,
            max((s.state_age_ms for s in m.states),default=0))),
        QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT))
    device.create_subscription(RobotState,'/robot/state',lambda m:state.update(message=m),latched)
    report=CalibrationState(robot_id='sil37-fictional',model_name='sil37',
        calibration_id='fictional-software-fixture-only',config_hash=compute_fingerprint(str(driver),str(urdf)),
        valid=True,commissioned=True,motor_ids=list(range(1,N+1)),bus_indices=[i%3 for i in range(N)],
        protocols=[1]*N,model_names=['cerb_r48_101']*N,joint_names=[f'j{i}' for i in range(1,N+1)])
    def publish_calibration():
        report.header.stamp=device.get_clock().now().to_msg();calibration.publish(report)
    device.create_timer(.2,publish_calibration)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    children=[];logs=[]
    def call(kind,name,request):
        client=device.create_client(kind,'/robot/'+name)
        wait(client.service_is_ready)
        future=client.call_async(request);wait(future.done)
        result=future.result();device.destroy_client(client)
        assert result.success,result.reason
        return result
    try:
        executable=Path(get_package_prefix('micro_ros_bridge'))/'lib/micro_ros_bridge/micro_ros_bridge_node'
        launch=Path(get_package_share_directory('motor_hardware_interface'))/'launch/host_trajectory.launch.py'
        for name,command in [('bridge',[str(executable)]),('control',['ros2','launch',str(launch),
                f'motor_config_file:={driver}',f'urdf_file:={urdf}','start_controllers:=true'])]:
            log=open(tmp_path/f'{name}.log','w');logs.append(log)
            children.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        try: wait(lambda:state.get('message') is not None and state['message'].ready,30)
        except AssertionError: raise AssertionError(f"readiness={state}; native={list(device.head)}; logs="+
            '\n'.join((tmp_path/f'{name}.log').read_text() for name in ['bridge','control']))
        wait(lambda:supervisor._motion_adapter._client.server_is_ready()
            and supervisor._motion_adapter._controllers_ready
            and supervisor._motion_adapter._feedback is not None)
        lease=call(AcquireControl,'acquire_control',AcquireControl.Request(owner='sil-test',mode=3,lease_ms=2000))
        renew=device.create_client(RenewControl,'/robot/renew_control')
        device.create_timer(.3,lambda:renew.call_async(RenewControl.Request(
            lease_id=lease.lease_id,epoch=lease.epoch,lease_ms=2000)))
        call(SetArmed,'set_armed',SetArmed.Request(lease_id=lease.lease_id,epoch=lease.epoch,armed=True))
        wait(lambda:state['message'].mcu_authorized)
        policy=device.create_publisher(LeasedPolicy,'/robot/control/policy',10)
        wait(lambda:policy.get_subscription_count()>0)
        envelope=LeasedPolicy(lease_id=lease.lease_id,epoch=lease.epoch,session_id=lease.session_id,sequence=1,
            policy=MotorPolicyArray(count=N,targets=[MotorPolicyTarget(motor_id=i+1,enabled=True,kp=30.,kd=1.) for i in range(N)]))
        envelope.header.stamp=device.get_clock().now().to_msg();policy.publish(envelope)
        wait(lambda:all(device.states[i*10+1] for i in range(N)))
        action=ActionClient(device,FollowJointTrajectory,'/robot/follow_joint_trajectory')
        wait(action.server_is_ready)
        wait(lambda:supervisor._motion_adapter._client.server_is_ready()
            and supervisor._motion_adapter._feedback is not None)
        def start_goal(sequence,target,seconds):
            identifier=UUID(uuid=list(uuid.uuid4().bytes))
            request=AuthorizeMotion.Request(lease_id=lease.lease_id,epoch=lease.epoch,
                session_id=lease.session_id,sequence=sequence,goal_id=identifier.uuid)
            request.header.stamp=device.get_clock().now().to_msg()
            call(AuthorizeMotion,'authorize_motion',request)
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=[f'j{i}' for i in range(1,N+1)]
            for position,sec in [(None,0),(target,seconds)]:
                point=JointTrajectoryPoint(positions=[float(device.values[i*3]) if position is None else position for i in range(N)],velocities=[0.]*N)
                point.time_from_start.sec=sec;goal.trajectory.points.append(point)
            future=action.send_goal_async(goal,goal_uuid=identifier);wait(future.done)
            handle=future.result();assert handle.accepted
            return handle
        goal=start_goal(2,.05,2)
        result=goal.get_result_async();wait(result.done,10)
        print('SIL goal result=',result.result(), 'last command receipts=',device.receipts[-20:],flush=True)
        assert result.result().status==GoalStatus.STATUS_SUCCEEDED, (result.result(),
            list(device.head),list(device.states[:10]),motion_history[-10:])
        assert max(abs(device.values[i*3]-.05) for i in range(N))<.01
        print('SIL: supervised JTC/native/CAN/37-axis feedback convergence passed',flush=True)
        goal=start_goal(3,.15,4)
        wait(lambda:min(device.values[i*3] for i in range(N))>.055)
        canceled=goal.cancel_goal_async();wait(canceled.done)
        result=goal.get_result_async();wait(result.done)
        assert result.result().status==GoalStatus.STATUS_CANCELED,result.result()
        assert all(device.states[i*10+1] for i in range(N)), 'cancel must preserve enabled policy'
        print('SIL: cancel keeps independent enabled policy passed',flush=True)
        # A healthy ownership renewal cannot keep a dead target source alive.
        device.drop_commands=True
        wait(lambda:not any(device.states[i*10+1] for i in range(N)),2)
        assert not device.head[3], 'command-age barrier must revoke MCU authority'
        wait(lambda:device.lib.sil_enabled_mask()==0,1)
        print('SIL: dropped target stream revokes despite continued ownership renewals passed',flush=True)
    finally:
        print('SIL diagnostic: head=',list(device.head),'first=',list(device.states[:10]),
            'motion history=',motion_history[-5:], 'max motion age=',max((x[2] for x in motion_history),default=-1),
            'core=',[device.lib.sil_diagnostic(0,j) for j in range(9)],flush=True)
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        executor.shutdown();thread.join(timeout=2)
        supervisor.destroy_node();device.destroy_node();context.shutdown()
