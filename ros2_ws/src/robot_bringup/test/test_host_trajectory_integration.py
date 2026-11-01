"""Real supervised JTC / native firmware regressions, not an ideal TopicDevice.

Both the standard Action and the legacy workbench intent use the same managed
path. Inventory, buses and calibration are explicitly fictional fixtures.
"""
from pathlib import Path
import sys
import time

import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from can_bridge_msgs.msg import MotorCommandArray, MotorCommandTarget, MotorTrajectory, MotorTrajectoryPoint, MotorTrajectoryState

sys.path.insert(0,str(Path(__file__).resolve().parent/'sil'))
from managed_native_fixture import ManagedNativeFixture, wait, stop
from native_device import N


def client_call(h,kind,name,request):
    client=h.device.create_client(kind,name)
    try:
        wait(client.service_is_ready);future=client.call_async(request);wait(future.done)
        return future.result()
    finally:h.device.destroy_client(client)


def complete(h,position,seconds=1):
    handle=h.goal(position,seconds);assert handle.accepted
    result=handle.get_result_async();wait(result.done,seconds+4)
    assert result.result().status==GoalStatus.STATUS_SUCCEEDED,result.result()
    assert result.result().result.error_code==0
    assert max(abs(p-position) for p in h.device.positions())<.01


def exercise_workbench(h):
    # Warm the drives through an explicitly authorized goal; a manual request
    # is still bounded to its original 200ms lifetime, not the vendor enable wait.
    complete(h,0.,.5)
    request=MotorCommandArray(sync_time_ns=h.device.get_clock().now().nanoseconds,count=1,
        targets=[MotorCommandTarget(motor_id=1,position=.04,velocity=0.,effort=1.)])
    h.command_pub.publish(h.envelope('command',request))
    wait(lambda:h.host.get('mode')=='manual')
    wait(lambda:any(m.count and any(q!=0 for q in m.effort_q) for m in h.device.commands[-15:]))
    wait(lambda:abs(h.device.positions()[0]-.04)<.01,1)
    h.call(Trigger,'stop',Trigger.Request())
    wait(lambda:h.device.enabled_mask()==0 and h.latest and 'awaiting_disabled' not in h.latest.blockers)
    time.sleep(.06);positive=sum(bool(m.count) for m in h.device.commands)
    time.sleep(.15);assert sum(bool(m.count) for m in h.device.commands)==positive
    # Stop requires an explicit fresh reset/acquire/arm/enable sequence.
    h.lease=None
    h.call(Trigger,'reset_fault',Trigger.Request());h.acquire();h.arm();h.enable()
    complete(h,0.,.5)
    progress=[]
    h.device.create_subscription(MotorTrajectoryState,'/control/trajectory_state',progress.append,10)
    def trajectory(identifier,position,milliseconds):
        payload=MotorTrajectory(sync_time_ns=h.device.get_clock().now().nanoseconds,
            trajectory_id=identifier,segment_count=1,count=2,target_count=1,target_ids=[1],points=[
                MotorTrajectoryPoint(motor_id=1,time_from_start_ms=0,position=h.device.positions()[0]),
                MotorTrajectoryPoint(motor_id=1,time_from_start_ms=milliseconds,position=position)])
        h.trajectory_pub.publish(h.envelope('trajectory',payload))
    trajectory(77,.08,1500)
    wait(lambda:any(m.trajectory_id==77 and m.active for m in progress))
    wait(lambda:any(m.trajectory_id==77 and m.completed for m in progress),5)
    assert abs(h.device.positions()[0]-.08)<.01
    # Preserve the dynamics/effort debug path while JTC is the sole writer.
    gravity='/host_control/gravity_controller/set_parameters_atomically'
    response=client_call(h,SetParametersAtomically,gravity,SetParametersAtomically.Request(parameters=[
        Parameter('enabled',value=True).to_parameter_msg(),Parameter('effort_gain',value=1.).to_parameter_msg()]))
    assert response.result.successful,response.result.reason
    wait(lambda:h.host.get('gravity_enabled') is True)
    wait(lambda:any(m.count and any(abs(q)>400 for q in m.effort_q) for m in h.device.commands[-15:]))
    response=client_call(h,SetParametersAtomically,gravity,SetParametersAtomically.Request(parameters=[
        Parameter('enabled',value=False).to_parameter_msg()]))
    assert response.result.successful
    wait(lambda:h.host.get('gravity_enabled') is False)
    wait(lambda:h.device.commands[-1].count and all(q==0 for q in h.device.commands[-1].effort_q))
    trajectory(78,.3,6000)  # Host plan is longer than the old MCU 2-second adapter.
    wait(lambda:h.device.positions()[0]>.09)
    h.trajectory_pub.publish(h.envelope('trajectory',MotorTrajectory(
        sync_time_ns=h.device.get_clock().now().nanoseconds,trajectory_id=78,segment_count=1)))
    wait(lambda:any(m.trajectory_id==78 and not m.active and not m.completed for m in progress))
    assert h.latest.armed and all(h.device.states[i*10+1] for i in range(N))
    clients=h.device.create_publisher(Int32,'/client_count',h.latched)
    wait(lambda:clients.get_subscription_count()>0)
    clients.publish(Int32(data=0));h.renew_timer.cancel()
    wait(lambda:h.host.get('mode')=='stopped')
    time.sleep(.08);positive=sum(bool(m.count) for m in h.device.commands)
    request.sync_time_ns=h.device.get_clock().now().nanoseconds
    h.command_pub.publish(h.envelope('command',request));time.sleep(.1)
    assert sum(bool(m.count) for m in h.device.commands)==positive
    wait(lambda:h.device.enabled_mask()==0,3)


@pytest.mark.parametrize('via_workbench',[False,True])
def test_real_jtc_executes_cancels_and_cannot_renew_after_control_loop_stops(tmp_path,via_workbench):
    with ManagedNativeFixture(tmp_path,domain=179) as h:
        h.acquire();h.arm();h.enable()
        if via_workbench:
            exercise_workbench(h)
        else:
            complete(h,.08)
            handle=h.goal(.3,3);assert handle.accepted
            wait(lambda:min(h.device.positions())>.09)
            canceled=handle.cancel_goal_async();wait(canceled.done)
            assert canceled.result().goals_canceling
            result=handle.get_result_async();wait(result.done)
            assert result.result().status==GoalStatus.STATUS_CANCELED
            time.sleep(.2);assert max(h.device.positions())<.2
            assert all(h.device.states[i*10+1] for i in range(N))
            changed=client_call(h,SwitchController,'/host_control/controller_manager/switch_controller',
                SwitchController.Request(deactivate_controllers=['trajectory_controller'],
                    strictness=SwitchController.Request.STRICT,timeout=Duration(sec=2)))
            assert changed.ok
            time.sleep(.1);positive=sum(bool(m.count) for m in h.device.commands)
            time.sleep(.15);assert sum(bool(m.count) for m in h.device.commands)==positive
            assert h.processes['control'].poll() is None
        stop(h.processes['control'])
        time.sleep(.1);count=len(h.device.commands)
        time.sleep(.15);assert len(h.device.commands)==count
        wait(lambda:h.device.enabled_mask()==0,1)
