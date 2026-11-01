"""Unified Agent/JTC runtime regressions with production native MCU logic.

Migrated from A's LowCommand/MCU-Action fixture. Goal/lease assertions now cross
the sole host JTC path. Cancel is independent of disable; explicit stop still
requires fresh disabled acknowledgement. No hardware or fake motion ACKs.
"""
import copy
from pathlib import Path
import signal
import os
import sys
import time

from action_msgs.msg import GoalStatus
from can_bridge_msgs.msg import (AgentConfigRequest, ConfigRequest, ConfigResult,
    MotorCommandArray, MotorPolicyArray, MotorPolicyTarget, SystemCommand)
from robot_system_msgs.srv import ExecuteMaintenance
from std_srvs.srv import Trigger

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'robot_bringup/test/sil'))
from managed_native_fixture import ManagedNativeFixture, wait
from native_device import N


def test_wire_handshake_standard_action_and_no_bridge_renewal(tmp_path):
    with ManagedNativeFixture(tmp_path,domain=187) as h:
        h.acquire();h.device.drop_authority=True;h.arm(wait_ack=False)
        wait(lambda:any(m.lease_active for m in h.device.grants))
        h.policy_pub.publish(h.envelope('policy',MotorPolicyArray(count=N,targets=[
            MotorPolicyTarget(motor_id=i+1,enabled=True,kp=30.,kd=1.) for i in range(N)])))
        time.sleep(.025)
        assert not h.device.policies, 'host arm cannot replace the MCU lease ACK'
        h.device.drop_authority=False
        wait(lambda:h.latest.mcu_authorized)
        h.enable()
        assert h.device.enabled_mask()==0, 'policy alone cannot generate a motion target'
        lease_id=int(h.device.head[0])
        assert all(p.lease_id==lease_id for p in h.device.policies if any(t.enabled for t in p.policies))
        assert not h.goal(.02,1,bind=False).accepted, 'standard Action requires the one-use UUID grant'
        feedback=[]
        goal=h.goal(.05,1,feedback=feedback.append);assert goal.accepted
        result=goal.get_result_async();wait(result.done,5)
        assert result.result().status==GoalStatus.STATUS_SUCCEEDED and result.result().result.error_code==0
        assert feedback and any(m.count and m.lease_id==lease_id for m in h.device.commands)
        assert h.device.enabled_mask() and h.latest.armed
        # Production supervisor is frozen, not just a fake executor removed.
        os.killpg(h.processes['supervisor'].pid,signal.SIGSTOP)
        time.sleep(.04)
        before=len(h.device.grants)
        wait(lambda:not h.device.head[3] and h.device.enabled_mask()==0,1)
        time.sleep(.05)
        assert len(h.device.grants)==before
        assert h.processes['bridge'].poll() is None
        # An expired generation cannot be revived by refreshing a facade stamp.
        old=copy.deepcopy(next(m for m in reversed(h.private_commands) if m.count))
        old.sync_time_ns=h.device.get_clock().now().nanoseconds
        facade=h.device.create_publisher(MotorCommandArray,'/robot/managed/motor/command_array',10)
        wait(lambda:facade.get_subscription_count()>0)
        count=sum(bool(m.count) for m in h.device.commands)
        facade.publish(old);time.sleep(.08)
        assert sum(bool(m.count) for m in h.device.commands)==count
        assert h.device.enabled_mask()==0


def test_wire_action_cancel_preserves_policy_and_stop_waits_for_disabled_ack(tmp_path):
    with ManagedNativeFixture(tmp_path,domain=188) as h:
        h.acquire();h.arm();h.enable()
        goal=h.goal(.15,3);assert goal.accepted
        wait(lambda:min(h.device.positions())>.01)
        canceled=goal.cancel_goal_async();wait(canceled.done)
        assert canceled.result().goals_canceling
        result=goal.get_result_async();wait(result.done)
        assert result.result().status==GoalStatus.STATUS_CANCELED and result.result().result.error_code==0
        assert all(h.device.states[i*10+1] for i in range(N)) and h.latest.armed
        # Stop's service response is not the physical disabled acknowledgement.
        h.device.suppress_status=True
        h.call(Trigger,'stop',Trigger.Request())
        wait(lambda:h.device.enabled_mask()==0,1)
        wait(lambda:'awaiting_disabled' in h.latest.blockers,1)
        h.device.suppress_status=False
        wait(lambda:'awaiting_disabled' not in h.latest.blockers)
        assert not h.latest.armed and not h.latest.owner
        assert not any(h.device.states[i*10+1] for i in range(N))


def test_parameter_report_uses_bridge_request_id_namespace(tmp_path):
    with ManagedNativeFixture(tmp_path,domain=189) as h:
        reports=[];results=[]
        h.device.create_subscription(SystemCommand,'/robot/managed/maintenance/system_command',reports.append,10)
        h.device.create_subscription(ConfigResult,'/motor/config_result',results.append,10)
        wait(lambda:any(m.command==SystemCommand.COMMAND_REPORT for m in reports))
        # Automatic reads delegate allocation to the bridge (zero sentinel),
        # rather than guessing its wire request high-water after a restart.
        assert all(m.request_id==0 for m in reports)
        invalid=ConfigRequest(sync_time_ns=h.device.get_clock().now().nanoseconds,session_id=100,
            request_id=0x7ffffff0,kind=1,command=SystemCommand.COMMAND_REPORT,motor_id=65535)
        refused=h.call(ExecuteMaintenance,'execute_maintenance',ExecuteMaintenance.Request(
            epoch=h.latest.epoch,session_id=100,request=invalid),success=False)
        assert not refused.success
        request=ConfigRequest(sync_time_ns=h.device.get_clock().now().nanoseconds,session_id=100,
            request_id=0x3ffffffe,kind=1,command=SystemCommand.COMMAND_REPORT,motor_id=65535)
        h.call(ExecuteMaintenance,'execute_maintenance',ExecuteMaintenance.Request(
            epoch=h.latest.epoch,session_id=100,request=request))
        wait(lambda:any(r.request_id==request.request_id and r.status==ConfigResult.STATUS_SUCCEEDED for r in results))
        queries=[m for m in h.device.configs if m.operation==AgentConfigRequest.QUERY_INFO]
        assert queries and all(m.session_id==100 and 0<=m.request_id<0x40000000 for m in queries)
        assert all(m.query_id>0 for m in queries)
