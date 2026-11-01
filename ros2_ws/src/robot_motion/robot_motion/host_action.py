"""One-use supervised Action proxy to the host JTC; no MCU trajectory upload.

Cancellation asks JTC to hold and preserves the drive policy. Ownership loss is
handled separately by the supervisor's MCU lease revocation/disable barrier.
"""
import copy
import math
import time
import json
from concurrent.futures import ThreadPoolExecutor
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from motor_hardware_interface.workbench_control import WorkbenchModel


class HostMotionActionAdapter:
    def __init__(self, node, driver, urdf, authorized, stop):
        self.node, self._valid, self._stop = node, authorized, stop
        self.mapping = WorkbenchModel(driver, urdf)
        self._grant = self._active = self.context = None
        self._reserved = self._closed = False
        self._feedback = None
        self._feedback_at = -math.inf
        self._controllers_ready = False
        self._validation = ThreadPoolExecutor(max_workers=1, thread_name_prefix='trajectory_validation')
        self._compiled_goal = None
        group = ReentrantCallbackGroup()
        self._client = ActionClient(node, FollowJointTrajectory,
            '/host_control/trajectory_controller/follow_joint_trajectory', callback_group=group)
        self._list = node.create_client(ListControllers, '/host_control/controller_manager/list_controllers', callback_group=group)
        self._switch = node.create_client(SwitchController, '/host_control/controller_manager/switch_controller', callback_group=group)
        self._cancel_client = node.create_client(CancelGoal,
            '/robot/follow_joint_trajectory/_action/cancel_goal', callback_group=group)
        self._server = ActionServer(node, FollowJointTrajectory, '/robot/follow_joint_trajectory',
            execute_callback=self._execute, goal_callback=self._goal,
            handle_accepted_callback=self._accepted, cancel_callback=self._cancel,
            callback_group=group)
        self._sub = node.create_subscription(JointState, '/host_control/joint_states',
            self._observe, qos_profile_sensor_data)
        self._timer = node.create_timer(.02, self._tick)
        node.create_subscription(String, '/control/status', self._host_status, 10)

    def _host_status(self, message):
        try: self._controllers_ready = json.loads(message.data).get('controllers_ready') is True
        except (TypeError, ValueError): self._controllers_ready = False

    @property
    def busy(self): return self._active is not None or self._reserved

    def _observe(self, message):
        self._feedback, self._feedback_at = message, time.monotonic()

    def authorize_goal(self, goal_id, context, expires_in_s=.5):
        key = bytes(goal_id)
        if len(key) != 16 or not any(key) or not 0 < expires_in_s <= 1:
            raise ValueError('invalid one-use motion authorization')
        if self._closed or self.busy or not self._valid(context):
            raise ValueError('motion requires current armed MCU authority')
        self._grant = key, copy.deepcopy(context), time.monotonic() + expires_in_s

    def _grant_valid(self):
        return self._grant and time.monotonic() < self._grant[2] and self._valid(self._grant[1])

    def _compile(self, goal):
        if self._feedback is None or time.monotonic() - self._feedback_at > .2:
            raise ValueError('fresh measured joint feedback required')
        now = self.node.get_clock().now().nanoseconds
        measured = self.mapping.feedback(self._feedback, now)
        out = copy.deepcopy(goal)
        out.trajectory = self.mapping.joint_trajectory(goal.trajectory, measured, now)
        return out

    async def _goal(self, goal):
        if self._closed or self.busy or not self._controllers_ready or not self._grant_valid() or not self._client.server_is_ready():
            return GoalResponse.REJECT
        # Model/segment validation can exceed a 20ms forwarding period for 37
        # axes. Yield this callback while validating, so the single state owner
        # continues forwarding freshly generated JTC samples and lease ACKs.
        # The worker does not mutate supervisor ownership or issue commands.
        self._reserved = True
        checked = Future(executor=self.node.executor)
        def completed(work):
            try: checked.set_result((work.result(), None))
            except Exception as error: checked.set_result((None, error))
        self._validation.submit(self._compile, goal).add_done_callback(completed)
        compiled, error = await checked
        if error or self._closed or not self._grant_valid():
            self._reserved = False
            return GoalResponse.REJECT
        self._compiled_goal = (compiled, time.monotonic())
        return GoalResponse.ACCEPT

    def _accepted(self, handle):
        self._reserved = False
        compiled = self._compiled_goal
        self._compiled_goal = None
        if (not self._grant_valid() or bytes(handle.goal_id.uuid) != self._grant[0]
                or compiled is None or time.monotonic()-compiled[1] > .2):
            handle.execute(); return
        context = self._grant[1]
        self._grant = None
        continuing = bool(self.context and self._valid(self.context) and all(
            self.context[key] == context[key] for key in ('lease_id','epoch','session_id','mcu_lease_id')))
        if not continuing: self.context = None
        active = {'handle':handle, 'context':context, 'future':Future(),
                  'remote':None, 'cancel':False, 'sent':time.monotonic(), 'continuing':continuing,
                  'submitted':False}
        self._active = active
        try:
            active['goal'] = compiled[0]
            # Retire all previous controller intent before binding a new owner.
            if not self._list.service_is_ready() or not self._switch.service_is_ready():
                raise ValueError('controller manager unavailable')
            self._list.call_async(ListControllers.Request()).add_done_callback(
                lambda done: self._prepare(active, done))
        except Exception as error:
            self._finish(active, 'abort', FollowJointTrajectory.Result.INVALID_GOAL, str(error))
        handle.execute()

    def _prepare(self, active, done):
        try:
            if not self._preflight(active): return
            controllers = done.result().controller
            names = {c.name: c for c in controllers}
            if 'trajectory_controller' not in names: raise ValueError('JTC missing')
            # A terminal hold from the SAME live owner can be replaced directly.
            # Restarting JTC would intentionally emit empty snapshots and soft
            # stop the MCU, then require a new explicit arm. A new owner still
            # goes through the full retirement barrier below.
            if (active['continuing'] and names['trajectory_controller'].state == 'active'
                    and not any(c.state == 'active' and (c.name == 'gravity_controller'
                        or c.name.startswith('manual_controller_')) for c in controllers)):
                self._dispatch(active)
                return
            retire = [c.name for c in controllers if c.state == 'active' and
                (c.name in ('trajectory_controller', 'gravity_controller') or c.name.startswith('manual_controller_'))]
            request = SwitchController.Request(deactivate_controllers=retire,
                strictness=SwitchController.Request.STRICT, timeout=Duration(sec=1))
            if retire:
                self._switch.call_async(request).add_done_callback(lambda result: self._activate(active, result))
            else: self._activate(active)
        except Exception as error:
            self._finish(active, 'abort', -1, str(error))

    def _activate(self, active, retired=None):
        try:
            if not self._preflight(active): return
            if retired is not None and not retired.result().ok: raise ValueError('old controllers did not retire')
            request = SwitchController.Request(activate_controllers=['trajectory_controller'],
                strictness=SwitchController.Request.STRICT, timeout=Duration(sec=1))
            self._switch.call_async(request).add_done_callback(lambda done: self._send(active, done))
        except Exception as error: self._finish(active, 'abort', -1, str(error))

    def _send(self, active, switched):
        try:
            if not switched.result().ok: raise ValueError('JTC activation failed')
            self._dispatch(active)
        except Exception as error: self._finish(active, 'abort', -1, str(error))

    def _dispatch(self, active):
        try:
            if not self._preflight(active): return
            active['submitted'] = True
            future = self._client.send_goal_async(active['goal'], goal_uuid=active['handle'].goal_id,
                feedback_callback=lambda message: active['handle'].publish_feedback(message.feedback)
                    if self._active is active and self._valid(active['context']) else None)
            future.add_done_callback(lambda done: self._remote_accepted(active, done))
        except Exception as error: self._finish(active, 'abort', -1, str(error))

    def _preflight(self, active):
        if self._active is not active: return False
        if not self._valid(active['context']): raise ValueError('goal authorization retired')
        if active['cancel']:
            self._finish(active, 'cancel', FollowJointTrajectory.Result.SUCCESSFUL, 'canceled before JTC dispatch')
            return False
        return True

    def _remote_accepted(self, active, done):
        try:
            remote = done.result()
            if not remote.accepted:
                self.context = None
                self._finish(active, 'abort', FollowJointTrajectory.Result.INVALID_GOAL, 'JTC rejected goal')
                return
            active['remote'] = remote
            if self._active is active and self._valid(active['context']) and not active['cancel']:
                self.context = active['context']
            if active['cancel'] or self._active is not active or not self._valid(active['context']):
                remote.cancel_goal_async()
            remote.get_result_async().add_done_callback(lambda result: self._result(active, result))
        except Exception as error:
            self._stop(active['context'], 'JTC acceptance outcome unknown')
            self.context = None
            self._finish(active, 'abort', FollowJointTrajectory.Result.INVALID_GOAL, str(error))

    def _result(self, active, done):
        try:
            result = done.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0
            outcome = 'success' if success else ('cancel' if result.status == GoalStatus.STATUS_CANCELED else 'abort')
            if not self._valid(active['context']): outcome = 'abort'
            # A successful endpoint / canceled hold may continue only under the
            # same explicit owner; no cached web/manual command is renewed here.
            self.context = active['context'] if outcome != 'abort' and self._valid(active['context']) else None
            self._finish(active, outcome, result.result.error_code, result.result.error_string)
        except Exception as error:
            self.context = None
            self._stop(active['context'], 'JTC result unknown')
            self._finish(active, 'abort', FollowJointTrajectory.Result.INVALID_GOAL, str(error))

    def _finish(self, active, outcome, code, text):
        if not active['future'].done():
            active['future'].set_result((outcome, FollowJointTrajectory.Result(error_code=code, error_string=text)))

    async def _execute(self, handle):
        active = self._active
        if active is None or active['handle'] is not handle:
            handle.abort()
            return FollowJointTrajectory.Result(error_code=-1, error_string='goal UUID has no one-use authorization')
        outcome, result = await active['future']
        if outcome == 'success': handle.succeed()
        elif outcome == 'cancel': handle.canceled()
        else: handle.abort()
        if self._active is active: self._active = None
        return result

    def cancel_current(self):
        active = self._active
        if active is None or not self._valid(active['context']) or not self._cancel_client.service_is_ready(): return False
        # Use the standard server cancellation transition even for a leased
        # facade request; directly setting a flag cannot put a server handle
        # into CANCELING and would make its terminal CANCELED transition invalid.
        request = CancelGoal.Request()
        request.goal_info.goal_id = active['handle'].goal_id
        self._cancel_client.call_async(request)
        return True

    def _cancel(self, handle):
        active = self._active
        if active is None or active['handle'] is not handle: return CancelResponse.REJECT
        active['cancel'] = True
        if active['remote'] is not None: active['remote'].cancel_goal_async()
        return CancelResponse.ACCEPT

    def _tick(self):
        active = self._active
        if active and active['cancel'] and not active['submitted']:
            self._finish(active, 'cancel', 0, 'canceled before JTC dispatch')
            return
        if active and (not self._valid(active['context']) or
                (active['remote'] is None and time.monotonic() - active['sent'] > 2)):
            if active['remote']: active['remote'].cancel_goal_async()
            self._stop(active['context'], 'JTC authority or acceptance lost')
            self.context = None
            self._finish(active, 'abort', -1, 'motion authority or JTC acceptance lost')
        if self.context and not self._valid(self.context): self.context = None

    def shutdown(self):
        self._closed = True
        self._validation.shutdown(wait=False, cancel_futures=True)
        if self._active and self._active['remote']: self._active['remote'].cancel_goal_async()
        self._timer.cancel()
        self._client.destroy()
        self._cancel_client.destroy()
        self._server.destroy()
