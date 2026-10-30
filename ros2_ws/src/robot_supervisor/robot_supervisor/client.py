"""Typed client for the managed robot interface.

There is no background renewal, command replay or automatic arming. A control
loop must renew explicitly (leases are 100..2000 ms) and publish fresh commands
at least at 10 Hz. Respect the supervisor's configured setpoint timeout; the
MCU stream itself is produced by the supervisor rather than by this SDK.

Synchronous methods may be called from an ordinary application thread. If a
supplied node already belongs to an executor, that executor must be running in
another thread: this client waits for responses and never spins it a second
time. Do not block that executor's own callback with a synchronous SDK call.
An unattached node is temporarily driven by a private executor for each wait.
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
import threading
import time
import uuid


class ControlError(RuntimeError):
    """A supervisor rejected an operation or the client has no usable lease."""


class ControlTimeout(TimeoutError):
    """Discovery and service response exceeded the shared request budget."""


def _uint(value, bits=64, minimum=0):
    return type(value) is int and minimum <= value < 2**bits


class ControlClient:
    """Explicit full-robot lease client; ``close()`` only releases ROS handles.

    Service methods return their typed response or raise ``ControlError`` on
    rejection. A timeout does not prove that a request was never applied; do
    not blindly retry an acquire or arm. Inspect state, or allow the short
    lease to expire. Call release/disarm/stop explicitly as appropriate.
    """

    def __init__(self, owner, mode=1, lease_ms=1000, namespace='/robot',
                 node=None, timeout_sec=2.0):
        if (not isinstance(owner, str) or not owner.strip() or len(owner) > 128
                or any(ord(char) < 32 for char in owner)):
            raise ValueError('owner must be nonempty printable text (at most 128 characters)')
        if type(mode) is not int or mode not in (1, 2, 3):
            raise ValueError('mode must be MANUAL=1, TRAJECTORY=2 or TASK=3')
        self._validate_lease_ms(lease_ms)
        if (not isinstance(namespace, str) or not namespace.startswith('/')
                or '//' in namespace):
            raise ValueError('namespace must be an absolute ROS namespace')
        if (not isinstance(timeout_sec, (int, float)) or isinstance(timeout_sec, bool)
                or not 0 < timeout_sec <= 2.0):
            raise ValueError('timeout_sec must be in (0, 2] seconds')

        # Keep imports lazy: command-line help and callers that merely import
        # the SDK do not require a configured ROS runtime.
        import rclpy
        from rclpy.context import Context
        from robot_system_msgs import msg, srv
        from std_srvs.srv import Trigger
        from can_bridge_msgs import msg as motor_msg
        from rclpy.validate_namespace import validate_namespace

        validate_namespace(namespace.rstrip('/') or '/')

        self.owner, self.mode, self.lease_ms = owner, mode, lease_ms
        self.namespace = namespace.rstrip('/')
        self.timeout_sec = float(timeout_sec)
        self.lease_id = ''
        self.epoch = 0
        self.session_id = 0
        self.sequence = 0
        self._lease_deadline = None
        self._rclpy, self._messages, self._services = rclpy, msg, srv
        self._motor_messages = motor_msg
        self._trigger = Trigger
        self._own_node = node is None
        self._context = None
        self._closed = False
        self._clients = {}
        self._publishers = {}
        self._action_client = None
        self._lock = threading.RLock()
        if node is None:
            self._context = Context()
            rclpy.init(args=[], context=self._context)
            try:
                node = rclpy.create_node(
                    'robot_control_client_' + uuid.uuid4().hex[:8], context=self._context)
            except Exception:
                self._context.shutdown()
                raise
        self.node = node
        # Advertise before acquire/arm discovery. Low-rate policy and trajectory
        # intents must not be silently sent before their reader is discovered.
        for channel, envelope in (
                ('command', msg.LeasedCommand), ('policy', msg.LeasedPolicy),
                ('trajectory', msg.LeasedTrajectory)):
            self._publishers[channel] = node.create_publisher(
                envelope, self._name(f'control/{channel}'),
                1 if channel == 'command' else 10)

    @staticmethod
    def _validate_lease_ms(value):
        if not _uint(value, 32, 100) or value > 2000:
            raise ValueError('lease_ms must be an integer in [100, 2000]')

    def _name(self, suffix):
        return f'{self.namespace}/{suffix}'

    def _ensure_open(self):
        if self._closed:
            raise ControlError('client is closed')

    def _wait(self, future, deadline):
        """Drive an unattached node, or wait for its existing executor."""
        remaining = max(0.0, deadline - time.monotonic())
        if self.node.executor is not None:
            done = threading.Event()
            future.add_done_callback(lambda _future: done.set())
            if future.done():
                return
            done.wait(remaining)
            return

        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor(context=self.node.context)
        try:
            executor.add_node(self.node)
            executor.spin_until_future_complete(future, timeout_sec=remaining)
        finally:
            executor.remove_node(self.node)
            # Humble's remove_node() does not clear Node.executor. A future may
            # keep this executor alive, so relying on its weakref expiring would
            # make the next service wait on an executor that is no longer spun.
            if self.node.executor is executor:
                self.node.executor = None
            executor.shutdown(timeout_sec=0.0)

    def _call(self, suffix, service_type, request):
        self._ensure_open()
        deadline = time.monotonic() + self.timeout_sec
        client = self._clients.get(suffix)
        if client is None:
            client = self.node.create_client(service_type, self._name(suffix))
            self._clients[suffix] = client
        if not client.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
            raise ControlTimeout(f'{self._name(suffix)} is unavailable')
        future = client.call_async(request)
        try:
            self._wait(future, deadline)
        finally:
            # The temporary executor may finish before call_async's scheduled
            # done callback runs. Remove completed requests explicitly as well.
            client.remove_pending_request(future)
        if not future.done():
            future.cancel()
            raise ControlTimeout(
                f'{self._name(suffix)} timed out; use synchronous calls outside executor callbacks')
        try:
            response = future.result()
        except Exception as error:
            raise ControlError(f'{self._name(suffix)} failed: {error}') from error
        if response is None:
            raise ControlError(f'{self._name(suffix)} returned no response')
        if not response.success:
            reason = getattr(response, 'reason', getattr(response, 'message', 'rejected'))
            raise ControlError(reason or 'supervisor rejected the request')
        return response

    def bind_lease(self, lease_id, epoch, session_id, sequence=0):
        """Use explicitly supplied credentials, e.g. in a separate CLI process.

        This does not acquire or renew a lease. The server remains authoritative
        for its expiry. A caller resuming publication must also resume its prior
        sequence rather than starting a second writer with the same lease.
        """
        if (not isinstance(lease_id, str) or not lease_id
                or not _uint(epoch, minimum=1) or not _uint(session_id, minimum=1)
                or not _uint(sequence)):
            raise ValueError('lease_id, nonzero uint64 epoch and session_id are required')
        with self._lock:
            self._ensure_open()
            self.lease_id, self.epoch, self.session_id = lease_id, epoch, session_id
            self.sequence = sequence
            self._lease_deadline = None

    def _require_lease(self, *, fresh=False):
        self._ensure_open()
        if not self.lease_id or self.epoch == 0 or self.session_id == 0:
            raise ControlError('acquire or bind a lease first')
        if (fresh and self._lease_deadline is not None
                and time.monotonic() >= self._lease_deadline):
            raise ControlError('local lease deadline has expired; acquire control again')

    def acquire(self):
        with self._lock:
            request = self._services.AcquireControl.Request()
            request.owner, request.mode, request.lease_ms = self.owner, self.mode, self.lease_ms
            started = time.monotonic()
            response = self._call('acquire_control', self._services.AcquireControl, request)
            self.bind_lease(response.lease_id, response.epoch, response.session_id)
            self.sequence = 0
            self.lease_ms = response.lease_ms
            self._lease_deadline = started + response.lease_ms / 1000.0
            return response

    def renew(self, lease_ms=None):
        with self._lock:
            self._require_lease(fresh=True)
            duration = self.lease_ms if lease_ms is None else lease_ms
            self._validate_lease_ms(duration)
            request = self._services.RenewControl.Request()
            request.lease_id, request.epoch, request.lease_ms = self.lease_id, self.epoch, duration
            started = time.monotonic()
            response = self._call('renew_control', self._services.RenewControl, request)
            self.lease_ms = duration
            self._lease_deadline = started + response.lease_remaining_ms / 1000.0
            return response

    def _set_armed(self, armed):
        with self._lock:
            self._require_lease(fresh=armed)
            request = self._services.SetArmed.Request()
            request.lease_id, request.epoch, request.armed = self.lease_id, self.epoch, armed
            return self._call('set_armed', self._services.SetArmed, request)

    def arm(self):
        return self._set_armed(True)

    def disarm(self):
        return self._set_armed(False)

    def _forget_lease(self):
        self.lease_id, self.epoch, self.session_id = '', 0, 0
        self.sequence = 0
        self._lease_deadline = None

    def release(self):
        with self._lock:
            self._require_lease()
            request = self._services.ReleaseControl.Request()
            request.lease_id, request.epoch = self.lease_id, self.epoch
            response = self._call('release_control', self._services.ReleaseControl, request)
            self._forget_lease()
            return response

    def stop(self):
        with self._lock:
            response = self._call('stop', self._trigger, self._trigger.Request())
            self._forget_lease()
            return response

    def reset(self):
        """Reset the supervisor fault latch; this never arms the robot."""
        with self._lock:
            return self._call('reset_fault', self._trigger, self._trigger.Request())

    def reset_fault(self):
        return self.reset()

    def _publish(self, channel, payload, payload_type, envelope_type):
        with self._lock:
            self._require_lease(fresh=True)
            if not isinstance(payload, payload_type):
                raise TypeError(f'{channel} requires {payload_type.__name__}')
            if self.sequence >= 2**64 - 1:
                raise ControlError('lease sequence exhausted; acquire a new lease')
            publisher = self._publishers[channel]
            discovery_deadline = time.monotonic() + self.timeout_sec
            while publisher.get_subscription_count() == 0:
                self._require_lease(fresh=True)
                remaining = discovery_deadline - time.monotonic()
                if remaining <= 0:
                    raise ControlTimeout(f'{self._name(f"control/{channel}")} has no reader')
                time.sleep(min(0.01, remaining))
            self._require_lease(fresh=True)
            now = self.node.get_clock().now()
            message = envelope_type()
            message.header.stamp = now.to_msg()
            message.lease_id, message.epoch = self.lease_id, self.epoch
            message.session_id = self.session_id
            message.sequence = self.sequence + 1
            outgoing = copy.deepcopy(payload)
            outgoing.sync_time_ns = now.nanoseconds
            if hasattr(outgoing, 'session_id'):
                outgoing.session_id = self.session_id
            setattr(message, channel, outgoing)
            publisher.publish(message)
            self.sequence = message.sequence
            return message

    def publish_command(self, command):
        return self._publish('command', command, self._motor_messages.MotorCommandArray,
                             self._messages.LeasedCommand)

    def publish_policy(self, policy):
        return self._publish('policy', policy, self._motor_messages.MotorPolicyArray,
                             self._messages.LeasedPolicy)

    def publish_trajectory(self, trajectory):
        return self._publish('trajectory', trajectory, self._motor_messages.MotorTrajectory,
                             self._messages.LeasedTrajectory)

    def authorize_motion(self, goal_id):
        """Bind one standard Action UUID to the current explicitly armed lease.

        The service consumes the same sequence as leased topic inputs. It does
        not acquire, renew, arm, or enable anything. A response timeout consumes
        the local sequence too, because the server may have accepted the request.
        """
        raw = goal_id.bytes if isinstance(goal_id, uuid.UUID) else bytes(goal_id)
        if len(raw) != 16 or not any(raw):
            raise ValueError('goal_id must be a nonzero 16-byte UUID')
        with self._lock:
            self._require_lease(fresh=True)
            if self.mode == 1:
                raise ControlError('standard motion requires TRAJECTORY or TASK mode')
            if self.sequence >= 2**64 - 1:
                raise ControlError('input sequence exhausted; acquire a new lease')
            request = self._services.AuthorizeMotion.Request()
            request.header.stamp = self.node.get_clock().now().to_msg()
            request.lease_id, request.epoch = self.lease_id, self.epoch
            request.session_id = self.session_id
            self.sequence += 1
            request.sequence, request.goal_id = self.sequence, list(raw)
            return self._call('authorize_motion', self._services.AuthorizeMotion, request)

    def follow_joint_trajectory(self, goal, feedback_callback=None):
        """Authorize one UUID and send a standard FollowJointTrajectory goal.

        Returns the standard rclpy ClientGoalHandle. The caller explicitly
        renews its control lease and drives its executor while awaiting results.
        The MCU supports complete position-only trajectories; see robot_motion.
        No retry or motion replay occurs after discovery/response failure.
        """
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from unique_identifier_msgs.msg import UUID
        if not isinstance(goal, FollowJointTrajectory.Goal):
            raise TypeError('goal must be control_msgs.action.FollowJointTrajectory.Goal')
        with self._lock:
            self._require_lease(fresh=True)
            if self._action_client is None:
                self._action_client = ActionClient(
                    self.node, FollowJointTrajectory, self._name('follow_joint_trajectory'))
            if not self._action_client.wait_for_server(timeout_sec=self.timeout_sec):
                raise ControlTimeout('standard trajectory action is unavailable')
            goal_id = uuid.uuid4()
            self.authorize_motion(goal_id)
            future = self._action_client.send_goal_async(
                copy.deepcopy(goal), feedback_callback=feedback_callback,
                goal_uuid=UUID(uuid=list(goal_id.bytes)))
            self._wait(future, time.monotonic() + self.timeout_sec)
            if not future.done():
                raise ControlTimeout('goal response timed out; inspect state before issuing another goal')
            handle = future.result()
            if handle is None or not handle.accepted:
                raise ControlError('standard trajectory goal was rejected')
            return handle

    def status(self):
        with self._lock:
            self._ensure_open()
            from rclpy.task import Future
            future = Future()
            def receive(message):
                if not future.done():
                    future.set_result(message)
            subscription = self.node.create_subscription(
                self._messages.RobotState, self._name('state'), receive, 1)
            try:
                self._wait(future, time.monotonic() + self.timeout_sec)
                if not future.done():
                    future.cancel()
                    raise ControlTimeout(f'{self._name("state")} is unavailable')
                return future.result()
            finally:
                self.node.destroy_subscription(subscription)

    def close(self):
        with self._lock:
            if self._closed:
                return
            for publisher in self._publishers.values():
                self.node.destroy_publisher(publisher)
            for client in self._clients.values():
                self.node.destroy_client(client)
            if self._action_client is not None:
                self._action_client.destroy()
                self._action_client = None
            self._publishers.clear()
            self._clients.clear()
            if self._own_node:
                self.node.destroy_node()
                if self._context.ok():
                    self._context.shutdown()
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/robot')
    parser.add_argument('--timeout', type=float, default=2.0)
    commands = parser.add_subparsers(dest='operation', required=True)
    commands.add_parser('status')
    acquire = commands.add_parser('acquire')
    acquire.add_argument('--owner', required=True)
    acquire.add_argument('--mode', choices=('manual', 'trajectory', 'task'), default='manual')
    acquire.add_argument('--lease-ms', type=int, default=1000)
    for name in ('renew', 'arm', 'disarm', 'release'):
        command = commands.add_parser(name)
        command.add_argument('--lease-id', required=True)
        command.add_argument('--epoch', type=int, required=True)
        command.add_argument('--session-id', type=int, required=True)
        if name == 'renew':
            command.add_argument('--lease-ms', type=int, default=1000)
    commands.add_parser('stop')
    commands.add_parser('reset')
    return parser


def main(args=None):
    options = _parser().parse_args(args)
    mode = {'manual': 1, 'trajectory': 2, 'task': 3}.get(getattr(options, 'mode', ''), 1)
    try:
        with ControlClient(getattr(options, 'owner', 'robot_control_cli'), mode,
                           getattr(options, 'lease_ms', 1000), options.namespace,
                           timeout_sec=options.timeout) as client:
            if hasattr(options, 'lease_id'):
                client.bind_lease(options.lease_id, options.epoch, options.session_id)
            response = getattr(client, options.operation)()
            from rosidl_runtime_py.convert import message_to_ordereddict
            print(json.dumps(message_to_ordereddict(response), ensure_ascii=False))
        return 0
    except (ControlError, ControlTimeout, ValueError, ImportError) as error:
        print(json.dumps({'success': False, 'reason': str(error)}, ensure_ascii=False),
              file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
