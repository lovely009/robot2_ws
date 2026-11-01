// Browser-side control intent only. MCU timing and final authority stay on the robot.
// Tokens live in memory and are never restored after a page/connection change.
export class ManagedControl {
  constructor({service, publish, changed = () => {}, now = () => performance.now(), wall = Date.now}) {
    Object.assign(this, {service, publish, changed, now, wall});
    this.state = null;
    this.stateAt = -Infinity;
    this.lease = null;
    this.sequence = 0;
    this.generation = 0;
    this.connected = false;
    this.visible = true;
    this.command = null;
    this.timer = null;
    this.renewing = false;
    this.acquiring = false;
    this.actionBusy = false;
    this.actionGoal = null;
    this.actionAccepted = this.cancelRequested = this.cancelSent = false;
    this.commandAt = -Infinity;
    this.message = '等待整机状态';
  }

  view() {
    return {state: this.state, pending: this.acquiring, ownsControl: !!this.lease,
      armed: !!this.lease && !!this.state?.armed,
      mcuAuthorized: !!this.lease && !!this.state?.mcu_authorized,
      actionBusy: this.actionBusy,
      message: this.message, model: this.model || ''};
  }

  update(state) {
    if (!Number.isSafeInteger(state?.epoch) || !Number.isSafeInteger(state?.session_id)) {
      this.suspend('整机标识超出浏览器精确整数范围');
      return;
    }
    const previousState = this.state;
    const previouslyAuthorized = this.state?.mcu_authorized;
    this.state = state;
    this.stateAt = this.now();
    if (this.lease && (this.lease.epoch !== state.epoch || this.lease.session_id !== state.session_id
        || (state.owner && state.owner !== this.owner) || state.fault_latched || !state.ready)) {
      this.suspend('整机控制会话已失效');
    }
    if (this.lease && state.armed && state.mcu_authorized && !previouslyAuthorized) {
      this.message = 'MCU 已确认授权；请显式下发使能策略';
    }
    if (!this.lease && (!previousState || previousState.ready !== state.ready || this.message.includes('连接已断开'))) {
      this.message = state.ready ? '整机已就绪；请显式申请控制权' : '整机未就绪，控制保持关闭';
    }
    this.changed();
  }

  setConnected(value) {
    if (value && !this.connected) this.message = '通信已连接；等待整机状态，不代表运动授权';
    this.connected = value;
    if (!value) { this.state = null; this.stateAt = -Infinity; this.suspend('连接已断开，请重新申请控制'); }
  }

  setVisible(value) {
    this.visible = value;
    if (!value) this.suspend('页面已离开前台，控制权已释放', true);
  }

  _clear() {
    this.generation++;
    this.lease = null;
    this.command = null;
    this.commandAt = -Infinity;
    this.actionBusy = false;
    this.actionGoal = null;
    this.actionAccepted = this.cancelRequested = this.cancelSent = false;
    this.renewing = false;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
  }

  suspend(reason, release = false) {
    const old = this.lease;
    this._clear();
    this.message = reason;
    this.changed();
    if (release && old && this.connected) {
      void this.service('/robot/release_control', 'robot_system_msgs/srv/ReleaseControl',
        {lease_id: old.lease_id, epoch: old.epoch}, 500).catch(() => {});
    }
  }

  _fresh() { return this.connected && this.visible && this.now() - this.stateAt <= 500; }

  _authorized(armed = true) {
    return this._fresh() && this.lease && this.now() < this.lease.deadline
      && this.state?.ready && !this.state.fault_latched
      && this.state.epoch === this.lease.epoch && this.state.session_id === this.lease.session_id
      && (!armed || (this.state.armed && this.state.mcu_authorized));
  }

  canSubmitMotion() { return !!this._authorized() && !this.actionBusy; }

  maintenance(key, payload) {
    if (key !== 'sysCmd' || ![1, 3, 4].includes(payload.command)) return false;
    const report = key === 'sysCmd' && payload.command === 4;
    if (!this._fresh() || (!report && (!this._authorized(false) || this.state.armed || this.actionBusy))) {
      this.message = '维护写入需要当前控制权、解除武装，并等待 MCU 失能反馈'; this.changed(); return false;
    }
    const request = {sync_time_ns:payload.sync_time_ns, session_id:this.state.session_id,
      request_id:payload.request_id, kind:1,
      count:payload.count || 0, targets:payload.targets || [], command:payload.command || 0,
      motor_id:payload.motor_id ?? 65535};
    void this._call('execute_maintenance', 'robot_system_msgs/srv/ExecuteMaintenance', {
      lease_id:this.lease?.lease_id || '', epoch:this.lease?.epoch || this.state.epoch,
      session_id:this.state.session_id, request,
    }).then(() => {this.message='维护请求已受理；等待设备事务终态';this.changed();})
      .catch(error => {this.message=String(error.message || error);this.changed();});
    return true;
  }

  async _call(name, type, request = {}) {
    const result = await this.service('/robot/' + name, type, request, 1000);
    if (!result.success) throw new Error(result.reason || result.message || '整机拒绝操作');
    return result;
  }

  async acquire() {
    if (!this._fresh() || !this.state?.ready) throw new Error('整机尚未就绪或状态已过期');
    if (this.lease || this.acquiring) throw new Error('本页面已持有或正在申请控制权');
    this.acquiring = true; this.changed();
    try {
    this.owner = 'web-' + Array.from(crypto.getRandomValues(new Uint8Array(8)), x => x.toString(16).padStart(2, '0')).join('');
    const generation = this.generation;
    const started = this.now();
    const result = await this._call('acquire_control', 'robot_system_msgs/srv/AcquireControl',
      {owner: this.owner, mode: 3, lease_ms: 1000});
    if (generation !== this.generation || !this.connected || !this.visible) {
      void this.service('/robot/release_control', 'robot_system_msgs/srv/ReleaseControl',
        {lease_id: result.lease_id, epoch: result.epoch}, 500).catch(() => {});
      throw new Error('页面会话已更换，未保留控制权');
    }
    if (!Number.isSafeInteger(result.epoch) || !Number.isSafeInteger(result.session_id)) {
      void this.service('/robot/release_control', 'robot_system_msgs/srv/ReleaseControl',
        {lease_id: result.lease_id, epoch: result.epoch}, 500).catch(() => {});
      throw new Error('控制标识无法精确表示');
    }
    this.lease = {...result, deadline: started + result.lease_ms};
    this.sequence = 0;
    this.renewAt = started + 300;
    this.timer = setInterval(() => this.tick(), 50);
    this.message = '已申请控制；请显式武装，再下发使能策略';
    this.changed();
    } finally { this.acquiring = false; this.changed(); }
  }

  async arm(armed) {
    if (!this._authorized(false)) throw new Error('请先申请有效控制权');
    this.command = null;
    const generation = this.generation;
    await this._call('set_armed', 'robot_system_msgs/srv/SetArmed',
      {lease_id: this.lease.lease_id, epoch: this.lease.epoch, armed});
    if (generation !== this.generation) return;
    this.message = armed ? '已请求武装；等待整机与 MCU 确认后下发使能' : '已请求解除武装；请核对 MCU 停止反馈';
    this.changed();
  }

  async release() {
    const lease = this.lease;
    this.suspend('已释放控制权，旧目标已清除');
    if (lease) await this._call('release_control', 'robot_system_msgs/srv/ReleaseControl',
      {lease_id: lease.lease_id, epoch: lease.epoch});
  }

  async stop() {
    this.suspend('已请求整机停止，等待 MCU 反馈');
    await this._call('stop', 'std_srvs/srv/Trigger');
  }

  async reset() {
    await this._call('reset_fault', 'std_srvs/srv/Trigger');
    this.message = '已复位；需要重新申请、武装与使能';
    this.changed();
  }

  _envelope(channel, payload) {
    const ms = this.wall();
    return {header: {stamp: {sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1000000}, frame_id: ''},
      lease_id: this.lease.lease_id, epoch: this.lease.epoch, session_id: this.lease.session_id,
      sequence: ++this.sequence, [channel]: structuredClone(payload)};
  }

  send(channel, payload) {
    if (!this._authorized()) {
      this.message = '请先申请控制、武装，并等待 MCU 授权确认'; this.changed(); return false;
    }
    if (!['command', 'policy', 'trajectory'].includes(channel)) return false;
    const disabling = channel === 'policy' && payload.targets?.length && payload.targets.every(t => !t.enabled);
    const canceling = channel === 'trajectory' && !payload.count && !payload.points?.length;
    if (this.actionBusy && canceling) {
      this.cancelRequested = true;
      this._cancelAction();
      return true;
    }
    if (this.actionBusy && !disabling && !canceling) {
      this.message = '轨迹正在执行，请先停止后再提交其他目标'; this.changed(); return false;
    }
    if (channel === 'command') {
      this.command = structuredClone(payload);
      this.command.sync_time_ns ||= this.wall() * 1000000;
      payload = this.command;
      this.commandAt = this.now();
    }
    else if (channel === 'trajectory' || (channel === 'policy' && payload.targets?.every(t => !t.enabled))) this.command = null;
    this.publish('/robot/control/' + channel,
      'robot_system_msgs/msg/Leased' + channel[0].toUpperCase() + channel.slice(1),
      this._envelope(channel, payload));
    return true;
  }

  _cancelAction() {
    if (!this.cancelRequested || this.cancelSent || !this.actionAccepted || !this.actionGoal) return;
    this.cancelSent = true;
    const generation=this.generation;
    const goal=this.actionGoal;
    void this.service('/robot/follow_joint_trajectory/_action/cancel_goal', 'action_msgs/srv/CancelGoal',
      {goal_info:{goal_id:{uuid:this.actionGoal},stamp:{sec:0,nanosec:0}}},1000).then(result => {
        if (generation !== this.generation || this.actionGoal !== goal || !this.actionBusy) return;
        this.message=result.return_code === 0 ? '已请求取消轨迹，等待终态；使能状态独立' : '取消未受理，请核对轨迹状态';
        this.changed();
      }).catch(error => {
        if (generation === this.generation && this.actionGoal === goal && this.actionBusy) {this.message='取消结果不确定：'+error.message;this.changed();}
      });
  }

  async sendJointTrajectory(trajectory) {
    if (!this._authorized()) throw new Error('请先申请控制并武装');
    if (this.actionBusy) throw new Error('轨迹正在执行，请先停止');
    if (trajectory.header?.stamp?.sec || trajectory.header?.stamp?.nanosec) {
      throw new Error('当前 MCU 轨迹不支持预约开始时间');
    }
    this.actionBusy = true;
    this.cancelRequested = this.cancelSent = this.actionAccepted = false;
    this.command = null;
    const generation = this.generation;
    let submitted = false;
    try {
    const uuid = Array.from(crypto.getRandomValues(new Uint8Array(16)));
    this.actionGoal = uuid;
    const envelope = this._envelope('trajectory', {});
    delete envelope.trajectory;
    await this._call('authorize_motion', 'robot_system_msgs/srv/AuthorizeMotion', {...envelope, goal_id: uuid});
    if (generation !== this.generation || !this._authorized()) throw new Error('轨迹授权已失效');
    // ROS 2 Action uses standard generated services; no motor conversion happens in the browser.
    const name = '/robot/follow_joint_trajectory';
    submitted = true;
    const accepted = await this.service(name + '/_action/send_goal',
      'control_msgs/action/FollowJointTrajectory_SendGoal', {goal_id: {uuid}, goal: {
        trajectory: {...trajectory, header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}},
        path_tolerance: [], goal_tolerance: [], goal_time_tolerance: {sec: 1, nanosec: 0},
      }}, 1000);
    if (!accepted.accepted) throw new Error('轨迹目标被拒绝');
    this.actionAccepted = true;
    this._cancelAction();
    this.message = '轨迹已受理，等待实际执行结果'; this.changed();
    const result = await this.service(name + '/_action/get_result',
      'control_msgs/action/FollowJointTrajectory_GetResult', {goal_id: {uuid}}, 10000);
    if (generation !== this.generation) return;
    this.message = result.status === 4 && result.result?.error_code === 0
      ? '轨迹执行成功，已通过到位检查' : result.status === 5
        ? '轨迹已取消；驱动使能状态以 MCU 反馈为准' : `轨迹未成功：${result.result?.error_string || result.status}`;
    this.changed();
    } catch (error) {
      if (generation === this.generation && submitted && this.connected) {
        // A lost service response does not prove that the goal was rejected.
        // Stop explicitly instead of silently leaving an unknown Action running.
        this.suspend('轨迹结果不确定，已请求整机停止');
        void this._call('stop', 'std_srvs/srv/Trigger').catch(() => {});
      }
      throw error;
    } finally {
      if (generation === this.generation) {
        this.actionBusy = false; this.actionGoal = null;
        this.actionAccepted = this.cancelRequested = this.cancelSent = false; this.changed();
      }
    }
  }

  tick() {
    if (!this._authorized(false)) { this.suspend('控制状态过期或控制权已失效', true); return; }
    if (this.command && this.now() - this.commandAt >= 200) {
      this.command = null;
      this.message = '手动目标已过期，请重新提交；关节运动请使用轨迹目标'; this.changed();
    }
    if (this.command && this._authorized()) {
      // Repeating a frame never refreshes the source intent's age.
      this.publish('/robot/control/command', 'robot_system_msgs/msg/LeasedCommand',
        this._envelope('command', this.command));
    }
    if (!this.renewing && this.now() >= this.renewAt) {
      const generation = this.generation;
      const sentAt = this.now();
      this.renewing = true;
      this.renewAt = sentAt + 300;
      void this._call('renew_control', 'robot_system_msgs/srv/RenewControl', {
        lease_id: this.lease.lease_id, epoch: this.lease.epoch, lease_ms: 1000,
      }).then(result => {
        if (generation === this.generation && this.lease) {
          if (!Number.isInteger(result.lease_remaining_ms) || result.lease_remaining_ms <= 0) {
            throw new Error('续租时长无效');
          }
          this.lease.deadline = sentAt + result.lease_remaining_ms;
        }
      }).catch(error => {
        if (generation === this.generation) this.suspend('续租失败：' + error.message, true);
      }).finally(() => { if (generation === this.generation) this.renewing = false; });
    }
  }
}
