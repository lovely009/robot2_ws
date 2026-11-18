import { afterEach, beforeEach, expect, it, vi } from "vitest";

beforeEach(() => {
  vi.resetModules();
  vi.stubGlobal("location", { search: "" });
  vi.stubGlobal("localStorage", { getItem: () => null, setItem() {} });
  vi.spyOn(console, "warn").mockImplementation(() => {});
});
afterEach(() => {
  vi.unstubAllGlobals();
  vi.restoreAllMocks();
});

it("保留拓扑只读状态及差动组，不生成旧版一对一映射", async () => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue({ok:true,json:async()=>({
    model:'half_body_robot', mapping_kind:'topology', hardware_ready:false,
    hardware_block_reason:'传动参数待标定', driver_file:'', direct_joints:{},
    groups:[{name:'waist',motors:[{id:2},{id:3}],joints:['pitch','roll'],direct_mapping:false}],
  })}));
  const {store, loadModelJoints, jointMapSource, JOINT_OF_MOTOR} = await import('../../web/js/core/store.js');
  await loadModelJoints();
  expect(jointMapSource.kind).toBe('topology');
  expect(store.modelTransmissions.hardware_ready).toBe(false);
  expect(store.modelTransmissions.groups[0].motors.map((m:any)=>m.id)).toEqual([2,3]);
  expect(JOINT_OF_MOTOR).toEqual({});
});

it("电机出现后汇总持续使用 MCU 状态更新，不停留在空列表", async () => {
  vi.spyOn(Date, "now").mockReturnValue(1000);
  const { countMotors, store } = await import("../../web/js/core/store.js");
  const { applyControlState } = await import("../../web/js/core/motor_state.js");
  expect(countMotors()).toMatchObject({ total: 0, online: 0 });
  const states = [1, 2].map(motor_id => ({
    motor_id, online: true, control_lifecycle: 2, execution_state: 1,
    policy_enabled: false, motion_enabled: false, state_age_ms: 5,
    unified_fault_code: 0, vendor_fault_code: 0, last_failure_reason: 0,
    policy_generation: 1,
  }));
  store.conn.connected = true;
  for (const { motor_id } of states) store.motors.set(motor_id, {});
  expect(applyControlState(store.controlStates,
    { schema_version: 2, session_id: 100, sequence: 1, states })).toBe(true);
  expect(countMotors()).toEqual({ total: 2, online: 2, active: 0, fault: 0, conflict: 0 });
  states[1]!.online = false;
  expect(applyControlState(store.controlStates,
    { schema_version: 2, session_id: 100, sequence: 2, states })).toBe(true);
  expect(countMotors()).toMatchObject({ total: 2, online: 1, active: 0 });
});

it("曲线持续接收不同频率的通道时保留各自最新的有界样本", async () => {
  const { chartPush, store } = await import("../../web/js/core/store.js");
  for (let i = 0; i < 2000; i++) {
    chartPush(1, 'position', i, i * 20);
    chartPush(1, 'velocity', i + 0.1, i * 20);
    chartPush(1, 'effort', i + 0.2, i * 20);
    if (i % 5 === 0) chartPush(1, 'current', i + 0.3, i * 20 + 1);
  }
  const channels = store.chartBuf.get(1);
  expect(channels.position.t).toHaveLength(1500);
  expect(channels.position.t[0]).toBe(10000);
  expect(channels.position.values[0]).toBe(500);
  expect(channels.position.values.at(-1)).toBe(1999);
  expect(channels.current.t).toHaveLength(400);
  expect(channels.current.t.at(-1)).toBe(39901);
  expect(channels.current.values.at(-1)).toBe(1995.3);
});

it("模型无映射时清除上一模型映射，不将上一模型设备配给新模型", async () => {
  const fetcher = vi
    .fn()
    .mockResolvedValueOnce({
      ok: true,
      json: async () => ({ model: "half_body_robot", direct_joints: { 1: "joint_a" }, groups: [] }),
    })
    .mockResolvedValueOnce({ ok: false, status: 404 });
  vi.stubGlobal("fetch", fetcher);
  const store = await import("../../web/js/core/store.js");
  await store.loadModelJoints();
  expect(store.JOINT_OF_MOTOR).toEqual({ 1: "joint_a" });
  store.setModel("preview_only");
  await vi.waitFor(() => expect(store.jointMapSource.kind).toBe("unavailable"));
  expect(store.JOINT_OF_MOTOR).toEqual({});
});

it("晚到的旧模型响应不覆盖当前模型映射", async () => {
  let first!: (response: unknown) => void;
  const fetcher = vi
    .fn()
    .mockImplementationOnce(() => new Promise((resolve) => (first = resolve)))
    .mockResolvedValue({
      ok: true,
      json: async () => ({ model: store.MODEL, direct_joints: { 9: "new_joint" }, groups: [] }),
    });
  vi.stubGlobal("fetch", fetcher);
  const store = await import("../../web/js/core/store.js");
  const pending = store.loadModelJoints();
  store.setModel("new_model");
  await vi.waitFor(() =>
    expect(store.JOINT_OF_MOTOR).toEqual({ 9: "new_joint" }),
  );
  first({ ok: true, json: async () => ({ model: "half_body_robot", direct_joints: { 1: "old_joint" }, groups: [] }) });
  await pending;
  expect(store.JOINT_OF_MOTOR).toEqual({ 9: "new_joint" });
});

it("传动组 metadata 保留耦合关系，选择不被压平成 motor 对 joint", async () => {
  const groups = [
    {
      name: "wrist",
      motors: [{ name: "m8", id: 8 }, { name: "m9", id: 9 }],
      joints: ["yaw", "pitch"],
      direct_mapping: false,
    },
    {
      name: "elbow",
      motors: [{ name: "m2", id: 2 }],
      joints: ["elbow_joint"],
      direct_mapping: true,
    },
  ];
  vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
    ok: true,
    json: async () => ({
      model: "half_body_robot",
      driver_file: "/tmp/motor_driver.yaml",
      groups,
      direct_joints: { 2: "elbow_joint" },
    }),
  }));
  const store = await import("../../web/js/core/store.js");
  await store.loadModelJoints();
  expect(store.JOINT_OF_MOTOR).toEqual({ 2: "elbow_joint" });
  expect(store.store.modelTransmissions).toMatchObject({
    model: "half_body_robot",
    driver_file: "/tmp/motor_driver.yaml",
    direct_mapping: { 2: "elbow_joint" },
    error: "",
  });
  expect(store.store.modelTransmissions.groups).toEqual(groups);
  store.selectGroups(["wrist"]);
  store.selectJoint("pitch");
  expect(store.store.selectedGroup).toBe("wrist");
  expect(store.store.selectedJoint).toBe("pitch");
  store.selectJoint("invented_joint");
  expect(store.store.selectedJoint).toBe("");
});

const commandFrame = () => ({model:'half_body_robot',joint_names:['elbow'],mode:'stopped',
  output_topic:'/motor/host_command_array',
  trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
  gravity_enabled:null,effort_gain:null,feedback_fresh:false,controllers_ready:false,active_controllers:[]});

it.each([
  {feedback_fresh:1}, {controllers_ready:'true'}, {active_controllers:[1]},
  {joint_names:['elbow','elbow']}, {joint_names:['']}, {gravity_enabled:0}, {effort_gain:NaN},
])('坏控制帧撤销身份且不刷新时间 %j', async patch => {
  const {store,applyCommandStatus} = await import('../../web/js/core/store.js');
  applyCommandStatus(commandFrame());
  expect(store.commandStatus.last).toBeGreaterThan(0);
  applyCommandStatus({...commandFrame(),...patch});
  expect(store.commandStatus).toEqual({data:null,last:0});
});

it('传动状态接受未知扩展、null限位和未知重力，拒绝坏业务字段', async () => {
  const data = await import('../../web/js/core/store.js');
  const frame = {...commandFrame(),trajectory_enabled:true,
    control_groups:[{name:'direct',joints:['elbow'],motors:[{id:1}]}],
    dynamic_joint_names:['elbow'],joint_limits:{elbow:{lower:null,upper:null}},
    joint_limit_errors:{elbow:'反馈未知'},joint_limit_error:'反馈未知',future_extension:{v:1}};
  const legacy={model:'arm',joint_names:['elbow'],trajectory_enabled:false,
    joint_limits:{elbow:{lower:null,upper:null}}};
  data.applyTransmissionStatus(legacy);
  expect(data.store.transmissionStatus.data).toEqual(legacy);
  data.applyTransmissionStatus(frame);
  expect(data.store.transmissionStatus.data).toEqual(frame);
  for (const patch of [{joint_names:['']},{control_groups:[{name:'bad',joints:['elbow'],motors:[{id:'1'}]}]},
    {dynamic_joint_names:['other']},{joint_limits:{elbow:{lower:'0',upper:1}}},
    {joint_limit_errors:{elbow:false}},{trajectory_enabled:1},{feedback_fresh:1},
    {controllers_ready:'true'},{active_controllers:[1]}]) {
    data.applyTransmissionStatus({...frame,...patch});
    expect(data.store.transmissionStatus).toEqual({data:null,last:0});
  }
});

it.each([null,'start','stop','gain'])('台架重力合法 pending=%j 保留状态和接收时间', async pending => {
  vi.spyOn(Date, 'now').mockReturnValue(1000);
  const {store,applyBenchStatus} = await import('../../web/js/core/store.js');
  const gravity = {pending,fresh:true,error:'',config:{model:'arm'},extra:42};
  applyBenchStatus({gravity_feedforward:gravity});
  expect(store.bench.gravity_feedforward).toEqual(gravity);
  expect(store.bench.gravity_feedforward_last).toBe(1000);
});

it.each([{pending:false},{pending:true},{pending:'false'},{pending:'unknown'},{fresh:1},{fresh:'true'}])(
  '坏台架重力帧 %j 撤销确认资格且清空时间', async patch => {
    vi.spyOn(Date, 'now').mockReturnValue(1000);
    const {store,applyBenchStatus} = await import('../../web/js/core/store.js');
    const gravity = {pending:null,fresh:true,error:'',config:{model:'arm'}};
    applyBenchStatus({gravity_feedforward:gravity});
    expect(store.bench.gravity_feedforward_last).toBe(1000);
    applyBenchStatus({gravity_feedforward:{...gravity,...patch}});
    expect(store.bench.gravity_feedforward).toBeNull();
    expect(store.bench.gravity_feedforward_last).toBe(0);
  },
);

it.each([{}, {fresh:false}, {pending:null}, {config:null}])('台架重力可选字段兼容 %j', async gravity => {
  vi.spyOn(Date, 'now').mockReturnValue(1000);
  const {store,applyBenchStatus} = await import('../../web/js/core/store.js');
  applyBenchStatus({gravity_feedforward:gravity});
  expect(store.bench.gravity_feedforward).toEqual(gravity);
  expect(store.bench.gravity_feedforward_last).toBe(1000);
});

it("停止渲染循环会取消待执行帧，卸载后不再重新排队", async () => {
  let next = 0;
  const frames = new Map<number, () => void>();
  vi.stubGlobal("requestAnimationFrame", (callback: () => void) => {
    frames.set(++next, callback);
    return next;
  });
  vi.stubGlobal("cancelAnimationFrame", (id: number) => frames.delete(id));
  const store = await import("../../web/js/core/store.js");
  const stop = store.runRenderLoop();
  expect(frames.size).toBe(1);
  expect(typeof stop).toBe("function");
  stop();
  expect(frames.size).toBe(0);
});
