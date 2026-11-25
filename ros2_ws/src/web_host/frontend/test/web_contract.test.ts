import { afterEach, beforeEach, expect, it, vi } from "vitest";

const { callService } = vi.hoisted(() => ({ callService: vi.fn() }));
vi.mock("../../web/js/core/ros.js", () => ({ callService }));

beforeEach(() => {
  vi.resetModules();
  vi.stubGlobal("location", { search: "" });
  vi.stubGlobal("localStorage", { getItem: () => null, setItem() {} });
  callService.mockReset().mockResolvedValue({ ok: true, status_json: "{}" });
});
afterEach(() => vi.unstubAllGlobals());

it("默认半身模型的初始映射包含两侧全部电机", async () => {
  const { MODEL, JOINT_OF_MOTOR } = await import("../../web/js/core/store.js");
  expect(MODEL).toBe("half_body_robot");
  expect(Object.keys(JOINT_OF_MOTOR).map(Number)).toEqual(
    Array.from({ length: 18 }, (_, index) => index + 1),
  );
  expect(Object.values(JOINT_OF_MOTOR)).toEqual([
    "joint_link1", "joint_link2", "joint_link3", "joint_link4", "joint_link5",
    "joint_link6", "joint_link7", "joint_link8", "joint_link9",
    "joint_link1.1", "joint_link2.2", "joint_link3.3", "joint_link4.4",
    "joint_link5.5", "joint_link6.6", "joint_link7.7", "joint_link8.8", "joint_link9.9",
  ]);
});

it("控制话题使用主机控制器入口和对应消息类型", async () => {
  const { TOPICS } = await import("../../web/js/core/format.js");
  expect(TOPICS).toMatchObject({
    state: { name: "/motor/state", type: "can_bridge_msgs/msg/MotorSystemState" },
    motion: { name: "/motor/motion", type: "can_bridge_msgs/msg/MotorMotionStateArray" },
    events: { name: "/motor/events", type: "can_bridge_msgs/msg/MotorEvent" },
    temp: { name: "/imu/temp_raw", type: "sensor_msgs/msg/Temperature" },
    trajectory: { name: "/control/trajectory", type: "trajectory_msgs/msg/JointTrajectory" },
    jointTarget: { name: "/control/joint_target", type: "sensor_msgs/msg/JointState" },
    commandStatus: { name: "/control/status", type: "std_msgs/msg/String" },
    stop: { name: "/control/stop", type: "std_msgs/msg/Empty" },
  });
});

it.each([
  ["getBenchOptions", [], "options", "GetBenchOptions", {}],
  ["getAgentStatus", [], "agent_status", "GetAgentStatus", {}],
  ["startSimulation", [{ model: "arm" }], "start_simulation", "StartSimulation", { model: "arm" }],
  ["stopSimulation", [], "stop_simulation", "StopSimulation", {}],
  ["startAgentService", ["/dev/example"], "start_agent", "StartAgent", { device: "/dev/example" }],
  ["stopAgentService", [], "stop_agent", "StopAgent", {}],
  ["stopMotorBackend", [], "stop_motor_backend", "StopMotorBackend", {}],
  ["stopGravityFeedforward", [], "stop_gravity_feedforward", "StopGravityFeedforward", {}],
  ["setGravityFeedforwardGain", [.5], "set_gravity_feedforward_gain", "SetGravityFeedforwardGain", { effort_gain: .5 }],
  ["getUnitLog", ["bench-simulation.service", 20], "get_unit_log", "GetUnitLog", { unit_name: "bench-simulation.service", lines: 20 }],
  ["motorBackendCommand", [{ cmd: "status" }], "motor_backend_command", "MotorBackendCommand", { request_json: '{"cmd":"status"}' }],
] as const)("%s 提交对应服务类型和请求", async (method, args, name, type, request) => {
  const api = await import("../../web/js/core/agent.js");
  await api[method](...args);
  expect(callService).toHaveBeenCalledExactlyOnceWith(
    `/bench/${name}`, `bench_msgs/srv/${type}`, request,
  );
});

it("启动模拟电机时序列化通道配置", async () => {
  const { startMotorBackend } = await import("../../web/js/core/agent.js");
  await startMotorBackend({ model: "arm", transport: "socketcan", sim_interface: "can1" });
  expect(callService).toHaveBeenCalledExactlyOnceWith(
    "/bench/start_motor_backend", "bench_msgs/srv/StartMotorBackend", {
      model: "arm", transport: "socketcan", sim_interface: "can1", sim_tty: "",
      sim_channels_json: "", physical_bus_isolated: false,
    },
  );
});

it("重力补偿请求包含模型、关节映射和数值增益", async () => {
  const { startGravityFeedforward } = await import("../../web/js/core/agent.js");
  await startGravityFeedforward({ model: "arm", joint_map: "{}", effort_gain: "0.5" });
  expect(callService).toHaveBeenCalledExactlyOnceWith(
    "/bench/start_gravity_feedforward", "bench_msgs/srv/StartGravityFeedforward",
    { model: "arm", joint_map: "{}", effort_gain: .5 },
  );
});
