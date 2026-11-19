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
    setpoint: { name: "/control/setpoint", type: "can_bridge_msgs/msg/MotorCommandArray" },
    trajectory: { name: "/control/trajectory", type: "can_bridge_msgs/msg/MotorTrajectory" },
    rawTrajectory: { name: "/control/trajectory_raw", type: "can_bridge_msgs/msg/MotorTrajectory" },
    convertedTrajectory: { name: "/control/trajectory", type: "can_bridge_msgs/msg/MotorTrajectory" },
    jointTarget: { name: "/control/joint_target", type: "sensor_msgs/msg/JointState" },
    commandStatus: { name: "/control/status", type: "std_msgs/msg/String" },
    trajectoryState: { name: "/control/trajectory_state", type: "can_bridge_msgs/msg/MotorTrajectoryState" },
    stop: { name: "/control/stop", type: "std_msgs/msg/Empty" },
    policy: { name: "/motor/policy_array", type: "can_bridge_msgs/msg/MotorPolicyArray" },
    sysCmd: { name: "/motor/system_command", type: "can_bridge_msgs/msg/SystemCommand" },
  });
});

it.each([
  ["getBenchOptions", [], "options", "GetBenchOptions", {}],
  ["getAgentStatus", [], "agent_status", "GetAgentStatus", {}],
  ["startAgentService", ["/dev/example"], "start_agent", "StartAgent", { device: "/dev/example" }],
  ["stopAgentService", [], "stop_agent", "StopAgent", {}],
  ["stopGravityFeedforward", [], "stop_gravity_feedforward", "StopGravityFeedforward", {}],
  ["setGravityFeedforwardGain", [.5], "set_gravity_feedforward_gain", "SetGravityFeedforwardGain", { effort_gain: .5 }],
  ["getUnitLog", ["micro-ros-agent.service", 20], "get_unit_log", "GetUnitLog", { unit_name: "micro-ros-agent.service", lines: 20 }],
] as const)("%s 提交对应服务类型和请求", async (method, args, name, type, request) => {
  const api = await import("../../web/js/core/agent.js");
  await api[method](...args);
  expect(callService).toHaveBeenCalledExactlyOnceWith(
    `/bench/${name}`, `bench_msgs/srv/${type}`, request,
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

it("不导出已移除的仿真服务", async () => {
  const api = await import("../../web/js/core/agent.js");
  for (const name of ["startSimulation", "stopSimulation", "startMotorBackend", "stopMotorBackend", "motorBackendCommand"])
    expect(api).not.toHaveProperty(name);
});
