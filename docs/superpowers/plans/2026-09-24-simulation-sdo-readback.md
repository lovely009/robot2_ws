# 仿真设备参数 SDO 回读实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 host + vcan 纯软件仿真通过真实 CANopen SDO 请求回读 CERB 虚拟驱动器参数，并在 WebHost 设备参数面提供与真机数据源隔离的只读视图。

**Architecture:** `motor_sim` 的虚拟 CERB 节点在 vcan 上应答上传请求；同包新增独立 ROS 网关 socket，串行执行每台电机的 SDO 事务并发布 `/motor_sim/*` 快照。`simulation.launch.py` 仅在 host-vcan 组合启动网关；WebHost 只在设备参数卡内切换真机和仿真，分别维护报告、状态、事件及回读序号。

**Tech Stack:** C++17、ROS 2 Humble / rclcpp、SocketCAN CAN-FD/BRS、yaml-cpp、can_bridge_msgs、Python 3 / pytest、静态 HTML/CSS/JavaScript。

**Spec:** `docs/superpowers/specs/2026-09-24-simulation-sdo-readback-design.md`

## Global Constraints

- 本功能只支持 `controller_mode:=host transport:=vcan`；其他控制模式、物理 SocketCAN、SLCAN 和 firmware HIL 不启动仿真 SDO 网关。
- 仿真只使用 `/motor_sim/system_command`、`/motor_sim/device_param_report`、`/motor_sim/system_status`、`/motor_sim/motor_events`；真机继续使用现有 `/micro_ros_up/*` 与 `/micro_ros_do/*`。
- `REFRESH` 每条命令必须带有效 `motor_id`；全量读取由页面逐台串行发起，不得用 `MOTOR_ID_NONE`。
- 仿真 CERB 对象按 0x607D:01/02、0x6080:00、0x2538:00、0x2650:00、0x2541:00 顺序逐项上传；一轮只允许一个未完成 SDO 请求，只对超时重试。
- 位置 cnt、速度 RPM、电流 mA、安全原值、方向原值分别按设计规格换算；未读值为 NaN，不能以 0 代替。
- VAXOR 显示“无设备对象”，不发 CERB SDO，也不伪造设备值。
- 真机页面行为、`motor_drive`、`motor_hardware_interface`、固件、micro-ROS Agent 及非设备参数面数据源保持不变。
- 自动测试不得启动 Gazebo 或真实硬件；修改前后均不得覆盖、暂存或回滚工作区中已有的用户改动。
- 新增与修改的说明使用中文；C++ 保持 C++17、2 空格缩进和 `-Wall -Wextra -Wpedantic`。

---

### Task 1: 增加 CERB expedited SDO 上传编解码

**Files:**
- Create: `ros2_ws/src/motor_sim/include/motor_sim/protocol/cerb_sdo.hpp`
- Create: `ros2_ws/src/motor_sim/src/protocol/cerb_sdo.cpp`
- Create: `ros2_ws/src/motor_sim/test/test_cerb_sdo.cpp`
- Modify: `ros2_ws/src/motor_sim/CMakeLists.txt`

**Interfaces:**
- Consumes: `motor_sim::protocol::CanFrame`，请求/应答均为 CAN-FD、BRS、8 字节。
- Produces: `SdoObjectAddress { uint16_t index; uint8_t subindex; bool operator<(const SdoObjectAddress &) const; }`（按 index、subindex 排序）；`SdoObjectDictionary = std::map<SdoObjectAddress, int32_t>`；`enum class SdoUploadStatus { Value, Abort, Unmatched, Malformed };`；`SdoUploadResult { SdoUploadStatus status; uint16_t motor_id; SdoObjectAddress object; int32_t raw_value; uint32_t abort_code; }`；函数 `std::optional<CanFrame> make_cerb_sdo_upload_request(uint16_t motor_id, SdoObjectAddress object)`、`std::optional<CanFrame> answer_cerb_sdo_upload_request(const CanFrame &request, const SdoObjectDictionary &objects)`、`SdoUploadResult parse_cerb_sdo_upload_response(const CanFrame &response, uint16_t motor_id, SdoObjectAddress expected)`。

- [ ] **Step 1: 先写覆盖请求、成功响应、ABORT 和忽略不匹配帧的测试**

在 `test_cerb_sdo.cpp` 写以下核心断言：

```cpp
TEST(CerbSdo, BuildsExpeditedUploadRequest) {
  const auto frame = make_cerb_sdo_upload_request(1U, {0x607DU, 1U});
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x601U);
  EXPECT_TRUE(frame->fd);
  EXPECT_TRUE(frame->brs);
  ASSERT_EQ(frame->size, 8U);
  EXPECT_EQ(frame->data[0], 0x40U);
  EXPECT_EQ(frame->data[1], 0x7DU);
  EXPECT_EQ(frame->data[2], 0x60U);
  EXPECT_EQ(frame->data[3], 1U);
}

TEST(CerbSdo, AnswersConfiguredObjectAndAbortsUnknownObject) {
  const SdoObjectDictionary objects{{{0x607DU, 1U}, -32768}};
  const auto request = make_cerb_sdo_upload_request(1U, {0x607DU, 1U});
  const auto response = answer_cerb_sdo_upload_request(*request, objects);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->id, 0x581U);
  EXPECT_EQ(response->data[0], 0x43U);
  EXPECT_EQ(response->data[4], 0x00U);
  EXPECT_EQ(response->data[5], 0x80U);
  EXPECT_EQ(response->data[6], 0xFFU);
  EXPECT_EQ(response->data[7], 0xFFU);

  const auto missing = make_cerb_sdo_upload_request(1U, {0x2541U, 0U});
  const auto abort = answer_cerb_sdo_upload_request(*missing, objects);
  ASSERT_TRUE(abort.has_value());
  EXPECT_EQ(abort->data[0], 0x80U);
  EXPECT_EQ(abort->data[4], 0x11U);
  EXPECT_EQ(abort->data[5], 0x00U);
  EXPECT_EQ(abort->data[6], 0x09U);
  EXPECT_EQ(abort->data[7], 0x06U);
}
```

再添加 parser 对 0x43、0x80、错误 CAN ID、错误 index/subindex、错误 DLC/命令字的断言。

- [ ] **Step 2: 注册测试并确认测试因接口缺失而失败**

只注册 `test_cerb_sdo` 并链接现有 `motor_protocol`，暂不把尚未创建的 `cerb_sdo.cpp` 加入库源列表，然后运行：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim --cmake-target test_cerb_sdo
```

预期：编译失败，错误指出 `cerb_sdo.hpp` 尚不存在。

- [ ] **Step 3: 实现精简编解码器**

实现时把 `src/protocol/cerb_sdo.cpp` 加回 `motor_protocol` 源列表。只接受 `0x40 index_lo index_hi sub 00 00 00 00` 上传请求；请求 ID 为 `0x600 + motor_id`，响应 ID 为 `0x580 + motor_id`。命中对象时用命令字 `0x43` 和有符号 little-endian int32 返回；对象不存在时返回命令字 `0x80`、ABORT `0x06090011`。非 FD/BRS、非 8 字节、非 expedited upload 或非法节点地址不生成应答。parser 只把预期 motor/index/subindex 的匹配应答标为 `Value` 或 `Abort`，其他响应不结束当前事务。

- [ ] **Step 4: 重跑协议测试并确认通过**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim
colcon test --packages-select motor_sim --ctest-args -R test_cerb_sdo
colcon test-result --verbose
```

预期：`test_cerb_sdo` 全部通过，其他既有测试不因新增协议源回归。

- [ ] **Step 5: 提交协议层**

```bash
git add ros2_ws/src/motor_sim/CMakeLists.txt ros2_ws/src/motor_sim/include/motor_sim/protocol/cerb_sdo.hpp ros2_ws/src/motor_sim/src/protocol/cerb_sdo.cpp ros2_ws/src/motor_sim/test/test_cerb_sdo.cpp
git commit -m "feat: add CERB SDO upload codec"
```

### Task 2: 将模型配置映射为仿真对象字典与基准

**Files:**
- Modify: `ros2_ws/src/motor_sim/include/motor_sim/core/config.hpp`
- Modify: `ros2_ws/src/motor_sim/src/core/config.cpp`
- Modify: `ros2_ws/src/motor_sim/test/test_config.cpp`
- Modify: `ros2_ws/src/motor_sim/test/fixtures/cerb_axis.yaml`
- Create: `ros2_ws/src/motor_sim/test/fixtures/vaxor_axis.yaml`
- Modify: `ros2_ws/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/config/simulation_topology.yaml`

**Interfaces:**
- Consumes: Task 1 的 `SdoObjectDictionary`。
- Produces: `struct SimulatedDeviceParams { double rec_pos_min_rad; double rec_pos_max_rad; double rec_velocity_rad_s; double rec_current_limit_a; uint32_t rec_safety_raw; uint32_t rec_direction_raw; ::motor_sim::protocol::SdoObjectDictionary expected_objects; ::motor_sim::protocol::SdoObjectDictionary sdo_objects; };`；`MotorConfig` 增加 `SimulatedDeviceParams device_params` 字段。CERB 默认对象由 profile 的 hard position limits、关节速度、`plant.current_limit_a`、安全原值和方向原值推导；`sdo_overrides` 数值覆盖设备原值，YAML null 从实际字典移除对象但保留配置基准。VAXOR 在电机绑定时清空 SDO 字典。

- [ ] **Step 1: 先写 profile 默认值、覆盖值和缺失对象的配置测试**

扩充 `cerb_axis.yaml`，在 CERB profile 增加：

```yaml
device_params:
  safety_raw: 0
  direction_raw: 0
  sdo_overrides:
    "607D:01": -12000
    "2541:00": null
```

在 `test_config.cpp` 增加测试，断言默认字典含 607D:01/02、6080:00、2538:00、2650:00、2541:00；覆盖的 607D:01 原值为 -12000；null 的 2541:00 不在实际字典但仍在 `expected_objects`；基准安全原值/方向分别为 0；将协议改为 VAXOR 时对象字典为空。

添加 `vaxor_axis.yaml`：复制 `cerb_axis.yaml` 的 timing、buses、profile，只把 motor protocol 改成 `vaxor`。在 `test_config.cpp` 加入：

```cpp
TEST(SystemConfig, AppliesCerbSdoDefaultsOverridesAndMissingObjects) {
  const auto config = core::load_system_config(fixture_path("cerb_axis.yaml"));
  const auto &params = config.motors.front().device_params;
  EXPECT_EQ(params.expected_objects.at({0x607DU, 1U}), -65536);
  EXPECT_EQ(params.expected_objects.at({0x607DU, 2U}), 65536);
  EXPECT_EQ(params.sdo_objects.at({0x607DU, 1U}), -12000);
  EXPECT_EQ(params.expected_objects.at({0x2541U, 0U}), 0);
  EXPECT_EQ(params.sdo_objects.count({0x2541U, 0U}), 0U);
  EXPECT_EQ(params.rec_safety_raw, 0U);
  EXPECT_EQ(params.rec_direction_raw, 0U);
}

TEST(SystemConfig, DoesNotCreateCerbObjectsForVaxor) {
  const auto config = core::load_system_config(fixture_path("vaxor_axis.yaml"));
  const auto &params = config.motors.front().device_params;
  EXPECT_TRUE(params.expected_objects.empty());
  EXPECT_TRUE(params.sdo_objects.empty());
}
```

- [ ] **Step 2: 确认新增断言先失败**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim --cmake-target test_config
```

预期：编译或测试失败，指出 `device_params` 类型/字段尚未存在。

- [ ] **Step 3: 实现配置解析和默认原值推导**

使用量化后的预期原值构造两套字典：位置 `round(rad * 65536 / (2π))` cnt，速度 `round(rad/s * gear_ratio * 60 / (2π))` RPM，峰值电流 `round(A * 1000)` mA；保护和方向用 32 位原值。`rec_*` 通过预期原值反算到消息单位，保证默认 SDO 回读与仿真配置基准在设备分辨率内一致。只允许已定义的六个对象键，拒绝未知键、非整数原值、越界安全/方向值及无效数值；未提供 `device_params` 时使用 `safety_raw=0`、`direction_raw=0`、无覆盖，保持旧配置可读。

- [ ] **Step 4: 在机器人仿真拓扑中声明 CERB 默认配置**

在 `simulation_topology.yaml` 的 CERB profile 增加 `device_params` 默认项（`safety_raw: 0`、`direction_raw: 0`、空 `sdo_overrides`）。保留 motor ID、protocol 和减速比来自真机拓扑绑定的现有规则。

- [ ] **Step 5: 运行配置测试并提交**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim
colcon test --packages-select motor_sim --ctest-args -R test_config
colcon test-result --verbose
```

预期：配置解析测试通过，旧 fixture 和拓扑契约测试仍通过。

```bash
git add ros2_ws/src/motor_sim/include/motor_sim/core/config.hpp ros2_ws/src/motor_sim/src/core/config.cpp ros2_ws/src/motor_sim/test/test_config.cpp ros2_ws/src/motor_sim/test/fixtures/cerb_axis.yaml ros2_ws/src/motor_sim/test/fixtures/vaxor_axis.yaml ros2_ws/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/config/simulation_topology.yaml
git commit -m "feat: configure simulated CERB device objects"
```

### Task 3: 让 MotorSimNode 在 vcan 上应答虚拟 SDO

**Files:**
- Modify: `ros2_ws/src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp`
- Modify: `ros2_ws/src/motor_sim/src/ros/motor_sim_node.cpp`
- Modify: `ros2_ws/src/motor_sim/test/test_motor_sim_node.cpp`
- Modify: `ros2_ws/src/motor_sim/README.md`

**Interfaces:**
- Consumes: `MotorConfig::protocol`、`MotorConfig::id`、`MotorConfig::bus` 和 Task 2 的 `device_params.sdo_objects`；Task 1 的 `answer_cerb_sdo_upload_request`。
- Produces: SocketCAN 接收帧进入 `FrameRouter::handle` 前的 CERB SDO 分发；只对配置中同 bus、同 motor ID 且 protocol 为 `cerb` 的节点应答。未匹配电机或 VAXOR 电机的 SDO 地址不应答，其他控制帧继续走现有路由。

- [ ] **Step 1: 给注入式测试 Transport 增加可排队收帧和已发送帧记录**

在 `test_motor_sim_node.cpp` 的 fake `FrameTransport` 中加入 `std::deque<transport::ReceivedFrame> incoming` 与 `std::vector<CanFrame> sent`；`receive()` 从队列取一帧，空队列返回 `NoData`，`send()` 保存帧并返回 true。
增加 `<deque>`、`<thread>` include。

使用以下 Transport 实现替换 `IdleTransport` 的测试用途：

```cpp
class CaptureTransport final : public transport::FrameTransport {
public:
  void open() override { opened = true; }
  void close() noexcept override { opened = false; }
  bool send(const motor_sim::protocol::CanFrame &frame) override {
    sent.push_back(frame);
    return opened;
  }
  transport::ReceiveResult receive(transport::ReceivedFrame &frame) override {
    if (incoming.empty()) return transport::ReceiveResult::NoData;
    frame = incoming.front();
    incoming.pop_front();
    return transport::ReceiveResult::Frame;
  }
  bool opened{false};
  std::deque<transport::ReceivedFrame> incoming;
  std::vector<motor_sim::protocol::CanFrame> sent;
};
```

- [ ] **Step 2: 写节点层回读测试并确认失败**

构造单 CERB 配置（ID 1，字典含 `{0x607D,1}: -32768`），向 fake transport 入队 Task 1 生成的上传请求；spin 节点 executor，断言发出一帧 `0x581`、命令字 `0x43`、值为 `-32768`。另以同 ID 的 VAXOR 配置发送 SDO 请求，断言没有 SDO 应答。先运行：

```cpp
TEST(MotorSimNodeSdo, RepliesOnlyForConfiguredCerbMotor) {
  ensure_rclcpp_initialized();
  auto config = one_axis_config();
  config.motors.front().protocol = "cerb";
  config.motors.front().device_params.sdo_objects = {{{0x607DU, 1U}, -32768}};
  const auto request = motor_sim::protocol::make_cerb_sdo_upload_request(
      1U, {0x607DU, 1U});
  ASSERT_TRUE(request);
  auto transport_impl = std::make_unique<CaptureTransport>();
  auto *capture = transport_impl.get();
  capture->incoming.push_back({*request, std::chrono::steady_clock::now()});
  ros::TransportMap transports;
  transports.emplace("arm", std::move(transport_impl));
  auto node = std::make_shared<ros::MotorSimNode>(
      rclcpp::NodeOptions{}, std::move(config), std::move(transports));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 100ms;
  while (capture->sent.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(capture->sent.size(), 1U);
  EXPECT_EQ(capture->sent.front().id, 0x581U);
  EXPECT_EQ(capture->sent.front().data[0], 0x43U);
  EXPECT_EQ(capture->sent.front().data[4], 0x00U);
  EXPECT_EQ(capture->sent.front().data[5], 0x80U);
  EXPECT_EQ(capture->sent.front().data[6], 0xFFU);
  EXPECT_EQ(capture->sent.front().data[7], 0xFFU);
  executor.remove_node(node);
  rclcpp::shutdown();
}

TEST(MotorSimNodeSdo, IgnoresVaxorSdoAddress) {
  ensure_rclcpp_initialized();
  auto config = one_axis_config();  // helper 默认 protocol=vaxor
  const auto request = motor_sim::protocol::make_cerb_sdo_upload_request(
      1U, {0x607DU, 1U});
  ASSERT_TRUE(request);
  auto transport_impl = std::make_unique<CaptureTransport>();
  auto *capture = transport_impl.get();
  capture->incoming.push_back({*request, std::chrono::steady_clock::now()});
  ros::TransportMap transports;
  transports.emplace("arm", std::move(transport_impl));
  auto node = std::make_shared<ros::MotorSimNode>(
      rclcpp::NodeOptions{}, std::move(config), std::move(transports));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 20ms;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(capture->sent.empty());
  executor.remove_node(node);
  rclcpp::shutdown();
}
```

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim --cmake-target test_motor_sim_node
colcon test --packages-select motor_sim --ctest-args -R test_motor_sim_node
```

预期：第一项行为失败，因为当前节点把 SDO 请求交给 FrameRouter 并丢弃。

- [ ] **Step 3: 在 `transport_tick()` 中先匹配 CERB SDO，再处理普通电机帧**

按 bus 和 CAN ID 找到唯一的 CERB motor config；识别该对象的有效 expedited upload 请求后调用 `answer_cerb_sdo_upload_request()`，发送成功后消费该帧，不交给 `FrameRouter`。匹配到虚拟 VAXOR、未知 ID 或非 SDO 帧时保持当前路由行为。发送失败沿用 `handle_disconnect(bus)`。

- [ ] **Step 4: 运行节点测试并更新 `motor_sim/README.md`**

运行：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim
colcon test --packages-select motor_sim --ctest-args -R test_motor_sim_node
colcon test-result --verbose
```

README 增加 CERB 对象上传应答、未知对象 ABORT、vcan 同接口双 socket 的说明；说明 VAXOR 和其他传输不提供该参数回读。说明 profile 的 `device_params.safety_raw` / `direction_raw` 是基准原值，`sdo_overrides` 用 `"607D:01"` 这样的对象键；整数表示覆盖原值，YAML `null` 表示模拟该对象不存在，未列出对象使用 profile 推导默认值。

- [ ] **Step 5: 提交虚拟设备应答路径**

```bash
git add ros2_ws/src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp ros2_ws/src/motor_sim/src/ros/motor_sim_node.cpp ros2_ws/src/motor_sim/test/test_motor_sim_node.cpp ros2_ws/src/motor_sim/README.md
git commit -m "feat: answer CERB SDO reads in motor simulator"
```

### Task 4: 实现串行回读序列和 `/motor_sim/*` 网关节点

**Files:**
- Create: `ros2_ws/src/motor_sim/include/motor_sim/core/sdo_read_sequence.hpp`
- Create: `ros2_ws/src/motor_sim/src/core/sdo_read_sequence.cpp`
- Create: `ros2_ws/src/motor_sim/test/test_sdo_read_sequence.cpp`
- Create: `ros2_ws/src/motor_sim/include/motor_sim/ros/sim_device_param_gateway.hpp`
- Create: `ros2_ws/src/motor_sim/src/ros/sim_device_param_gateway.cpp`
- Create: `ros2_ws/src/motor_sim/src/ros/sim_device_param_gateway_main.cpp`
- Create: `ros2_ws/src/motor_sim/test/test_sim_device_param_gateway.cpp`
- Modify: `ros2_ws/src/motor_sim/CMakeLists.txt`
- Modify: `ros2_ws/src/motor_sim/package.xml`

**Interfaces:**
- Consumes: Task 1 SDO parser/request函数；Task 2 的 `SystemConfig`/`MotorConfig::device_params`；`FrameTransport`；`can_bridge_msgs/msg/{SystemCommand,DeviceParamReport,SystemStatus,MotorEvent}`。
- Produces: `enum class SimDeviceInitState : uint8_t { NoDevice=0, Pending=1, Ok=2, Mismatch=3, Unverified=4, NoRecord=5 }; struct SdoReadResult { std::array<float,6> dev_values; uint8_t mismatch_mask; uint8_t unread_mask; SimDeviceInitState state; };`（`dev_values` 按消息槽位顺序保存位置下限、位置上限、速度、电流、安全原值、方向原值）；`SdoReadSequence(const MotorConfig &, std::chrono::milliseconds timeout, uint8_t retries)`，`using Clock = std::chrono::steady_clock;`，方法 `std::optional<::motor_sim::protocol::CanFrame> start(Clock::time_point now)`、`std::optional<::motor_sim::protocol::CanFrame> accept_response(const ::motor_sim::protocol::CanFrame &, Clock::time_point now)`、`std::optional<::motor_sim::protocol::CanFrame> advance_timeout(Clock::time_point now)` 各自返回“现在应发的下一帧”或空值，并提供 `bool complete() const noexcept` / `const SdoReadResult &result() const noexcept`；`explicit SimDeviceParamGatewayNode(const rclcpp::NodeOptions &options)` production 构造，以及 `SimDeviceParamGatewayNode(const rclcpp::NodeOptions &, ::motor_sim::core::SystemConfig, std::string bus, std::unique_ptr<::motor_sim::transport::FrameTransport>)` 测试构造。production 节点读取 `config_file`/`interface`/`sdo_timeout_ms`/`sdo_retries` ROS 参数，订阅 `SystemCommand`，发布独立报告、状态、事件。安装可执行名为 `sim_device_param_gateway`。

- [ ] **Step 1: 先写事务序列测试**

`test_sdo_read_sequence.cpp` 用固定时钟验证：首帧读取 607D:01；收到与 motor/index/sub 不匹配的响应不推进；正确值推进到 607D:02；ABORT 将当前槽记入 `unread_mask` 并立刻推进、不重试；超时先重发同一对象一次，再将槽标为 unread 并推进；六槽依次处理且任一时刻仅暴露一个待发送请求；全部值与预期原值相同得到状态 OK，不同时生成错误位图，原值覆盖不同得到 MISMATCH 位图。

测试文件顶部定义以下 namespace 别名，并包含 `<array>`、`<chrono>`、`<cstdint>`：

```cpp
namespace core = ::motor_sim::core;
namespace protocol = ::motor_sim::protocol;
namespace ros = ::motor_sim::ros;
namespace transport = ::motor_sim::transport;
using namespace std::chrono_literals;
```

成功读取用例按此构造配置和 fake SDO server：

```cpp
TEST(SdoReadSequence, ReadsSixObjectsInOrderAndConvertsUnits) {
  core::MotorConfig motor;
  motor.bus = "arm";
  motor.protocol = "cerb";
  motor.id = 1U;
  motor.transmission.gear_ratio = 101.0;
  const protocol::SdoObjectDictionary objects{
      {{0x607DU, 1U}, -32768}, {{0x607DU, 2U}, 32768},
      {{0x6080U, 0U}, 101}, {{0x2538U, 0U}, 2500},
      {{0x2650U, 0U}, 0}, {{0x2541U, 0U}, 1}};
  motor.device_params.expected_objects = objects;
  motor.device_params.sdo_objects = objects;
  core::SdoReadSequence sequence(motor, 150ms, 1U);
  const std::array<protocol::SdoObjectAddress, 6> order{{
      {0x607DU, 1U}, {0x607DU, 2U}, {0x6080U, 0U},
      {0x2538U, 0U}, {0x2650U, 0U}, {0x2541U, 0U}}};
  auto now = core::SdoReadSequence::Clock::now();
  auto request = sequence.start(now);
  ASSERT_TRUE(request);
  for (const auto &object : order) {
    const auto index = static_cast<uint16_t>(request->data[1]) |
        (static_cast<uint16_t>(request->data[2]) << 8U);
    EXPECT_EQ(index, object.index);
    EXPECT_EQ(request->data[3], object.subindex);
    const auto response = protocol::answer_cerb_sdo_upload_request(
        *request, objects);
    ASSERT_TRUE(response);
    now += 1ms;
    request = sequence.accept_response(*response, now);
  }
  EXPECT_FALSE(request);
  ASSERT_TRUE(sequence.complete());
  EXPECT_EQ(sequence.result().state, core::SimDeviceInitState::Ok);
  EXPECT_NEAR(sequence.result().dev_values[0], -3.14159265F, 1e-5F);
  EXPECT_NEAR(sequence.result().dev_values[1], 3.14159265F, 1e-5F);
  EXPECT_NEAR(sequence.result().dev_values[2], 2.0F * 3.14159265F / 60.0F,
              1e-5F);
  EXPECT_NEAR(sequence.result().dev_values[3], 2.5F, 1e-5F);
  EXPECT_EQ(sequence.result().dev_values[4], 0.0F);
  EXPECT_EQ(sequence.result().dev_values[5], 1.0F);
}
```

- [ ] **Step 2: 注册新测试并确认其因序列接口缺失而失败**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim --cmake-target test_sdo_read_sequence
```

预期：编译失败，错误指向缺失的 `sdo_read_sequence.hpp` 或 `SdoReadSequence`。

- [ ] **Step 3: 实现纯 C++ 六对象事务状态机**

对象顺序固定为 `607D:01`、`607D:02`、`6080:00`、`2538:00`、`2650:00`、`2541:00`。匹配 0x43 时按原始有符号 int32 换算成 rad、rad/s（使用 motor gear ratio）、A、安全原值、方向原值；按预期 raw object 对比，位置/速度/电流逐槽比较原始设备精度值。超时重试一次（包括首次发送共最多两次），ABORT 不重试；未读槽位保留 NaN。

- [ ] **Step 4: 运行纯事务状态机测试**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim
colcon test --packages-select motor_sim --ctest-args -R test_sdo_read_sequence
colcon test-result --verbose
```

预期：固定时钟下六对象顺序、过滤、ABORT、超时和位图测试通过，不需要 ROS domain、vcan 或真实硬件。

- [ ] **Step 5: 先写 ROS 网关节点测试并确认节点接口缺失**

在 `test_sim_device_param_gateway.cpp` 构造注入 `FrameTransport`，发布 `/motor_sim/system_command` 的 `COMMAND_REFRESH`，再由 fake transport 依次提供六个正确对象响应；断言第一帧为 `0x600 + motor_id`，节点串行读取，报告换算正确，`refresh_seq` 增长并发布 reason=2 的 `EVENT_DEV_INIT`。另测试 timeout/ABORT 的 41 状态和 VAXOR REFRESH 不发送帧但完成。注册测试 target、加入 `can_bridge_msgs` CMake/package 依赖，再运行：

fake `ResponderTransport::send()` 应自动生成并排队 SDO 应答：

```cpp
bool send(const protocol::CanFrame &frame) override {
  sent.push_back(frame);
  const auto response =
      protocol::answer_cerb_sdo_upload_request(frame, objects_);
  if (response) {
    incoming.push_back({*response, std::chrono::steady_clock::now()});
  }
  return opened_;
}
```

observer 测试通过 ROS topic 验证端到端消息：

```cpp
can_bridge_msgs::msg::SystemCommand command;
command.sync_time_ns = observer->now().nanoseconds();
command.request_id = 1U;
command.command = command.COMMAND_REFRESH;
command.motor_id = 1U;
const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
while (command_pub->get_subscription_count() == 0U &&
       std::chrono::steady_clock::now() < discovery_deadline) {
  executor.spin_some();
  std::this_thread::sleep_for(1ms);
}
ASSERT_GT(command_pub->get_subscription_count(), 0U);
command_pub->publish(command);
const auto deadline = std::chrono::steady_clock::now() + 2s;
while ((!status || status->refresh_seq == 0U || !report || !event) &&
       std::chrono::steady_clock::now() < deadline) {
  executor.spin_some();
  std::this_thread::sleep_for(1ms);
}
ASSERT_TRUE(status);
ASSERT_EQ(status->refresh_seq, 1U);
ASSERT_TRUE(report);
ASSERT_EQ(report->count, 1U);
EXPECT_NEAR(report->entries[0].dev_pos_min, -3.14159265F, 1e-5F);
EXPECT_NEAR(report->entries[0].dev_pos_max, 3.14159265F, 1e-5F);
EXPECT_NEAR(report->entries[0].dev_vel_limit,
            2.0F * 3.14159265F / 60.0F, 1e-5F);
EXPECT_NEAR(report->entries[0].dev_torque_limit, 2.5F, 1e-5F);
ASSERT_TRUE(event);
EXPECT_EQ(event->event_type, event->EVENT_DEV_INIT);
EXPECT_EQ(event->reason, 2U);
EXPECT_EQ(event->request_id, 0U);
EXPECT_EQ(fake_view->sent.size(), 6U);
```

在测试源中定义 `make_config()` 返回一条 ID 1、bus `arm`、gear ratio 101 的 CERB motor，`expected_objects` 与 fake 的 `sdo_objects` 使用 raw `-32768, 32768, 101, 2500, 0, 1`；按上述回调创建三个订阅、一个命令 publisher，将 gateway 和 observer 加入同一 `SingleThreadedExecutor`。另外各加一个 missing-object case（删除 2650:00，断言 status 41、UNVERIFIED、event reason 4、mask bit4）和 VAXOR case（REFRESH 后序号增长且 `sent.empty()`）。

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim --cmake-target test_sim_device_param_gateway
```

预期：编译失败，错误指向尚不存在的 `SimDeviceParamGatewayNode`。

- [ ] **Step 6: 实现 ROS 网关和安装入口**

网关为每个配置 motor 初始化一条报告 entry：CERB `dev_*` 为 NaN、状态 PENDING，`rec_*` 为仿真 profile 基准；VAXOR 的 `dev_init_state=NA` 且全部 `rec_*`/`dev_*` 为 NaN。周期发布配置/读回快照和 `SystemStatus`；`SystemCommand.COMMAND_REFRESH` 按 `motor_id` 入 FIFO，一次只运行一条 `SdoReadSequence`，消费 Transport 帧并驱动状态机。每条有效 REFRESH（含 VAXOR 的无对象立即完成）完成后增加 `refresh_seq`；状态一致 `last_error=0` 并发布 reason=2、mask=0 的 `EVENT_DEV_INIT`；全部成功但不一致 `last_error=40` 并发布 reason=3、mismatch 位图的事件；任一 ABORT/超时 `last_error=41`、状态 UNVERIFIED、`mismatch_mask=0`，并发布 reason=4、unread 位图的事件（同一轮有缺失时以 UNVERIFIED 为最终结论）。非法 ID 的 `last_error=10`、`last_error_motor=LAST_ERROR_MOTOR_NONE`；拒绝 SAVE、SET_ZERO、CLEAR_FAULT，不向其他命令话题转发。VAXOR 的有效 REFRESH 不发送 CAN 帧，保持 NA 并立即完成。

配置报告以 1 Hz 发心跳、每轮 REFRESH 完成后立即更新；`SystemStatus` 以 5 Hz 发心跳并在完成时立即更新。报告 `rec_torque_limit`/`dev_torque_limit` 使用消息约定的 A 单位，`0x2538` 原值按 mA→A 换算。

- [ ] **Step 7: 构建新目标并运行相关 C++ 测试**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select motor_sim
colcon test --packages-select motor_sim --ctest-args -R 'test_(sdo_read_sequence|sim_device_param_gateway)'
colcon test-result --verbose
```

预期：新事务和网关测试通过，`motor_sim` 其余测试仍通过。

- [ ] **Step 8: 提交网关**

```bash
git add ros2_ws/src/motor_sim/CMakeLists.txt ros2_ws/src/motor_sim/package.xml ros2_ws/src/motor_sim/include/motor_sim/core/sdo_read_sequence.hpp ros2_ws/src/motor_sim/src/core/sdo_read_sequence.cpp ros2_ws/src/motor_sim/test/test_sdo_read_sequence.cpp ros2_ws/src/motor_sim/include/motor_sim/ros/sim_device_param_gateway.hpp ros2_ws/src/motor_sim/src/ros/sim_device_param_gateway.cpp ros2_ws/src/motor_sim/src/ros/sim_device_param_gateway_main.cpp ros2_ws/src/motor_sim/test/test_sim_device_param_gateway.cpp
git commit -m "feat: add simulation device parameter gateway"
```

### Task 5: 仅在 host-vcan 启动网关并验证运行配置

**Files:**
- Modify: `ros2_ws/src/robot_arm_gazebo/launch/simulation.launch.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/test/test_launch_models.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/README.md`

**Interfaces:**
- Consumes: 已生成的 `motor_sim.yaml`（含 profile 对象基准和 overrides）、已安装的 `motor_sim/sim_device_param_gateway`。
- Produces: `_sim_device_param_gateway_enabled(controller_mode: str, transport: str) -> bool`，仅 host/vcan 返回 true；host 启动参数为同一 `motor_sim.yaml` 和 `vcan_interface`。仿真节点和 gateway 使用同一 vcan 网络但独立 SocketCAN socket。

- [ ] **Step 1: 先写启动谓词和运行期 YAML 保留字段测试**

在 `test_launch_models.py` 添加参数化测试，四种组合 `host/vcan`、`host/socketcan`、`external/vcan`、`firmware/vcan` 的预期结果依次为 true、false、false、false。另调用 `_write_runtime_configs()` 写入临时目录，读回 `motor_sim.yaml` 并断言 `profiles.cerb_r58.device_params.safety_raw`、`direction_raw` 和 `sdo_overrides` 均保留。

在文件顶部增加 `import yaml`。测试函数按以下方式从已安装的模型和 motor driver 配置生成运行期 YAML，不创建 vcan 或 launch process：

```python
def test_sim_device_param_gateway_is_host_vcan_only() -> None:
    launch_file = LAUNCH_DIR / "simulation.launch.py"
    spec = importlib.util.spec_from_file_location("simulation_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module._sim_device_param_gateway_enabled("host", "vcan") is True
    assert module._sim_device_param_gateway_enabled("host", "socketcan") is False
    assert module._sim_device_param_gateway_enabled("external", "vcan") is False
    assert module._sim_device_param_gateway_enabled("firmware", "vcan") is False


def test_runtime_motor_sim_yaml_keeps_device_params(tmp_path: Path) -> None:
    from ament_index_python.packages import get_package_share_directory

    launch_file = LAUNCH_DIR / "simulation.launch.py"
    spec = importlib.util.spec_from_file_location("simulation_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    share = _package_share()
    simulation = module._load_simulation(
        share / "models" / MODEL / "config" / "simulation_topology.yaml"
    )
    driver = Path(get_package_share_directory("motor_hardware_interface"))
    driver = driver / "config" / "motor_driver.yaml"
    specs_path = module._resolve_motor_specs_path(driver)
    simulation = module._bind_motor_topology(simulation, driver, specs_path)
    sim_config, _ = module._write_runtime_configs(
        simulation, tmp_path, "vcan", "vcan0", specs_path
    )
    runtime = yaml.safe_load(sim_config.read_text(encoding="utf-8"))
    params = runtime["motor_simulation"]["profiles"]["cerb_r58"]["device_params"]
    assert params == {"safety_raw": 0, "direction_raw": 0, "sdo_overrides": {}}
```

- [ ] **Step 2: 运行 launch 测试确认失败**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select robot_arm_gazebo
colcon test --packages-select robot_arm_gazebo
```

预期：失败，指出谓词不存在或运行配置缺少新字段。

- [ ] **Step 3: 接入条件启动和同接口参数**

在 launch helper 中实现上述纯谓词；在 `controller_mode == "host"` 分支，仅当谓词为 true 时加入 ROS Node：`package="motor_sim"`、`executable="sim_device_param_gateway"`、参数 `config_file=str(motor_sim_config)` 与 `interface=vcan_interface`。不要在 socketcan、slcan、external 或 firmware 分支创建该 Node。仿真 motor_sim 与硬件驱动原 launch 顺序、传输参数和 vcan 清理逻辑保持不变。

- [ ] **Step 4: 更新仿真运行说明并重跑 launch 测试**

在 `robot_arm_gazebo/README.md` 说明只有 host-vcan 有 `/motor_sim/*` SDO 回读、需预先创建并启动 vcan、仿真节点和网关共用接口；列出其他模式不支持的边界。运行：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select robot_arm_gazebo
colcon test --packages-select robot_arm_gazebo
colcon test-result --verbose
```

预期：启动模式谓词、运行配置保留和既有 launch/model 静态测试通过；不启动 Gazebo。

- [ ] **Step 5: 提交 launch 集成**

```bash
git add -p ros2_ws/src/robot_arm_gazebo/launch/simulation.launch.py
git add ros2_ws/src/robot_arm_gazebo/test/test_launch_models.py
git add -p ros2_ws/src/robot_arm_gazebo/README.md
git diff --cached --check
git diff --cached -- ros2_ws/src/robot_arm_gazebo/launch/simulation.launch.py ros2_ws/src/robot_arm_gazebo/test/test_launch_models.py ros2_ws/src/robot_arm_gazebo/README.md
git commit -m "feat: launch simulation SDO gateway on host vcan"
```

在 `git add -p` 中只选择新增网关谓词、启动动作和文档段落；用 staged diff 确认 vcan 权限修复和其他用户差异不在提交中。

### Task 6: 在设备参数卡隔离真机与仿真只读视图

**Files:**
- Modify: `ros2_ws/src/web_host/web/index.html`
- Modify: `ros2_ws/src/web_host/web/app.js`
- Modify: `ros2_ws/src/web_host/README.md`
- Create: `ros2_ws/src/web_host/test/test_sim_param_source.py`

**Interfaces:**
- Consumes: 原真机参数 topic；Task 4 的 `/motor_sim/system_command`、`/motor_sim/device_param_report`、`/motor_sim/system_status`、`/motor_sim/motor_events`。
- Produces: 设备参数卡的“真机 / 仿真”来源选择，默认真机；独立 `paramCache`/`simParamCache`、`state.sysStatus`/`state.simSysStatus` 和独立事件位图；仿真只发布 `/motor_sim/system_command` 的 `REFRESH`，真机只走原 `/micro_ros_do/system_command`。来源切换只重绘参数卡，不切换主监控、运动流或 3D 数据。

- [ ] **Step 1: 写 source selector 与 topic 隔离的 pytest**

在 `test_sim_param_source.py` 加入：

```python
from pathlib import Path

WEB = Path(__file__).parents[1] / "web"


def test_device_parameter_selector_defaults_to_real() -> None:
    html = (WEB / "index.html").read_text(encoding="utf-8")
    assert '<select id="paramSource">' in html
    assert '<option value="real" selected' in html


def test_sim_parameter_topics_and_caches_are_separate() -> None:
    app = (WEB / "app.js").read_text(encoding="utf-8")
    for topic in (
        "/motor_sim/system_command",
        "/motor_sim/device_param_report",
        "/motor_sim/system_status",
        "/motor_sim/motor_events",
    ):
        assert topic in app
    assert "simParamCache" in app
    assert "simSysStatus" in app
```


- [ ] **Step 2: 运行 pytest 确认它因缺少来源选择而失败**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select web_host
colcon test --packages-select web_host
```

预期：新加的 selector/topic 检查失败；此时尚未修改 HTML/JavaScript。

- [ ] **Step 3: 添加来源选择器、仿真 topics 和独立状态处理**

在 `设备参数` summary 区加入 `<select id="paramSource">`，选项值 `real`/`sim`，默认 `real`；给 `btnParamApply`、`btnParamSave` 增加写操作容器 id，保留公共读取和“显示全部”。

新增 `SIM_TOPICS` 和 `simSysCmd` publisher；订阅仿真报告时填充 `simParamCache`，仿真状态写入 `state.simSysStatus`，DEV_INIT 事件写入仿真事件缓存。订阅真机的逻辑继续只更新原状态。实现 `selectedParamSource()`、`renderSelectedDeviceRows()` 和来源专属回读函数；页面重连时分别清空两种缓存并重置两种状态的 `seen`，再按当前选择等待对应 system_status 后读取；来源变化时使旧的异步等待失效，避免迟到的真机完成序号覆盖仿真结果。

仿真逐台刷新仅对 CERB entry 发布 `SystemCommand{command: REFRESH, motor_id}`，每台等待仿真 `refresh_seq` 增长后再读下一台；不向仿真发布 SAVE/SET_ZERO/CLEAR_FAULT。报告的设备配置基准和实际 SDO 值文案明确显示“仿真配置 / SDO 回读”；未读值保留 NaN 并显示 PENDING、MISMATCH、UNVERIFIED 或“无设备对象”。

- [ ] **Step 4: 将仿真表格渲染为只读并保留真机行行为**

仿真参数格使用只读文本与标题提示基准/回读值，隐藏/禁用参数下发、W25、设限、保护开关、方向切换、设零、清错；逐行回读和全量顺序读取仍可用。仿真表只读选中源的 `DeviceParamReport`，不读取真机 `state.motors` 的型号/生命周期；CERB 行标签为“CERB SDO”，NA 行显示“无设备对象”。`renderDevRows()` 的真机分支沿用原可编辑行、真机型号/生命周期字段、真机请求和错误文案。source select 的 change handler 只触发设备参数面渲染及该来源回读。

所有真机写入口（整表下发、保存、设限、方向、设零、清错）在仿真来源被选中时还要在 handler/函数入口直接拒绝，不能只靠隐藏按钮；仿真事件保存在独立 `simDevInit`/事件缓存，不并入真机 `state.events`。使能与增益策略表仍使用真机 `paramCache`，不随设备参数来源切换。

- [ ] **Step 5: 更新 WebHost 文档并完成浏览器隔离核对**

`web_host/README.md` 说明设备参数来源选择、host-vcan 仿真读路径、只读能力与 topic 清单；注明主监控、策略表和 3D 仍属原真机来源。运行 `colcon build --packages-select web_host` 后 `colcon test --packages-select web_host`，确认新增 pytest 通过。再用隔离的 ROS_DOMAIN_ID、rosbridge 和静态 WebHost 页面，不连接真机 Agent，交错发布不同 motor ID 的仿真/真机 `DeviceParamReport` 与各自 `SystemStatus`，逐项核验：

1. 默认真机仍订阅和发布现有 `/micro_ros_*` 话题；
2. 切到仿真时只有 `/motor_sim/system_command` 收到 REFRESH，页面以 `/motor_sim/*` 状态/报告推进回读；
3. 所选页面只显示其来源缓存，切回后另一缓存仍独立保存；更改来源时旧异步回读完成不能改写当前来源结果；
4. 仿真行没有任何写操作，读取与逐台读取可用，主监控/策略/3D 不随来源选择变化。

此工作区没有 JS 自动测试器或 Node 构建依赖；不为本功能引入新的 JS 工具链。

- [ ] **Step 6: 提交网页来源隔离**

```bash
git add -p ros2_ws/src/web_host/web/index.html
git add -p ros2_ws/src/web_host/web/app.js
git add -p ros2_ws/src/web_host/README.md
git add ros2_ws/src/web_host/test/test_sim_param_source.py
git diff --cached --check
git diff --cached -- ros2_ws/src/web_host/web/index.html ros2_ws/src/web_host/web/app.js ros2_ws/src/web_host/README.md ros2_ws/src/web_host/test/test_sim_param_source.py
git commit -m "feat: isolate simulated device parameter source"
```

这三个文件已有未提交用户改动；在 `git add -p` 中只选来源选择和仿真只读逻辑对应的 hunks，逐段核对 staged diff 没有混入原有 rosbridge/Agent/vcan 差异。

### Task 7: 完成包构建、测试和变更范围复核

**Files:**
- Review only: 本计划列出的 `motor_sim`、`robot_arm_gazebo`、`web_host` 文件。

**Interfaces:**
- Consumes: Tasks 1–6 的实现与各任务提交。
- Produces: 相关 ROS 包构建和自动测试通过的记录、浏览器隔离检查结果、无无关用户改动入提交的工作区状态。

- [ ] **Step 1: 检查格式并构建相关包**

```bash
git diff --check
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to motor_sim robot_arm_gazebo web_host
```

预期：三个目标包及其依赖构建成功，`motor_sim` 安装出 `sim_device_param_gateway`。

- [ ] **Step 2: 运行相关自动测试**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon test --packages-select motor_sim robot_arm_gazebo web_host
colcon test-result --verbose
```

预期：SDO codec、虚拟设备应答、串行事务、launch 条件与现有相关包测试全部通过；无测试启动 Gazebo 或真实硬件。

- [ ] **Step 3: 对照规格检查 topic、事务顺序和来源隔离**

逐项检查规格验证标准：host-vcan 使用同一接口；仿真只收 `/motor_sim/*`；请求顺序为 6 个 CERB 对象；序号每次请求完成后增长；ABORT/超时生成 unread 位图；错误码 40/41 和事件 reason/mask 一致；VAXOR 不发 CERB SDO；切换来源不混 cache/status/event；仿真控件只读。

- [ ] **Step 4: 复核 Git 暂存和提交范围**

```bash
git status --short
git log -7 --oneline
```

逐个确认本功能提交只包含本计划列出的文件；工作区中 `robot_arm_gazebo/README.md`、`robot_arm_gazebo/launch/simulation.launch.py`、`robot_arm_gazebo/scripts/simulation_gate.py`、`robot_dynamics/README.md`、`web_host/README.md`、`web_host/package.xml`、`web_host/web/app.js`、`web_host/web/index.html` 等已有用户改动未被覆盖、暂存或混入功能提交。若这些文件同时承载新功能，提交只 stage 明确审过的 hunks，不 stage 文件的其他已有差异。
