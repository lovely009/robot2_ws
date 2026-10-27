import { test, expect, type Page } from "@playwright/test";
import { readFileSync } from "node:fs";

async function connectFixture(page: Page, coupled = false, withControlStates = false) {
  const publishes: { topic: string; msg: Record<string, unknown> }[] = [];
  const errors: string[] = [];
  const connections = { count: 0 };
  const feedbackModels: string[] = [];
  const controlRequests: boolean[] = [];
  let controlSequence = 0;
  let sendFeedback: (topic: string, msg: unknown) => void;
  page.on("pageerror", (error) => errors.push(error.message));
  await page.route("**/api/**", (route) => {
    const url = new URL(route.request().url());
    if (url.pathname === "/api/models")
      return route.fulfill({
        json: { models: ["half_body_robot", "robot_leg"] },
      });
    if (url.pathname === "/api/model-transmissions")
      return route.fulfill(
        url.searchParams.get("model") === "robot_leg"
          ? { status: 404, json: { error: "没有设备映射" } }
          : { json: { model: "half_body_robot", driver_file: "/fixture/driver.yaml",
              groups: coupled ? [{ name: "wrist", plugin: "transmission_interface/DifferentialTransmission",
                motors: [{ name: "m8", id: 8 }, { name: "m9", id: 9 }],
                joints: ["joint_link8", "joint_link9"], direct_mapping: false }]
                : [{ name: "direct_1", plugin: "transmission_interface/SimpleTransmission",
                motors: [{ name: "m1", id: 1 }], joints: ["joint_link1"], direct_mapping: true }],
              direct_joints: { 1: "joint_link1" } } },
      );
    if (url.pathname === "/api/model-joints")
      return route.fulfill(
        url.searchParams.get("model") === "robot_leg"
          ? { status: 404, json: { error: "没有设备映射" } }
          : { json: { joints: { 1: "joint_link1" } } },
      );
    if (url.pathname === "/api/motor-specs")
      return route.fulfill({ json: { motor_types: {} } });
    return route.fulfill({ json: { actions: [] } });
  });
  const fixtures: Record<string, unknown> = {
    "/motor/topology": {
      entries: [
        {
          motor_id: 1,
          online: true,
          model_name: "cerb_r48_101",
          bus_index: 0,
          lifecycle: 3,
        },
      ],
    },
    "/motor/motion_states": {
      states: [
        {
          motor_id: 1,
          position: 0.15,
          velocity: 0.2,
          effort: 0.1,
          target_source: 1,
          state_age_ms: 5,
        },
      ],
    },
    "/motor/status_array": {
      statuses: [
        {
          motor_id: 1,
          online: true,
          lifecycle: 3,
          current: 0.2,
          temperature: 30,
          unified_fault_code: 0,
        },
      ],
    },
    "/joint_states": {name: ["joint_link1"], position: [0.15], velocity: [0.2], effort: [0.1]},
    "/motor_command/status": {
      data: JSON.stringify({
        output_topic: "/motor/command_array",
        clients: 1,
        targets: 0,
      }),
    },
    "/bench/status": {
      data: JSON.stringify({
        agent: { state: "stopped" },
        simulation: { state: "stopped" },
        motor_backend: { state: "stopped" },
      }),
    },
    "/motor/system_status": {
      refresh_seq: 0,
      set_zero_seq: 0,
      clear_fault_seq: 0,
      save_seq: 0,
      save_in_progress: false,
      last_error: 0,
    },
  };
  if (coupled) {
    fixtures["/motor/topology"] = {entries:[1,8,9].map(motor_id =>
      ({motor_id,online:true,model_name:'cerb_r48_101',bus_index:0,lifecycle:motor_id===1?3:4}))};
    fixtures["/joint_states"] = { name: ["joint_link8", "joint_link9"],
      position: [0.2, 0.1], velocity: [0, 0], effort: [0, 0] };
    fixtures["/motor/status_array"] = {statuses: [8,9].map(motor_id =>
      ({motor_id, online: true, lifecycle: 4, unified_fault_code: 0}))};
    fixtures["/robot_transmission/status"] = {data: JSON.stringify({model: "half_body_robot",
      trajectory_enabled: true, joint_names: ["joint_link8", "joint_link9"]})};
    fixtures["/motor_command/status"] = {data: JSON.stringify({output_topic: "/motor/command_array",
      trajectory_output_topic: "/motor/trajectory"})};
  }
  const makeControlStates = (enabled: boolean, fault = 0) => ({schema_version: 1, session_id: 7001,
    sequence: ++controlSequence, tick_ms: controlSequence * 20,
    states: [8,9].map(motor_id => ({motor_id, online: true,
      control_lifecycle: enabled && !fault ? 3 : 2, execution_state: 1,
      policy_enabled: enabled, motion_enabled: false, state_age_ms: 5,
      unified_fault_code: fault, vendor_fault_code: 0, last_failure_reason: 0,
      policy_generation: enabled ? 2 : 1}))});
  if (coupled && withControlStates)
    fixtures['/motor/control_states'] = makeControlStates(false);
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => {
    sendFeedback = (topic, msg) => socket.send(JSON.stringify({op: 'publish', topic, msg}));
    const subscribed = new Set<string>();
    if (coupled) {
      const heartbeat = setInterval(() => {
        for (const topic of subscribed) {
          if (topic === '/motor/control_states' && fixtures[topic])
            (fixtures[topic] as {sequence:number}).sequence = ++controlSequence;
          if (fixtures[topic]) socket.send(JSON.stringify({op: "publish", topic, msg: fixtures[topic]}));
        }
      }, 100);
      page.once("close", () => clearInterval(heartbeat));
    }
    connections.count++;
    socket.onMessage((raw) => {
      const message = JSON.parse(String(raw));
      if (message.op === "subscribe") subscribed.add(message.topic);
      if (message.op === "subscribe" && fixtures[message.topic])
        socket.send(
          JSON.stringify({
            op: "publish",
            topic: message.topic,
            msg: fixtures[message.topic],
          }),
        );
      if (message.op === "publish") {
        publishes.push(message);
        if (
          message.topic === "/motor/system_command" &&
          message.msg.command === 4
        )
          socket.send(
            JSON.stringify({
              op: "publish",
              topic: "/motor/device_param_report",
              msg: { count: 0, entries: [] },
            }),
          );
      }
      if (message.op === "call_service") {
        if (message.service === '/firmware_topic_bridge/set_parameters_atomically') {
          const enabled = message.args.parameters[0].value.bool_value;
          controlRequests.push(enabled);
          const current = JSON.parse((fixtures['/robot_transmission/status'] as {data: string}).data);
          fixtures['/robot_transmission/status'] = {data: JSON.stringify({...current, trajectory_enabled: enabled})};
          socket.send(JSON.stringify({op: 'service_response', id: message.id,
            service: message.service, result: true,
            values: {result: {successful: true, reason: ''}}}));
          return;
        }
        if (message.service === '/bench_manager/set_parameters_atomically') {
          feedbackModels.push(message.args.parameters[0].value.string_value);
          socket.send(JSON.stringify({op: 'service_response', id: message.id,
            service: message.service, result: true,
            values: {result: {successful: true, reason: ''}}}));
          return;
        }
        const response =
          message.service === "/bench/options"
            ? {
                simulation_models: ["half_body_robot"],
                mujoco_models: ["half_body_robot"],
                serial_ports_json: "[]",
                can_interfaces_json: "[]",
              }
            : {
                ok: true,
                status_json: '{"state":"stopped"}',
                response_json: '{"ok":true,"motors":[]}',
              };
        socket.send(
          JSON.stringify({
            op: "service_response",
            id: message.id,
            service: message.service,
            result: true,
            values: response,
          }),
        );
      }
    });
  });
  return { publishes, errors, connections, feedbackModels, controlRequests,
    sendFeedback: (topic: string, msg: unknown) => {fixtures[topic]=msg;sendFeedback(topic, msg);},
    sendControlState: (enabled: boolean, fault = 0) => {
      const message = makeControlStates(enabled, fault);
      fixtures['/motor/control_states'] = message;
      sendFeedback('/motor/control_states', message);
    } };
}

test("Vue 命令记录展示发送值，导出完整历史，清空不发送控制命令", async ({page}) => {
  const fixture = await connectFixture(page);
  const legacyRequests: string[] = [];
  page.on('request', request => {
    if (request.url().endsWith('/js/panels/history.js')) legacyRequests.push(request.url());
  });
  await page.goto('/workbench/#/run');
  await expect(page.locator('.startup-status')).toHaveCount(0);
  await page.getByRole('button', {name: '命令记录', exact: true}).click();
  const panel = page.locator('[data-workbench-panel="history"]');
  await expect(panel).toContainText('不代表设备已接收或执行');
  await expect(panel).toContainText('暂无记录');
  await expect(panel.getByRole('button', {name: '导出 JSON'})).toBeDisabled();
  await page.evaluate(async () => {
    const {store, markDirty, DIRTY} = await import('/js/core/store.js');
    store.history = Array.from({length: 21}, (_, i) => ({
      t: 1700000000000 - i * 1000, seq: 21 - i,
      targets: [{motor_id: 8, position: 0.123, velocity: -0.456, effort: 1.789, kp: 2, kd: 0.1}],
    }));
    markDirty(DIRTY.HISTORY);
  });
  await expect(panel).toContainText('21 条');
  await expect(panel.locator('tbody tr')).toHaveCount(20);
  await expect(panel.locator('tbody tr').first()).toContainText('0.123');
  await expect(panel.locator('tbody tr').first()).toContainText('-0.456');
  await expect(panel.locator('tbody tr').first()).toContainText('1.789');
  const downloadPromise = page.waitForEvent('download');
  await panel.getByRole('button', {name: '导出 JSON'}).click();
  const download = await downloadPromise;
  const {readFile} = await import('node:fs/promises');
  const records = JSON.parse(await readFile((await download.path())!, 'utf8'));
  expect(records).toHaveLength(21);
  expect(records[0].seq).toBe(1);
  expect(records[20].seq).toBe(21);
  expect(records[0].targets[0]).toEqual({motor_id:8, position:0.123, velocity:-0.456, effort:1.789, kp:2, kd:0.1});
  const beforeClear = fixture.publishes.length;
  await panel.getByRole('button', {name: '清空', exact: true}).click();
  await expect(panel).toContainText('暂无记录');
  await expect(panel.getByRole('button', {name: '导出 JSON'})).toBeDisabled();
  expect(fixture.publishes).toHaveLength(beforeClear);
  expect(legacyRequests).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test("关节收起后向下让出空间，观察区使用箭头折叠", async ({page}) => {
  await page.setViewportSize({width: 1063, height: 752});
  await connectFixture(page);
  await page.goto('/workbench/#/run');
  const joints = page.locator('#run-joints');
  const summary = joints.locator('summary');
  await expect(summary).toBeVisible();
  await expect(page.locator('.startup-status')).toHaveCount(0);
  // 小视口先收起动作库，给关节与臂视图的高度变化留出空间。
  await page.locator('[data-workbench-panel="actionLibrary"] summary').click();
  const robot = page.locator('[data-workbench-panel="arm"]');
  const before = (await robot.boundingBox())!.height;
  await summary.click();
  await expect.poll(async () => (await joints.boundingBox())!.height).toBeLessThan(45);
  await expect.poll(async () => (await robot.boundingBox())!.height).toBeGreaterThan(before);
  await summary.click();
  await expect.poll(async () => (await joints.boundingBox())!.height).toBeGreaterThan(150);
  const toggle = page.locator('.observer-head').getByRole('button', {name: '收起观察区', exact: true});
  await expect(toggle).toHaveText('▾');
  const arrowLeft = await toggle.evaluate(element => {
    const range = document.createRange(); range.selectNodeContents(element);
    return range.getBoundingClientRect().left;
  });
  const upperArrowLeft = await summary.evaluate(element =>
    element.getBoundingClientRect().left + parseFloat(getComputedStyle(element).paddingLeft));
  expect(Math.abs(arrowLeft - upperArrowLeft)).toBeLessThanOrEqual(1);
  await toggle.click();
  await expect(page.locator('#observation-content')).toBeHidden();
  await page.locator('.observer-head').getByRole('button', {name: '展开观察区', exact: true}).click();
  await expect(page.locator('#observation-content')).toBeVisible();
});

test("曲线在持续运动反馈超过缓冲上限后仍自动绘制新数据", async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  await expect(page.locator('#chartHint')).toContainText('1 个组合');
  await expect(page.locator('#chartHint')).toContainText('范围');
  for (let i = 1; i <= 900; i++) {
    fixture.sendFeedback('/motor/motion_states', {states: [{
      motor_id: 1, position: i / 10, velocity: 0.2, effort: 0.1,
      target_source: 1, state_age_ms: 5,
    }]});
  }
  await expect(page.locator('#selRows')).toContainText('90.000');
  await expect(page.locator('#chartHint')).toContainText('1 个组合');
  fixture.sendFeedback('/motor/motion_states', {states: [{
    motor_id: 1, position: 180, velocity: 0.2, effort: 0.1,
    target_source: 1, state_age_ms: 5,
  }]});
  await expect(page.locator('#chartHint')).toContainText('194.39');
  expect(fixture.errors).toEqual([]);
});

test("窄栏电机卡片清楚显示单位、状态并保持选择联动", async ({page}) => {
  await page.setViewportSize({width: 1063, height: 752});
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  const card = page.locator('.motor-card[data-id="1"]');
  await expect(card).toContainText('0.150');
  await expect(card).toContainText('rad');
  await expect(card).toContainText('N·m');
  await expect(card).toContainText('30.0');
  await expect(card).toContainText('°C');
  await expect(card).toContainText('电流');
  await expect(card).toContainText('0.20');
  await expect(card).toContainText('就绪');
  await expect(card).toContainText('速度');
  await expect(card.locator('.motor-card-metrics > span')).toHaveCount(5);
  const metrics = await card.locator('.motor-card-metrics b').evaluateAll(nodes =>
    nodes.map(n => getComputedStyle(n).fontSize));
  expect(new Set(metrics).size).toBe(1);
  await expect.poll(async () => (await card.boundingBox())?.height ?? Infinity).toBeLessThan(205);
  await expect(page.locator('#selTitle')).toHaveText('电机状态');
  expect(await card.evaluate(e => e.scrollWidth <= e.clientWidth)).toBe(true);
  const cardNode = await card.elementHandle();
  await page.waitForTimeout(650);
  expect(await card.evaluate((current, original) => current === original, cardNode)).toBe(true);
  await card.click();
  await expect(page.locator('.object-heading h2')).toHaveText('joint_link1');
  await expect(page.locator('#jointRows [data-joint="joint_link1"] .jpos')).toHaveText('0.150');
  expect(fixture.publishes.filter(p => p.topic !== '/motor/system_command')).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test("连接与切换模型自动同步只读反馈模型，不产生运动命令", async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  await expect.poll(() => fixture.feedbackModels).toEqual(['half_body_robot']);
  await page.locator('.arm-model').selectOption('robot_leg');
  await expect.poll(() => fixture.feedbackModels).toEqual(['half_body_robot', 'robot_leg']);
  expect(fixture.publishes.filter(p => p.topic !== '/motor/system_command')).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test("其他页面选择模型后，当前页面不会定时抢回旧模型", async ({page}) => {
  const fixture = await connectFixture(page);
  await page.clock.install();
  await page.goto('/workbench/#/run');
  await expect.poll(() => fixture.feedbackModels).toEqual(['half_body_robot']);
  fixture.sendFeedback('/bench/status', {data: JSON.stringify({feedback: {
    state: 'running', source: 'firmware', model: 'other_arm',
    desired_model: 'other_arm', reason: '固件关节反馈', error: '',
  }})});
  await expect(page.locator('#jointFeedbackSource')).toContainText('other_arm');
  await page.clock.fastForward(5500);
  await page.clock.resume();
  fixture.sendFeedback('/motor/motion_states', {states: [{
    motor_id: 1, position: 0.333, velocity: 0, effort: 0, state_age_ms: 0,
  }]});
  await expect(page.locator('.motor-card')).toContainText('0.333');
  expect(fixture.feedbackModels).toEqual(['half_body_robot']);
});

test('关节标题固定、选择不重排，滑块拖动发送且软限按关节保存', async ({page}) => {
  await page.setViewportSize({width:1063,height:724});
  const fixture = await connectFixture(page, true);
  await page.goto('/workbench/#/run');
  const rows = page.locator('#jointRows > li[data-joint]');
  await expect(rows).toHaveCount(18);
  const order = await rows.evaluateAll(nodes => nodes.map(n => n.getAttribute('data-joint')));
  await page.locator('#jointRows [data-joint="joint_link9"] .jid').click();
  expect(await rows.evaluateAll(nodes => nodes.map(n => n.getAttribute('data-joint')))).toEqual(order);
  const pane = page.locator('#run-joints');
  await pane.evaluate(e => {e.scrollTop = 200;});
  const panelTop = (await pane.boundingBox())!.y;
  expect(Math.abs((await pane.locator('summary').boundingBox())!.y-panelTop)).toBeLessThan(3);
  const row = page.locator('#jointRows [data-joint="joint_link8"]');
  const slider = row.locator('input[type="range"]');
  await expect(slider).toBeEnabled();
  await expect(slider).toHaveAttribute('min','-1.047');
  await expect(slider).toHaveAttribute('max','1.047');
  await slider.evaluate(e => {e.value='0.35';e.dispatchEvent(new Event('input',{bubbles:true}));});
  fixture.sendFeedback('/joint_states',{name:['joint_link8','joint_link9'],position:[0.21,0.1],velocity:[0,0],effort:[0,0]});
  await expect(row.locator('.jpos')).toHaveText('0.210');
  await expect(slider).toHaveValue('0.35');
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/arm_controller/joint_trajectory').length).toBe(1);
  await slider.evaluate(e => e.dispatchEvent(new Event('change',{bubbles:true})));
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/arm_controller/joint_trajectory').length).toBe(1);
  const sent=fixture.publishes.find(p=>p.topic==='/arm_controller/joint_trajectory')!.msg;
  expect(sent.joint_names).toEqual(['joint_link8']);
  expect((sent.points as {positions:number[]}[]).at(-1)!.positions).toEqual([0.35]);
  await row.locator('[data-bound="upper"]').dblclick();
  const editor = row.locator('[data-bound-editor="upper"]');
  await editor.fill('0.25'); await editor.press('Enter');
  await expect(slider).toHaveAttribute('max','0.25');
  await expect(row.locator('[data-bound="upper"]')).toHaveText('0.250');
  await page.locator('[data-joint-name]').selectOption('joint_link8');
  await page.locator('[data-joint-target]').fill('0.3');
  await page.locator('[data-joint-send]').click();
  await expect(page.locator('[data-joint-result]')).toContainText('软限');
  expect(fixture.publishes.filter(p=>p.topic==='/arm_controller/joint_trajectory')).toHaveLength(1);
  await page.reload();
  await expect(page.locator('#jointRows [data-joint="joint_link8"] input[type="range"]')).toHaveAttribute('max','0.25');
  expect(fixture.errors).toEqual([]);
});

test("关键参数面板缺失时不建立 ROS 连接", async ({ page }) => {
  const fixture = await connectFixture(page);
  const html = readFileSync(
    new URL("../../../web/index.html", import.meta.url),
    "utf8",
  );
  await page.route("**/index.html", (route) =>
    route.fulfill({
      contentType: "text/html",
      body: html.replace(
        'data-panel-template="deviceParams"',
        'data-panel-template="missingParams"',
      ),
    }),
  );
  await page.goto("/workbench/#/run");
  await expect(page.locator("main > .panel-error")).toContainText("关键面板");
  expect(fixture.connections.count).toBe(0);
});

test("非关键观察面板失败时仍能读取设备状态", async ({ page }) => {
  const fixture = await connectFixture(page);
  const html = readFileSync(
    new URL("../../../web/index.html", import.meta.url),
    "utf8",
  );
  await page.route("**/index.html", (route) =>
    route.fulfill({
      contentType: "text/html",
      body: html.replace(
        'data-panel-template="curve"',
        'data-panel-template="missingCurve"',
      ),
    }),
  );
  await page.goto("/workbench/#/run");
  await expect(
    page.locator('[data-workbench-panel="curve"] .panel-error'),
  ).toContainText("缺少面板模板");
  await expect(page.locator("#selRows")).toContainText("0.150");
  expect(fixture.connections.count).toBe(1);
  expect(fixture.errors).toEqual([]);
});

test("A 工作台接入真实模块，切屏保留草稿且导航不产生控制消息", async ({
  page,
}) => {
  const fixture = await connectFixture(page);
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("0.150");
  await expect(page.locator("[data-workbench-panel]")).toHaveCount(13);
  await page.locator("#cmdRows .c-pos").fill("0.25");
  await page.locator(".main-nav button").filter({ hasText: "设备工作台" }).click();
  await expect(
    page.locator('[data-workbench-panel="deviceParams"]'),
  ).toBeVisible();
  await expect(page.locator('[data-workbench-panel="arm"]')).toBeHidden();
  await expect(page.locator('[data-workbench-panel="curve"]')).toBeHidden();
  await expect(page.locator('[data-workbench-panel="jointList"]')).toBeHidden();
  await expect(page.locator('[data-workbench-panel="actionLibrary"]')).toBeHidden();
  await page.locator(".main-nav button").filter({ hasText: "仿真" }).click();
  await expect(page.locator('[data-workbench-panel="sim"]')).toBeVisible();
  await page.locator(".main-nav button").filter({ hasText: "调试工作台" }).click();
  await expect(page.locator("#cmdRows .c-pos")).toHaveValue("0.25");
  expect(
    fixture.publishes.filter((p) => p.topic !== "/motor/system_command"),
  ).toEqual([]);
  expect(
    fixture.publishes.every(
      (p) => p.topic !== "/motor/system_command" || p.msg.command === 4,
    ),
  ).toBe(true);
  expect(fixture.errors).toEqual([]);
});

test("全局软停显式取消轨迹，并清空主机设定值", async ({ page }) => {
  const fixture = await connectFixture(page);
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("0.150");
  await page
    .getByRole("button", { name: /软停|停止运动|停止动作/ })
    .first()
    .click();
  await expect
    .poll(() =>
      fixture.publishes.some((p) => p.topic === "/motor_command/setpoint"),
    )
    .toBe(true);
  expect(
    fixture.publishes.find((p) => p.topic === "/motor/trajectory")?.msg,
  ).toMatchObject({ count: 0, target_count: 0 });
  expect(
    fixture.publishes.find((p) => p.topic === "/motor_command/setpoint")?.msg,
  ).toMatchObject({ count: 0, targets: [] });
  expect(fixture.errors).toEqual([]);
});

test("静态工作台在没有 ROS 连接时显示离线，不生成模拟设备", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.route("**/api/**", (route) =>
    route.fulfill({
      json: { models: [], actions: [], joints: {}, motor_types: {} },
    }),
  );
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => socket.close());
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("等待");
  await expect(page.locator("#selRows tr[data-id]")).toHaveCount(0);
  await expect(page.locator("#btnSendOnce")).toBeDisabled();
  for (const width of [1280, 800, 390]) {
    await page.setViewportSize({ width, height: 900 });
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
  }
  expect(errors).toEqual([]);
});

test('主机限位阻止超限手动目标及整条动作，刷新后恢复', async ({ page }) => {
  const fixture = await connectFixture(page);
  await page.route('**/api/actions**', route => route.fulfill({json:
    new URL(route.request().url()).pathname.endsWith('/limited.json')
      ? {target_ids: [1], points: [
        {time_from_start_ms: 0, motor_id: 1, position: 0},
        {time_from_start_ms: 1000, motor_id: 1, position: 1}]}
      : {actions: [{name: 'limited.json', target_ids: [1], target_count: 1, point_count: 2, duration_ms: 1000}]}}));
  await page.goto('/workbench/#/run');
  await page.getByRole('button', {name: '主机限位', exact: true}).click();
  await page.getByRole('button', {name: '添加电机', exact: true}).click();
  await page.getByLabel('最小位置 rad', {exact: true}).fill('-0.417');
  await page.getByLabel('最大位置 rad', {exact: true}).fill('0.304');
  await page.locator('.host-limits input[type="checkbox"]').check();
  await page.getByRole('button', {name: '保存限位', exact: true}).click();
  await expect(page.locator('.host-limits [role="status"]')).toContainText('已保存');
  await page.getByRole('button', {name: '控制', exact: true}).click();
  await expect(page.getByLabel('当前已保存主机限位')).toContainText('m1：-0.417 ～ 0.304 rad · 已启用');
  await page.locator('#cmdRows .c-pos').fill('1');
  await page.locator('#btnSendOnce').click();
  await expect(page.locator('#cmdResult')).toContainText('主机限位');
  await expect(page.getByLabel('当前已保存主机限位')).toContainText('-0.417 ～ 0.304 rad');
  await page.locator('button[data-action="play"]').click();
  await expect(page.locator('[data-workbench-panel="actionLibrary"]')).toContainText('主机限位');
  expect(fixture.publishes.filter(p => ['/motor/policy_array', '/motor/trajectory', '/motor_command/setpoint'].includes(p.topic))).toEqual([]);
  await page.reload();
  await page.getByRole('button', {name: '主机限位', exact: true}).click();
  await expect(page.getByLabel('最小位置 rad', {exact: true})).toHaveValue('-0.417');
  await expect(page.locator('.host-limits input[type="checkbox"]')).toBeChecked();
});


test("差动组通过关节目标入口发送标准轨迹", async ({page}) => {
  const fixture = await connectFixture(page, true);
  await page.goto("/workbench/#/run");
  await expect(page.locator("[data-joint-name]")).toHaveValue("joint_link8");
  await page.locator("[data-joint-target]").fill("0.4");
  await expect(page.locator("[data-joint-send]")).toBeEnabled();
  await page.locator("[data-joint-send]").click();
  await expect.poll(() => fixture.publishes.some(p => p.topic === "/arm_controller/joint_trajectory")).toBe(true);
  const command = fixture.publishes.find(p => p.topic === "/arm_controller/joint_trajectory")!.msg;
  expect(command.joint_names).toEqual(["joint_link8"]);
  expect((command.points as {positions: number[]}[])[0].positions).toEqual([0.4]);
  expect(fixture.publishes.filter(p => p.topic === "/motor_command/setpoint")).toEqual([]);
  expect(fixture.errors).toEqual([]);
});


test('关节控制复选框切换主机只读，未使能电机仍不可发送', async ({page}) => {
  const fixture = await connectFixture(page, true);
  await page.goto('/workbench/#/run');
  await expect(page.locator('[data-joint="joint_link8"]')).toBeVisible();
  fixture.sendFeedback('/robot_transmission/status', {data: JSON.stringify({model: 'half_body_robot',
    trajectory_enabled: false, joint_names: ['joint_link8','joint_link9']})});
  const toggle = page.getByRole('checkbox', {name: '使能控制', exact: true});
  const slider = page.locator('[data-joint="joint_link8"] input[type="range"]');
  await expect(toggle).not.toBeChecked();
  await expect(slider).toBeDisabled();
  await toggle.click();
  await expect.poll(() => fixture.controlRequests).toEqual([true]);
  await expect(toggle).toBeChecked();
  await expect(slider).toBeEnabled();
  expect(fixture.publishes.filter(p => p.topic !== '/motor/system_command' || p.msg.command !== 4)).toHaveLength(0);
  fixture.sendFeedback('/motor/status_array', {statuses: [8,9].map(motor_id =>
    ({motor_id, online:true, lifecycle:3, unified_fault_code:0}))});
  await expect(slider).toBeDisabled();
  await toggle.click();
  await expect.poll(() => fixture.controlRequests).toEqual([true,false]);
  await expect(toggle).not.toBeChecked();
  await expect(slider).toBeDisabled();
});

test('已下发使能的就绪电机可发送首个关节目标，无需先变成 ACTIVE', async ({page}) => {
  const fixture = await connectFixture(page, true, true);
  await page.goto('/workbench/#/run');
  const row = page.locator('[data-joint="joint_link8"]');
  await expect(row).toBeVisible();
  fixture.sendFeedback('/motor/status_array', {statuses: [8,9].map(motor_id =>
    ({motor_id,online:true,lifecycle:3,unified_fault_code:0}))});
  const slider = row.locator('input[type="range"]');
  await expect(slider).toBeDisabled();
  fixture.sendFeedback('/motor/device_param_report', {count:2, entries:[8,9].map(motor_id =>
    ({motor_id,dev_init_state:2,rec_safety:0,dev_safety:0}))});
  await page.locator('[data-joint="joint_link8"] .jid').click();
  await expect(page.locator('#polRows [data-pol-field="enabled"]')).toHaveCount(2);
  await page.locator('#chkPolEnableAll').check();
  await page.locator('#btnPolSendAll').click();
  await expect.poll(() => fixture.publishes.some(p => p.topic === '/motor/policy_array')).toBe(true);
  const policy = fixture.publishes.find(p => p.topic === '/motor/policy_array')!.msg;
  for (const motor_id of [8,9]) fixture.sendFeedback('/motor/events',
    {event_type:13,motor_id,reason:0,request_id:policy.request_id});
  fixture.sendControlState(true);
  await expect(slider).toBeEnabled();
  await slider.evaluate(e => {e.value = '0.3'; e.dispatchEvent(new Event('change', {bubbles:true}));});
  await expect.poll(() => fixture.publishes.filter(p => p.topic === '/arm_controller/joint_trajectory').length).toBe(1);
  fixture.sendFeedback('/motor/status_array', {statuses: [8,9].map(motor_id =>
    ({motor_id,online:true,lifecycle:6,unified_fault_code:2}))});
  fixture.sendControlState(false, 2);
  await expect(slider).toBeDisabled();
  expect(fixture.errors).toEqual([]);
});


test('MCU 周期快照独立确认控制状态，本地使能缓存不能冒充设备授权', async ({page}) => {
  const fixture=await connectFixture(page,true);
  await page.goto('/workbench/#/run');
  const slider=page.locator('[data-joint="joint_link8"] input[type="range"]');
  await expect(slider).toBeVisible();
  const member=(motor_id:number, enabled:boolean)=>({motor_id,online:true,control_lifecycle:enabled?3:2,
    execution_state:1,policy_enabled:enabled,motion_enabled:false,state_age_ms:5,
    unified_fault_code:0,vendor_fault_code:0,last_failure_reason:0,policy_generation:1});
  fixture.sendFeedback('/motor/control_states',{schema_version:1,session_id:100,sequence:1,
    states:[member(8,false),member(9,false)]});
  await expect(slider).toBeDisabled();
  fixture.sendFeedback('/motor/device_param_report',{count:2,entries:[8,9].map(motor_id=>
    ({motor_id,dev_init_state:2,rec_safety:0,dev_safety:0}))});
  await page.locator('[data-joint="joint_link8"] .jid').click();
  await expect(page.locator('#polRows [data-pol-field="enabled"]')).toHaveCount(2);
  await page.locator('#chkPolEnableAll').check();
  await page.locator('#btnPolSendAll').click();
  await expect(page.locator('#polResult')).toContainText('请求已发送');
  await expect(slider).toBeDisabled();
  await expect(page.locator('#polActual')).toContainText('未使能');
  fixture.sendFeedback('/motor/control_states',{schema_version:1,session_id:101,sequence:1,
    states:[member(8,true),member(9,true)]});
  await expect(slider).toBeEnabled();
  await expect(page.locator('#polActual')).toContainText('已使能');
  fixture.sendFeedback('/motor/control_states',{schema_version:2,session_id:101,sequence:500,states:[]});
  await expect(slider).toBeDisabled();
  await expect(page.locator('#polActual')).toContainText('状态待更新');
  expect(fixture.errors).toEqual([]);
});


test('选择差动关节后可直接使能整组，并保留每台增益与拒绝回执', async ({page}) => {
  const fixture = await connectFixture(page, true, true);
  await page.route('**/api/model-transmissions?*', route => route.fulfill({json:{
    model:'half_body_robot',driver_file:'/fixture/driver.yaml',direct_joints:{},
    policy_defaults:{8:{kp:0.008,kd:1},9:{kp:0.009,kd:1.2}},
    groups:[{name:'wrist',motors:[{name:'m8',id:8},{name:'m9',id:9}],
      joints:['joint_link8','joint_link9'],direct_mapping:false}]}}));
  await page.goto('/workbench/#/run');
  await expect(page.locator('[data-joint="joint_link8"]')).toBeVisible();
  fixture.sendFeedback('/motor/status_array',{statuses:[8,9].map(motor_id =>
    ({motor_id,online:true,lifecycle:3,unified_fault_code:0}))});
  const slider=page.locator('[data-joint="joint_link8"] input[type="range"]');
  await expect(slider).toBeDisabled();
  await page.locator('[data-joint="joint_link8"] .jid').click();
  await expect(page.locator('#polRows [data-pol-field="enabled"]')).toHaveCount(2);
  await expect(page.locator('#polRows [data-id="8"][data-pol-field="kp"]')).toHaveValue('0.008');
  const kp9 = page.locator('#polRows [data-id="9"][data-pol-field="kp"]');
  await kp9.fill('0.012');
  await expect(kp9).toHaveValue('0.012');
  await page.getByRole('button',{name:'使能所选组',exact:true}).click();
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/motor/policy_array').length).toBe(1);
  const policy=fixture.publishes.find(p=>p.topic==='/motor/policy_array')!.msg;
  expect(policy).toMatchObject({count:2,targets:[
    {motor_id:8,enabled:true,kp:0.008,kd:1},
    {motor_id:9,enabled:true,kp:0.012,kd:1.2}]});
  fixture.sendFeedback('/motor/events',{event_type:13,motor_id:8,reason:0,request_id:policy.request_id});
  fixture.sendFeedback('/motor/events',{event_type:13,motor_id:9,reason:2,request_id:policy.request_id});
  await expect(page.locator('#polActual')).toContainText('m9：策略被拒绝');
  await expect(slider).toBeDisabled();
  await page.getByRole('button',{name:'使能所选组',exact:true}).click();
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/motor/policy_array').length).toBe(2);
  const retry=fixture.publishes.filter(p=>p.topic==='/motor/policy_array')[1].msg;
  for(const motor_id of [8,9]) fixture.sendFeedback('/motor/events',
    {event_type:13,motor_id,reason:0,request_id:retry.request_id});
  fixture.sendControlState(true);
  await expect(page.locator('#polActual')).toContainText('m9：策略已受理');
  await expect(slider).toBeEnabled();
  await page.locator('[data-joint-target]').fill('0.22');
  await page.locator('[data-joint-send]').click();
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/arm_controller/joint_trajectory').length).toBe(1);
  await page.getByRole('button',{name:'失能所选组',exact:true}).click();
  fixture.sendControlState(false);
  await expect(slider).toBeDisabled();
  const disabled=fixture.publishes.filter(p=>p.topic==='/motor/policy_array').at(-1)!.msg;
  expect(disabled).toMatchObject({count:2,targets:[{motor_id:8,enabled:false},{motor_id:9,enabled:false}]});
  expect(fixture.errors).toEqual([]);
});


test('切换传动组会显示完整使能行，同时保留原组未提交的增益草稿', async ({page}) => {
  await connectFixture(page,true);
  await page.route('**/api/model-transmissions?*', route=>route.fulfill({json:{
    model:'half_body_robot',direct_joints:{1:'joint_link1'},
    groups:[{name:'direct',motors:[{id:1}],joints:['joint_link1']},
      {name:'wrist',motors:[{id:8},{id:9}],joints:['joint_link8','joint_link9']}],
    policy_defaults:{1:{kp:30,kd:1},8:{kp:0.008,kd:1},9:{kp:0.008,kd:1}}}}));
  await page.goto('/workbench/#/run');
  const gain=page.locator('#polRows [data-id="1"][data-pol-field="kp"]');
  await expect(gain).toBeVisible();
  await gain.fill('12');
  await page.locator('[data-joint="joint_link8"] .jid').click();
  await expect(page.locator('#polRows [data-pol-field="enabled"]')).toHaveCount(2);
  await page.locator('[data-joint="joint_link1"] .jid').click();
  await expect(gain).toHaveValue('12');
});


test('滑块无需松手连续发送最新目标，松手不重复，关闭控制后停止发送', async ({page}) => {
  const fixture=await connectFixture(page,true);
  await page.goto('/workbench/#/run');
  const slider=page.locator('[data-joint="joint_link8"] input[type="range"]');
  await expect(slider).toBeEnabled();
  await slider.evaluate(e=>{
    for(const value of ['0.25','0.26','0.27']) {
      e.value=value;e.dispatchEvent(new Event('input',{bubbles:true}));
    }
  });
  const sent=()=>fixture.publishes.filter(p=>p.topic==='/arm_controller/joint_trajectory');
  await expect.poll(()=>sent().length).toBe(2);
  expect((sent()[1].msg.points as {positions:number[]}[]).at(-1)!.positions).toEqual([.27]);
  await slider.evaluate(e=>e.dispatchEvent(new Event('change',{bubbles:true})));
  await page.getByRole('checkbox',{name:'使能控制',exact:true}).click();
  await expect(page.getByRole('checkbox',{name:'使能控制',exact:true})).not.toBeChecked();
  await expect(slider).toBeDisabled();
  await slider.evaluate(e=>{e.value='0.3';e.dispatchEvent(new Event('input',{bubbles:true}));});
  expect(sent()).toHaveLength(2);
  expect(fixture.errors).toEqual([]);
});


test("设备工作台直接显示所选电机参数，直连快照不混入 MCU", async ({page}) => {
  const fixture = await connectFixture(page);
  const requests: {action: string; body: Record<string, unknown>}[] = [];
  await page.route('**/api/device-can/**', async route => {
    const action = new URL(route.request().url()).pathname.split('/').pop()!;
    if (action === 'options') return route.fulfill({json: {socketcan: ['fixture-can'], slcan: ['/fixture-tty']}});
    const body = route.request().postDataJSON(); requests.push({action, body});
    if (action === 'connect') return route.fulfill({json: {session: 'fixture-session', transport: body.transport, endpoint: body.endpoint}});
    if (action === 'read' || action === 'discover') return route.fulfill({json: {motors: [
      {id: 2, key: 'cerb:2', protocol: 'cerb', online: true, read_at: 1792836000, fields: [{key: 'identity', label: '设备型号', object: '0x1008:00', value: 'R48LITE_V2', unit: ''}, {key: 'kp_scale', label: 'MIT Kp 满量程', object: '0x2544:00', value: 2500, unit: 'Nm/rad'}]},
      {id: 3, key: 'vaxor:3', protocol: 'vaxor', online: false, read_at: 1792836000, fields: [{key: 'identity', label: '设备型号', object: '0x1008:00', error: 'No SDO response', unit: ''}]}]}});
    return route.fulfill({json: {connected: action !== 'disconnect'}});
  });
  await page.goto('/workbench/#/teach');
  await expect(page.locator('[data-workbench-panel="deviceParams"]')).toBeVisible();
  await expect(page.locator('.object-heading')).toBeHidden();
  await expect(page.locator('.inspector-tabs')).toBeHidden();
  await expect(page.getByLabel('当前已保存主机限位')).toBeHidden();
  await expect(page.locator('.motor-card[data-id="1"]')).toBeVisible();
  fixture.sendFeedback('/motor/topology', {entries: [
    {motor_id: 1, online: true, model_name: 'cerb_r48_101', bus_index: 0, lifecycle: 3},
    {motor_id: 2, online: true, model_name: 'cerb_r58_101', bus_index: 0, lifecycle: 3},
  ]});
  await page.locator('.motor-card[data-id="2"]').click();
  await expect(page.locator('.motor-specifications h3').first()).toContainText('电机 2 · cerb_r58_101');
  await page.getByLabel('设备通信来源').selectOption('socketcan');
  await expect(page.locator('.workspace-grid')).toBeHidden();
  await page.getByRole('button', {name: '连接 CAN', exact: true}).click();
  await expect(page.getByLabel('直连节点 ID')).toHaveCount(0);
  await page.getByRole('button', {name: '读取电机参数', exact: true}).click();
  await expect(page.locator('.device-detail-table')).toContainText('2500');
  await expect(page.locator('.direct-motor')).toHaveCount(2);
  await page.locator('.direct-motor').filter({hasText: 'm3'}).click();
  await expect(page.locator('.device-detail-table')).toContainText('No SDO response');
  expect(requests.find(r => r.action === 'read')?.body.node_ids).toBeUndefined();
  expect(requests.some(r => r.action === 'discover')).toBe(true);
  await page.getByRole('button', {name: '断开 CAN', exact: true}).click();
  await page.getByLabel('设备通信来源').selectOption('firmware');
  await expect(page.locator('[data-workbench-panel="deviceParams"]')).toBeVisible();
  expect(fixture.publishes.filter(p => ['/motor/policy_array', '/motor/trajectory', '/motor_command/setpoint'].includes(p.topic))).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test("侧栏可切换为图标并记住选择，移除底部说明", async ({page}) => {
  await connectFixture(page);
  await page.goto('/workbench/#/run');
  const nav = page.getByRole('navigation', {name: '工作场景'});
  const before = (await nav.boundingBox())!.width;
  await expect(nav).not.toContainText('WORKBENCH / A');
  await page.getByRole('button', {name: '收起侧栏', exact: true}).click();
  await expect.poll(async () => (await nav.boundingBox())!.width).toBe(64);
  expect(before).toBeGreaterThan(64);
  await expect(nav.getByRole('button', {name: '设备工作台'})).toBeVisible();
  await expect(nav.getByRole('button', {name: '设备工作台'}).locator('span')).toBeHidden();
  await page.reload();
  await expect(page.getByRole('button', {name: '展开侧栏', exact: true})).toBeVisible();
  await page.getByRole('button', {name: '展开侧栏', exact: true}).click();
  await expect.poll(async () => (await nav.boundingBox())!.width).toBe(before);
  await expect(nav.getByRole('button', {name: '设备工作台'}).locator('span')).toBeVisible();
});

test("动作库折叠释放面板占位，展开恢复内容", async ({page}) => {
  await connectFixture(page);
  await page.goto('/workbench/#/run');
  const panel = page.locator('[data-workbench-panel="actionLibrary"]');
  const summary = panel.locator('summary');
  await expect(summary).toBeVisible();
  const before = (await panel.boundingBox())!.height;
  const robot = page.locator('[data-workbench-panel="arm"]');
  const robotBefore = (await robot.boundingBox())!.height;
  await summary.click();
  await expect.poll(async () => (await panel.boundingBox())!.height).toBeLessThan(45);
  await expect.poll(async () => (await robot.boundingBox())!.height).toBeGreaterThan(robotBefore);
  await summary.click();
  await expect.poll(async () => (await panel.boundingBox())!.height).toBeGreaterThanOrEqual(before - 1);
});

test('拖动分隔线调整窗口尺寸并在刷新后保留', async ({page}) => {
  await page.setViewportSize({width: 1806, height: 1448});
  await connectFixture(page);
  await page.goto('/workbench/#/run');
  await expect(page.locator('#run-joints summary')).toBeVisible();
  const objects = page.locator('.objects-area');
  const before = (await objects.boundingBox())!.width;
  const separator = page.getByRole('separator', {name: '调整对象浏览器宽度'});
  let box = (await separator.boundingBox())!;
  await page.mouse.move(box.x + box.width / 2, box.y + 50);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width / 2 + 60, box.y + 50);
  await page.mouse.up();
  await expect.poll(async () => (await objects.boundingBox())!.width).toBe(before + 60);
  const observer = page.locator('.observer-area');
  const height = (await observer.boundingBox())!.height;
  const horizontal = page.getByRole('separator', {name: '调整观察区高度'});
  box = (await horizontal.boundingBox())!;
  await page.mouse.move(box.x + 40, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + 40, box.y + box.height / 2 - 60);
  await page.mouse.up();
  await expect.poll(async () => (await observer.boundingBox())!.height).toBe(height + 60);
  await page.reload();
  await expect.poll(async () => (await objects.boundingBox())!.width).toBe(before + 60);
  await expect.poll(async () => (await observer.boundingBox())!.height).toBe(height + 60);
  const action = page.locator('.action-panel-host');
  await expect(page.locator('#actList')).toBeAttached();
  const actionBefore = (await action.boundingBox())!.height;
  await page.getByRole('separator', {name: '调整动作库高度'}).focus();
  await page.keyboard.press('ArrowDown');
  await expect.poll(async () => (await action.boundingBox())!.height).toBeLessThan(actionBefore);
});

for (const viewport of [{width:1063, height:752}, {width:1600, height:650}, {width:900, height:752}, {width:640, height:752}]) {
  test(`外层不滚动，工作区适应窗口 ${viewport.width}×${viewport.height}`, async ({page}) => {
    await page.setViewportSize(viewport);
    await connectFixture(page);
    await page.goto('/workbench/#/run');
    await expect(page.locator('.startup-status')).toHaveCount(0);
    const main = page.locator('.main-workspace');
    expect(await main.evaluate(el => getComputedStyle(el).overflowY)).toBe('hidden');
    expect(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight)).toBe(true);
    const grid = (await page.locator('.workspace-grid').boundingBox())!;
    const workspace = (await main.boundingBox())!;
    expect(grid.y + grid.height).toBeLessThanOrEqual(workspace.y + workspace.height);
    if (viewport.width <= 650)
      await page.locator('.workspace-grid').evaluate(el => {el.scrollTop = el.scrollHeight;});
    await expect(page.locator('.observer-head')).toBeInViewport({ratio: 1});
    await expect(page.locator('.app-footer')).toBeInViewport({ratio: 1});
    await page.locator('.workspace-heading').hover();
    await page.mouse.wheel(0, 400);
    expect(await main.evaluate(el => el.scrollTop)).toBe(0);
    expect(await page.evaluate(() => window.scrollY)).toBe(0);
  });
}

test('设备工作台在矮窗口中保持详情位于内部滚动区', async ({page}) => {
  await page.setViewportSize({width:1063, height:650});
  await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await expect(page.locator('.startup-status')).toHaveCount(0);
  const main = (await page.locator('.main-workspace').boundingBox())!;
  const inspector = (await page.locator('.inspector-area').boundingBox())!;
  expect(inspector.y + inspector.height).toBeLessThanOrEqual(main.y + main.height);
  const pane = page.locator('.inspector-scroll');
  expect(await pane.evaluate(el => getComputedStyle(el).overflowY)).toBe('auto');
  await pane.evaluate(el => {el.scrollTop = el.scrollHeight;});
  await expect(page.locator('.motor-specifications')).toBeInViewport();
  await page.getByLabel('设备通信来源').selectOption('socketcan');
  const connectionPane = page.locator('.main-workspace > div').filter({has: page.locator('.device-connection')});
  expect(await connectionPane.evaluate(el => getComputedStyle(el).overflowY)).toBe('auto');
  expect(await connectionPane.evaluate(el => el.scrollHeight > el.clientHeight)).toBe(true);
  await connectionPane.evaluate(el => {el.scrollTop = el.scrollHeight;});
  await expect(page.locator('.direct-device-grid')).toBeInViewport();
});

test('动作库只有列表内部滚动，标题和操作栏固定', async ({page}) => {
  await connectFixture(page);
  await page.route('**/api/actions**', route => route.fulfill({json: {actions:
    Array.from({length: 50}, (_, i) => ({name: `action-${i}.json`, target_ids:[1], target_count:1, point_count:20, duration_ms:1000}))}}));
  await page.goto('/workbench/#/run');
  const panel = page.locator('.action-panel-host');
  await expect(panel.locator('.act-row')).toHaveCount(50);
  expect(await panel.evaluate(el => getComputedStyle(el).overflowY)).toBe('hidden');
  expect(await panel.evaluate(el => el.scrollHeight <= el.clientHeight + 1)).toBe(true);
  const list = page.locator('#actList');
  expect(await list.evaluate(el => el.scrollHeight > el.clientHeight)).toBe(true);
  for (const selector of ['.stage-body', '.inspector-scroll', '#actList']) {
    expect(await page.locator(selector).first().evaluate(el => getComputedStyle(el).scrollbarWidth)).not.toBe('none');
  }
  await list.hover();
  await page.mouse.wheel(0, 200);
  await expect.poll(() => list.evaluate(el => el.scrollTop)).toBeGreaterThan(0);
  await expect(panel.locator('summary')).toBeVisible();
  await expect(page.locator('#btnActImport')).toBeVisible();
});

test('工作台曲线支持复选及滚轮缩放，画布随观察区伸缩', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  await expect(page.locator('#chartHint')).toContainText('10 s');
  await expect(page.locator('#chartWindow')).toHaveCount(0);
  await expect(page.locator('[data-chart-motor-summary]')).toHaveText('电机 1');
  await expect(page.locator('[data-chart-channel-summary]')).toHaveText('位置 rad');
  await expect(page.locator('#chartHint')).toContainText('范围');
  fixture.sendFeedback('/motor/topology', {entries: [1,2].map(motor_id => ({motor_id,
    online: true, model_name:'cerb_r48_101', bus_index:0, lifecycle:3}))});
  await page.locator('#chartMotor summary').click();
  await page.locator('input[name="chart-motor"][value="2"]').check();
  await page.locator('#chartCh summary').click();
  await page.locator('input[name="chart-channel"][value="velocity"]').check();
  await expect(page.locator('#chartLegend .curve-legend-item')).toHaveCount(4);
  await page.locator('#chartCh summary').click();
  const canvas = page.locator('#chart');
  await canvas.hover();
  await page.mouse.wheel(0, -120);
  await expect(page.locator('#chartHint')).toContainText('9 s');
  const before = (await canvas.boundingBox())!.height;
  await page.getByRole('separator', {name:'调整观察区高度'}).focus();
  await page.keyboard.press('ArrowUp');
  await expect.poll(async () => (await canvas.boundingBox())!.height).toBeGreaterThan(before);
});

test('动作库在210px处仍能向下拖小，并能反向拖大和保留尺寸', async ({page}) => {
  await page.setViewportSize({width:1417, height:965});
  await page.addInitScript(() => {
    if (localStorage.getItem('robot2.layout.--actions-height') === null) {
      localStorage.setItem('robot2.layout.--actions-height', '210');
      localStorage.setItem('robot2.layout.--joints-height', '75');
      localStorage.setItem('robot2.navCollapsed', 'true');
    }
  });
  await connectFixture(page);
  await page.goto('/workbench/#/run');
  const panel = page.locator('.action-panel-host');
  await expect(panel.locator('summary')).toBeVisible();
  await page.getByRole('button', {name:'收起观察区', exact:true}).click();
  await expect.poll(async () => (await panel.boundingBox())!.height).toBe(210);
  const handle = page.getByRole('separator', {name:'调整动作库高度', exact:true});
  const drag = async (dy:number) => {
    const box = (await handle.boundingBox())!;
    await page.mouse.move(box.x + 80, box.y + box.height/2);
    await page.mouse.down();
    await page.mouse.move(box.x + 80, box.y + box.height/2 + dy, {steps:5});
    await page.mouse.up();
  };
  await drag(40);
  await expect.poll(async () => (await panel.boundingBox())!.height).toBe(170);
  await drag(-40);
  await expect.poll(async () => (await panel.boundingBox())!.height).toBe(210);
  await drag(40);
  await page.reload();
  await expect.poll(async () => (await panel.boundingBox())!.height).toBe(170);
});
