import { test, expect, type Page } from "@playwright/test";
import { readFileSync } from "node:fs";

async function connectFixture(page: Page, coupled = false, withControlStates = false) {
  const publishes: { topic: string; msg: Record<string, unknown> }[] = [];
  const errors: string[] = [];
  const connections = { count: 0 };
  const feedbackModels: string[] = [];
  const gravityRequests: {service: string; args: unknown}[] = [];
  const gravityReply = {error: ''};
  const policyRequests: {session_id:number;source_time_ns:number;targets:any[];ticket:any}[] = [];
  let policyGeneration=1;
  const parameterReplies = {enabled: false, refreshError: 12};
  const firmware = {values: {'system.agent_ping_hz': 100, 'system.motion_report_hz': 50,
    'system.status_report_hz': 10, 'system.imu_report_hz': 100, 'system.temperature_report_hz': 2,
    'system.motor_command_hz': 200,
    'system.can_nominal_kbps': 1000, 'system.can_data_kbps': 5000, 'system.unsaved': false} as Record<string, number | boolean>};
  const limitRequests: {model: string; joint_limits: Record<string, {lower: number; upper: number}>}[] = [];
  const limitReply = {successful: true, confirm: true};
  let controlSequence = 0;
  let sendFeedback: (topic: string, msg: unknown) => void;
  page.on("pageerror", (error) => errors.push(error.message));
  await page.route("**/api/**", (route) => {
    const url = new URL(route.request().url());
    if (url.pathname === "/api/runtime")
      return route.fulfill({json: {control_mode: 'bench', robot_model: 'half_body_robot'}});
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
              policy_defaults:{1:{kp:30,kd:1},8:{kp:30,kd:1},9:{kp:30,kd:1}}, direct_joints: { 1: "joint_link1" } } },
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
    "/control/status": {data: JSON.stringify({model:"half_body_robot",joint_names:["joint_link1"],
      mode:"idle",reason:"no_setpoint",reject:"",gravity_enabled:false,effort_gain:1,
      feedback_fresh:true,controllers_ready:true,active_controllers:[],
      output_topic:"/motor/host_command_array",
      trajectory_output_topic:"/host_control/trajectory_controller/follow_joint_trajectory"})},
    "/robot_transmission/status": {data: JSON.stringify({model:"half_body_robot",
      joint_names:["joint_link1"],joint_limits:{joint_link1:{lower:-1.57,upper:1.57}},trajectory_enabled:true,
      control_service:'/workbench_control/set_parameters_atomically',last_error:""})},
    "/bench/status": {
      data: JSON.stringify({
        agent: { state: "stopped" },
        simulation: { state: "stopped" },
        motor_backend: { state: "stopped" },
        gravity_feedforward: {fresh:true,pending:null,error:'',config:{model:'half_body_robot',effort_gain:1}},
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
      trajectory_enabled: true, joint_names: ["joint_link8", "joint_link9"],
      joint_limits: {joint_link8:{lower:-1.047,upper:1.047},joint_link9:{lower:-1.22,upper:1.22}},
      control_service:'/workbench_control/set_parameters_atomically'})};
    fixtures["/control/status"] = {data: JSON.stringify({model:"half_body_robot",
      joint_names:["joint_link8","joint_link9"],mode:"trajectory",reason:"ok",reject:"",
      gravity_enabled:true,effort_gain:.8,output_topic:"/motor/host_command_array",
      trajectory_output_topic:"/host_control/trajectory_controller/follow_joint_trajectory"})};
  }
  const makeControlStates = (enabled: boolean, fault = 0) => ({schema_version: 2, session_id: 7001, sync_time_ns:Date.now()*1e6,
    sequence: ++controlSequence, tick_ms: controlSequence * 20,
    states: [8,9].map(motor_id => ({motor_id, online: true,
      control_lifecycle: enabled && !fault ? 3 : 2, execution_state: 1,
      policy_enabled: enabled, motion_enabled: false, state_age_ms: 5,
      unified_fault_code: fault, vendor_fault_code: 0, last_failure_reason: 0,
      policy_generation:policyGeneration}))});
  if (coupled && withControlStates)
    fixtures['/motor/control_states'] = makeControlStates(false);
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => {
    sendFeedback = (topic, msg) => {
      if (['/control/status', '/robot_transmission/status'].includes(topic)) fixtures[topic] = msg;
      socket.send(JSON.stringify({op: 'publish', topic, msg}));
    };
    const subscribed = new Set<string>();
    const firmwareNames = Object.keys(firmware.values).filter(n => !n.includes('can_') && n !== 'system.unsaved');
    const firmwareReport = () => ({...fixtures['/motor/system_status'] as object,
      firmware_values_valid: true, firmware_values: firmwareNames.map(n => Number(firmware.values[n])),
      firmware_unsaved: firmware.values['system.unsaved']});
    const timer = setInterval(() => {
      if (subscribed.has('/motor/system_status')) socket.send(JSON.stringify({op:'publish',topic:'/motor/system_status',msg:firmwareReport()}));
      for (const topic of ['/control/status', '/robot_transmission/status']) {
        if (subscribed.has(topic)) socket.send(JSON.stringify({op:'publish',topic,msg:fixtures[topic]}));
      }
    },200);
    page.once('close',()=>clearInterval(timer));

    if (coupled) {
      const heartbeat = setInterval(() => {
        for (const topic of subscribed) {
          if (topic === '/motor/control_states' && fixtures[topic])
            Object.assign(fixtures[topic] as object,{sequence:++controlSequence,sync_time_ns:Date.now()*1e6});
          if (fixtures[topic]) socket.send(JSON.stringify({op: "publish", topic, msg: fixtures[topic]}));
        }
      }, 20);
      page.once("close", () => clearInterval(heartbeat));
    }
    connections.count++;
    socket.onMessage((raw) => {
      const message = JSON.parse(String(raw));
      if (message.op === "subscribe") subscribed.add(message.topic);
      if (message.op === 'subscribe' && message.topic === '/motor/control_states' && !fixtures[message.topic]) {
        // Legacy fixtures supply only a fake MCU synchronized stamp; health remains unchanged.
        void page.evaluate(async stamp => {
          const path='/js/core/store.js';
          const {store}=await import(/* @vite-ignore */ path);
          store.controlStates.sourceTimeNs=stamp;
        },9000000000).catch(error=>errors.push(String(error)));
      }

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
        if ((message.topic === '/motor/config_request' && message.msg.kind === 2) ||
            (message.topic === '/motor/system_command' && message.msg.command === 0)) {
          throw new Error('固件固定配置不可在运行期改写或保存');
        }

        if (parameterReplies.enabled && message.topic === '/motor/system_command') {
          const error = message.msg.command === 1 ? parameterReplies.refreshError : 0;
          socket.send(JSON.stringify({op: 'publish', topic: '/motor/config_result', msg: {
            session_id: 9, request_id: message.msg.request_id, kind: 1, command: message.msg.command,
            status: error ? 2 : 1, error_code: error, completed_targets: error ? 0 : 1,
            failed_targets: error ? 1 : 0}}));
        }
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
        if (['/bench/start_gravity_feedforward', '/bench/stop_gravity_feedforward'].includes(message.service)) {
          gravityRequests.push({service:message.service,args:message.args});
          socket.send(JSON.stringify({op:'service_response',id:message.id,service:message.service,result:true,
            values:{ok:!gravityReply.error,error:gravityReply.error,status_json:JSON.stringify({gravity_feedforward:{
              fresh:true,pending:gravityReply.error?null:message.service.includes('/start_')?'start':'stop',
              error:gravityReply.error,config:{model:'half_body_robot',effort_gain:1}}})}}));
          return;
        }
        if (message.service === '/motor/submit_policy') {
          const enabled=message.args.targets.some((target:any)=>target.enabled);
          const ticket={accepted:true,session_id:7001,intent_id:`fixture:${policyRequests.length+1}`,
            request_id:enabled?50+policyRequests.length:0,policy_generation:enabled?++policyGeneration:0,reason:''};
          policyRequests.push({...message.args,ticket});
          socket.send(JSON.stringify({op:'service_response',id:message.id,service:message.service,result:true,values:ticket}));
          return;
        }
        if (message.service.startsWith('/micro_ros_node/') || message.service === '/motor/execute_config')
          throw new Error('固件不提供运行期参数或保存服务');
        if (['/firmware_topic_bridge/set_parameters_atomically','/workbench_control/set_parameters_atomically'].includes(message.service)) {
          const parameters = message.args.parameters as {name: string; value: {bool_value?: boolean; string_value?: string}}[];
          const limits = parameters.find(p => p.name === 'joint_limits_json')?.value.string_value;
          const settings = limits === undefined ? null : JSON.parse(limits);
          if (settings) limitRequests.push(settings);
          const current = JSON.parse((fixtures['/robot_transmission/status'] as {data: string}).data);
          if (settings && limitReply.successful) {
            fixtures['/robot_transmission/status'] = {data: JSON.stringify({...current,
              ...(settings && limitReply.confirm ? {joint_limits: settings.joint_limits} : {})})};
          }
          socket.send(JSON.stringify({op: 'service_response', id: message.id,
            service: message.service, result: true,
            values: {result: {successful: Boolean(settings) && limitReply.successful,
              reason: settings && !limitReply.successful ? '测试主机拒绝关节限位' : ''}}}));
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
  return { publishes, errors, connections, feedbackModels, firmware, parameterReplies, limitRequests,policyRequests,
    gravityRequests, gravityReply,
    sendGravity: (enabled: boolean, mode = 'stopped') => {
      sendFeedback('/control/status', {data:JSON.stringify({
        ...JSON.parse((fixtures['/control/status'] as {data:string}).data),
        gravity_enabled:enabled,effort_gain:enabled?1:0,mode,
        active_controllers:['follow','trajectory'].includes(mode)?[mode+'_controller','gravity_controller']:[]})});
      const status={data:JSON.stringify({gravity_feedforward:{fresh:true,pending:null,error:'',
        config:{model:'half_body_robot',effort_gain:enabled?1:0}}})};
      fixtures['/bench/status']=status;sendFeedback('/bench/status',status);
    },
    ackPolicy: () => {const ticket=policyRequests.at(-1)!.ticket;sendFeedback('/motor/config_result',{...ticket,kind:2,command:0,status:1,error_code:0,failed_targets:0,completed_targets:2});},
    setLimitReply: (successful: boolean, confirm = true) => {limitReply.successful=successful;limitReply.confirm=confirm;},
    sendFeedback: (topic: string, msg: unknown) => {fixtures[topic]=msg;sendFeedback(topic, msg);},
    sendControlState: (enabled: boolean, fault = 0) => {
      const message = makeControlStates(enabled, fault);
      fixtures['/motor/control_states'] = message;
      sendFeedback('/motor/control_states', message);
    } };
}

test('事件日志脱离旧面板后保留切换记录并导出全部数据', async ({page}) => {
  const fixture = await connectFixture(page);
  let legacyRequests = 0;
  await page.route('**/js/panels/events.js', route => {
    legacyRequests++;
    return route.fulfill({status: 404, body: '已退役'});
  });
  await page.goto('/workbench/#/run');
  await expect(page.locator('#selRows')).toContainText('0.150');
  await page.getByRole('button', {name: '事件日志', exact: true}).click();
  const panel = page.locator('[data-workbench-panel="events"]');
  await expect(panel).toContainText('暂无事件');
  for (let id = 1; id <= 55; id++) {
    fixture.sendFeedback('/motor/events', {event_type: 3, motor_id: id, reason: 2, request_id: id});
  }
  await expect(panel.locator('.sum')).toHaveText('55 条');
  await expect(panel.locator('li')).toHaveCount(50);
  await expect(panel.locator('li').first()).toContainText('READY · id=55 · reason=2 · req#55');
  await expect(panel.locator('li').last()).toContainText('id=6 ');
  await page.getByRole('button', {name: '曲线', exact: true}).click();
  await expect(panel).toBeHidden();
  fixture.sendFeedback('/motor/events', {event_type: 9, motor_id: 56, reason: 0, request_id: 0});
  await page.getByRole('button', {name: '事件日志', exact: true}).click();
  await expect(panel.locator('.sum')).toHaveText('56 条');
  await expect(panel.locator('li').first()).toContainText('OFFLINE · id=56');
  await expect(panel.locator('li').first()).not.toContainText('reason=');
  await expect(panel.locator('li').first()).not.toContainText('req#');

  const jsonDownload = page.waitForEvent('download');
  await panel.getByRole('button', {name: '导出 JSON', exact: true}).click();
  const json = await jsonDownload;
  expect(json.suggestedFilename()).toMatch(/^events-\d+\.json$/);
  const entries = readFileSync((await json.path())!, 'utf8').split('\n').map(line => JSON.parse(line));
  expect(entries).toHaveLength(56);
  expect(entries[0]).toEqual({t: expect.any(Number), name: 'OFFLINE', id: 56, reason: 0, request_id: 0});
  expect(entries[55]).toMatchObject({name: 'READY', id: 1, reason: 2, request_id: 1});

  const csvDownload = page.waitForEvent('download');
  await panel.getByRole('button', {name: '导出 CSV', exact: true}).click();
  const csv = await csvDownload;
  expect(csv.suggestedFilename()).toMatch(/^events-\d+\.csv$/);
  expect(readFileSync((await csv.path())!, 'utf8')).toBe(entries.map(entry =>
    [new Date(entry.t).toISOString(), entry.name, entry.id, entry.reason, entry.request_id].join(',')).join('\n'));
  expect(legacyRequests).toBe(0);
  expect(fixture.errors).toEqual([]);
});

test('事件日志按文本呈现并仅清空本地记录', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  await expect(page.locator('#selRows')).toContainText('0.150');
  await page.getByRole('button', {name: '事件日志', exact: true}).click();
  const panel = page.locator('[data-workbench-panel="events"]');
  fixture.sendFeedback('/motor/events', {event_type: '<em>未知</em>', motor_id: 1, reason: 0, request_id: 0});
  await expect(panel.locator('li')).toContainText('EVENT_<em>未知</em>');
  await expect(panel.locator('em')).toHaveCount(0);
  const before = fixture.publishes.length;
  await panel.getByRole('button', {name: '清空', exact: true}).click();
  await expect(panel.locator('.sum')).toHaveText('空');
  await expect(panel.locator('li')).toHaveText('暂无事件');
  expect(await page.evaluate(async () => {
    const path = '/js/core/store.js';
    return (await import(/* @vite-ignore */ path)).store.events.length;
  })).toBe(0);
  expect(fixture.publishes.slice(before)).toEqual([]);
  fixture.sendFeedback('/motor/events', {event_type: 3, motor_id: 2, reason: 0, request_id: 0});
  await expect(panel.locator('.sum')).toHaveText('1 条');
  await expect(panel.locator('li')).toContainText('READY · id=2');
  expect(fixture.errors).toEqual([]);
});

test('重力开关等待主机确认，在跟随和动作模式均显示补偿且不使能电机', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.setViewportSize({width:1333,height:965});
  await page.goto('/workbench/#/run');
  const toggle = page.getByRole('checkbox', {name:'重力补偿',exact:true});
  const status = page.locator('#gravityStatus');
  await expect(toggle).toBeEnabled();
  await expect(toggle).not.toBeChecked();
  expect(fixture.gravityRequests).toEqual([]);
  await toggle.click();
  await expect(status).toHaveText('等待主机确认');
  await expect(toggle).toBeDisabled();
  await expect(toggle).not.toBeChecked();
  expect(fixture.gravityRequests).toEqual([{service:'/bench/start_gravity_feedforward',
    args:{model:'half_body_robot',joint_map:'',effort_gain:1}}]);
  fixture.sendGravity(true);
  await expect(toggle).toBeChecked();
  await expect(status).toHaveText('已开启 · 待运动');
  for (const mode of ['follow','trajectory']) {
    fixture.sendGravity(true,mode);
    await expect(status).toHaveText('补偿中');
  }
  await toggle.click();
  await expect(status).toHaveText('等待主机确认');
  await expect(toggle).toBeChecked();
  fixture.sendGravity(false);
  await expect(toggle).not.toBeChecked();
  await expect(status).toHaveText('已关闭');
  expect(fixture.gravityRequests[1]).toEqual({service:'/bench/stop_gravity_feedforward',args:{}});
  expect(fixture.policyRequests).toEqual([]);
  expect(fixture.publishes.every(p => p.topic==='/motor/system_command' && p.msg.command===4)).toBe(true);
  await page.getByRole('button', {name:'仿真工作台',exact:true}).click();
  await expect(toggle).not.toBeVisible();
  expect(fixture.errors).toEqual([]);
});

test('重力开关保留服务拒绝原因，断联后不自动重试', async ({page}) => {
  const fixture = await connectFixture(page);
  fixture.gravityReply.error='重力模型缺少关节反馈';
  await page.goto('/workbench/#/run');
  const toggle = page.getByRole('checkbox', {name:'重力补偿',exact:true});
  await expect(toggle).toBeEnabled();
  await toggle.click();
  await expect(page.locator('#gravityStatus')).toHaveText('重力模型缺少关节反馈');
  await expect(toggle).not.toBeChecked();
  await page.getByRole('button', {name:'重新连接',exact:true}).click();
  await expect.poll(() => fixture.connections.count).toBe(2);
  await expect(page.locator('#gravityStatus')).not.toHaveText('重力模型缺少关节反馈');
  expect(fixture.gravityRequests).toHaveLength(1);
  expect(fixture.errors).toEqual([]);
});

test('重力请求受理后状态流中断显示超时并清除陈旧等待', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  const toggle = page.getByRole('checkbox', {name:'重力补偿',exact:true});
  await expect(toggle).toBeEnabled();
  await toggle.click();
  await expect(page.locator('#gravityStatus')).toHaveText('等待主机确认');
  await expect(page.locator('#gravityStatus')).toContainText('超时', {timeout:6000});
  await expect(page.locator('.gravity-control')).toHaveAttribute('aria-busy','false');
  await expect(toggle).not.toBeChecked();
  fixture.sendGravity(false);
  await expect(toggle).toBeEnabled();
  expect(fixture.gravityRequests).toHaveLength(1);
  expect(fixture.errors).toEqual([]);
});

test('布局和观察页切换保留3D场景，只有模型变化才重建', async ({page}) => {
  const { errors, publishes } = await connectFixture(page);
  await page.goto('/workbench/#/run');
  const canvas = page.locator('canvas.arm3d');
  await expect(canvas).toHaveAttribute('data-model-rendered', 'true');
  const original = await canvas.elementHandle();
  const modelRequests: string[] = [];
  page.on('request', request => {
    if (/\.(?:urdf|stl)(?:\?|$)/i.test(request.url())) modelRequests.push(request.url());
  });

  await page.getByRole('button', {name: '收起侧栏', exact: true}).click();
  await expect(page.locator('.workbench')).toHaveClass(/nav-collapsed/);
  await page.getByRole('button', {name: '展开侧栏', exact: true}).click();
  await expect(page.locator('.workbench')).not.toHaveClass(/nav-collapsed/);
  await page.getByRole('button', {name: '事件日志', exact: true}).click();
  await page.getByRole('button', {name: '曲线', exact: true}).click();
  await page.locator('[data-workbench-panel="actionLibrary"] summary').click();
  await page.getByRole('separator', {name: '调整关节面板高度'}).press('ArrowDown');
  await expect(canvas).toHaveAttribute('data-model-rendered', 'true');
  expect(await original!.evaluate(element => element.isConnected)).toBe(true);
  expect(modelRequests).toEqual([]);

  await page.locator('#armModel').selectOption('robot_leg');
  await expect.poll(() => original!.evaluate(element => element.isConnected)).toBe(false);
  await expect(canvas).toHaveAttribute('data-model-rendered', 'true');
  expect(modelRequests.some(url => url.endsWith('/robot_leg.urdf'))).toBe(true);
  expect(publishes.every(p => p.topic === '/motor/system_command' && p.msg.command === 4)).toBe(true);
  expect(errors).toEqual([]);
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
  const expandedHeight = (await joints.boundingBox())!.height;
  await summary.click();
  await expect.poll(async () => (await joints.boundingBox())!.height).toBeLessThan(45);
  await expect.poll(async () => (await robot.boundingBox())!.height).toBeGreaterThan(before);
  await summary.click();
  await expect.poll(async () => Math.abs((await joints.boundingBox())!.height-expandedHeight)).toBeLessThan(2);
  const toggle = page.locator('.observer-head').getByRole('button', {name: '收起观察区', exact: true});
  await expect(toggle).toHaveText('▾');
  await expect(toggle).toHaveAttribute('aria-expanded', 'true');
  await toggle.click();
  await expect(page.locator('.observer-fold')).toHaveAttribute('aria-expanded', 'false');
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
  await expect(page.locator('#selTitle')).toHaveText('电机状态');
  expect(await card.evaluate(e => e.scrollWidth <= e.clientWidth)).toBe(true);
  const cardNode = await card.elementHandle();
  await page.waitForTimeout(650);
  expect(await card.evaluate((current, original) => current === original, cardNode)).toBe(true);
  await card.click();
  await expect(page.locator('.object-heading')).toHaveCount(0);
  await expect(page.locator('#jointRows [data-joint="joint_link1"] .jpos')).toHaveText('0.150');
  fixture.sendFeedback('/motor/topology', {entries: [2, 1].map(motor_id => ({
    motor_id, online: true, model_name: 'cerb_r48_101', bus_index: 0, lifecycle: 3,
  }))});
  const cards = page.locator('.motor-card');
  await expect(cards).toHaveCount(2);
  const order = () => cards.evaluateAll(nodes => nodes.map(n => n.getAttribute('data-id')));
  expect(await order()).toEqual(['1', '2']);
  await page.locator('.motor-card[data-id="2"]').click();
  await expect(page.locator('.motor-card[data-id="2"]')).toHaveAttribute('aria-pressed', 'true');
  expect(await order()).toEqual(['1', '2']);
  await card.click();
  await expect(card).toHaveAttribute('aria-pressed', 'true');
  expect(await order()).toEqual(['1', '2']);
  expect(fixture.publishes.filter(p => !['/motor/system_command','/motor/config_request'].includes(p.topic))).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

for (const width of [1333, 640]) {
  test(`仿真页移除电机与关节侧栏并释放占位 ${width}px`, async ({page}) => {
    await page.setViewportSize({width, height: 965});
    const fixture = await connectFixture(page);
    await page.goto('/workbench/#/sim');
    await expect(page.locator('canvas.arm3d')).toHaveAttribute('data-model-rendered', 'true');
    await expect(page.locator('.objects-area')).toBeHidden();
    await expect(page.locator('#jointRows')).toBeHidden();
    await expect(page.getByRole('separator', {name: '调整对象浏览器宽度'})).toBeHidden();
    await expect(page.getByRole('navigation', {name: '工作区面板'}).getByRole('button', {name: /^电机/})).toHaveCount(0);
    await expect(page.locator('[data-workbench-panel="sim"]')).toBeVisible();
    const grid = (await page.locator('.workspace-grid').boundingBox())!;
    const stage = (await page.locator('.stage-area').boundingBox())!;
    expect(Math.abs(stage.x - grid.x)).toBeLessThan(2);
    expect(Math.abs(stage.width - grid.width)).toBeLessThan(2);
    if (width > 950) {
      const observer = (await page.locator('.observer-area').boundingBox())!;
      const separator = (await page.getByRole('separator', {name: '调整观察区高度'}).boundingBox())!;
      expect(Math.abs(observer.x - grid.x)).toBeLessThan(2);
      expect(Math.abs(observer.width - grid.width)).toBeLessThan(2);
      expect(Math.abs(separator.x - grid.x)).toBeLessThan(2);
    }
    const canvas = await page.locator('canvas.arm3d').elementHandle();
    await page.getByRole('button', {name: '调试工作台', exact: true}).click();
    await expect(page.locator('#jointRows')).toBeVisible();
    if (width <= 950)
      await page.getByRole('navigation', {name: '工作区面板'}).getByRole('button', {name: /^电机/}).click();
    await expect(page.locator('.motor-card')).toBeVisible();
    await page.getByRole('button', {name: '仿真工作台', exact: true}).click();
    await expect(page.locator('.objects-area')).toBeHidden();
    await expect(page.locator('#jointRows')).toBeHidden();
    await expect(page.locator('[data-workbench-panel="sim"]')).toBeVisible();
    expect(await canvas!.evaluate(el => el.isConnected)).toBe(true);
    expect(fixture.publishes.filter(p => !['/motor/system_command','/motor/config_request'].includes(p.topic))).toEqual([]);
    expect(fixture.errors).toEqual([]);
  });
}

test("连接与切换模型自动同步只读反馈模型，不产生运动命令", async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/run');
  await expect.poll(() => fixture.feedbackModels).toEqual(['half_body_robot']);
  await page.locator('.arm-model').selectOption('robot_leg');
  await expect.poll(() => fixture.feedbackModels).toEqual(['half_body_robot', 'robot_leg']);
  expect(fixture.publishes.filter(p => !['/motor/system_command','/motor/config_request'].includes(p.topic))).toEqual([]);
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
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/control/joint_target').length).toBe(1);
  await slider.evaluate(e => e.dispatchEvent(new Event('change',{bubbles:true})));
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/control/joint_target').length).toBe(1);
  const sent=fixture.publishes.find(p=>p.topic==='/control/joint_target')!.msg;
  expect(sent.name).toEqual(['joint_link8']);
  expect(sent).not.toHaveProperty('points');
  expect(sent.position).toEqual([0.35]);
  await row.locator('[data-bound="upper"]').click();
  const editor = row.locator('[data-bound-editor="upper"]');
  await editor.fill('0.25'); await editor.press('Enter');
  await expect(slider).toHaveAttribute('max','0.25');
  await expect(row.locator('[data-bound="upper"]')).toHaveText('0.250');
  await expect(row.locator('.joint-target-value')).toHaveText('目标 -- rad');
  await slider.evaluate(e => {e.value='0.3';e.dispatchEvent(new Event('input',{bubbles:true}));});
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/control/joint_target').length).toBe(2);
  expect(fixture.publishes.filter(p=>p.topic==='/control/joint_target').at(-1)!.msg.position).toEqual([0.25]);
  await page.reload();
  await expect(page.locator('#jointRows [data-joint="joint_link8"] input[type="range"]')).toHaveAttribute('max','0.25');
  expect(fixture.errors).toEqual([]);
});

test('手动放宽关节限位须经主机确认，刷新不自动改写主机限位', async ({page}) => {
  const fixture = await connectFixture(page, true);
  fixture.setLimitReply(true, false);
  await page.goto('/workbench/#/run');
  const row = page.locator('#jointRows [data-joint="joint_link8"]');
  const slider = row.locator('input[type="range"]');
  await expect(slider).toBeEnabled();
  expect(fixture.limitRequests).toHaveLength(0);
  await row.locator('[data-bound="lower"]').click();
  const editor = row.locator('[data-bound-editor="lower"]');
  await editor.fill('-1.5'); await editor.press('Enter');
  await expect.poll(() => fixture.limitRequests.length).toBe(1);
  expect(fixture.limitRequests[0]).toMatchObject({model:'half_body_robot',joint_limits:{
    joint_link8:{lower:-1.5,upper:1.047},joint_link9:{lower:-1.22,upper:1.22}}});
  await expect(slider).toHaveAttribute('min','-1.047');
  await expect(slider).toBeDisabled();
  expect(await page.evaluate(()=>localStorage.getItem('web_host.joint_limits.v1.half_body_robot'))).toBeNull();
  await slider.evaluate(e => {e.value='-1.2';e.dispatchEvent(new Event('input',{bubbles:true}));});
  expect(fixture.publishes.filter(p=>p.topic==='/control/joint_target')).toHaveLength(0);
  fixture.sendFeedback('/robot_transmission/status', {data:JSON.stringify({model:'half_body_robot',
    trajectory_enabled:true,joint_names:['joint_link8','joint_link9'],joint_limits:fixture.limitRequests[0].joint_limits,
    control_service:'/workbench_control/set_parameters_atomically'})});
  await expect(slider).toBeEnabled();
  await expect(row.locator('.joint-message')).toContainText('已回读确认');
  await expect.poll(async()=>JSON.parse((await page.evaluate(()=>localStorage.getItem('web_host.joint_limits.v1.half_body_robot')))!))
    .toMatchObject({joint_link8:{lower:-1.5,upper:1.047}});
  await slider.evaluate(e => {e.value='-1.2';e.dispatchEvent(new Event('input',{bubbles:true}));});
  await expect.poll(() => fixture.publishes.filter(p=>p.topic==='/control/joint_target').length).toBe(1);
  const sent = fixture.publishes.find(p=>p.topic==='/control/joint_target')!.msg;
  expect(sent.position).toEqual([-1.2]);
  await page.reload();
  await expect(slider).toHaveAttribute('min','-1.5');
  await expect(slider).toBeEnabled();
  expect(fixture.limitRequests).toHaveLength(1);
  expect(fixture.errors).toEqual([]);
});

test('主机拒绝关节限位后沿用生效范围，失败草稿不存储或自动重试', async ({page}) => {
  const fixture = await connectFixture(page, true);
  fixture.setLimitReply(false);
  await page.goto('/workbench/#/run');
  const row = page.locator('#jointRows [data-joint="joint_link8"]');
  const slider = row.locator('input[type="range"]');
  await expect(slider).toBeEnabled();
  await row.locator('[data-bound="lower"]').click();
  const editor = row.locator('[data-bound-editor="lower"]');
  await editor.fill('-1.5'); await editor.press('Enter');
  await expect.poll(() => fixture.limitRequests.length).toBe(1);
  await expect(row.locator('.joint-message')).toContainText('主机拒绝关节限位');
  await expect(slider).toHaveAttribute('min','-1.047');
  await expect(slider).toBeEnabled();
  expect(await page.evaluate(()=>localStorage.getItem('web_host.joint_limits.v1.half_body_robot'))).toBeNull();
  await page.reload();
  await expect(slider).toHaveAttribute('min','-1.047');
  await expect(slider).toBeEnabled();
  expect(fixture.limitRequests).toHaveLength(1);
  expect(fixture.publishes.filter(p=>p.topic==='/control/joint_target')).toHaveLength(0);
  expect(fixture.errors).toEqual([]);
});

test('旧浏览器限位草稿不覆盖主机范围，未接入关节不能假提交成功',async({page})=>{
  await page.addInitScript(()=>localStorage.setItem('web_host.joint_limits.v1.half_body_robot',
    JSON.stringify({joint_link8:{lower:-9,upper:9}})));
  const fixture=await connectFixture(page,true);
  await page.goto('/workbench/#/run');
  const row=page.locator('[data-joint="joint_link8"]');
  await expect(row.locator('input[type="range"]')).toHaveAttribute('min','-1.047');
  fixture.sendFeedback('/joint_states',{name:['joint_link8','joint_link9'],position:[1.952,.1],velocity:[0,0],effort:[0,0]});
  await expect(row.locator('.jpos')).toHaveText('1.952');
  await expect(row.locator('input[type="range"]')).toHaveValue('1.047');
  await expect(row.locator('.joint-target-value')).toHaveText('目标 -- rad');
  const disconnected=page.locator('[data-joint="joint_link1"]');
  await disconnected.locator('[data-bound="upper"]').click();
  const input=disconnected.locator('[data-bound-editor="upper"]');
  await input.fill('1');await input.press('Enter');
  await expect(disconnected.locator('.joint-message')).toContainText('未接入');
  expect(fixture.limitRequests).toHaveLength(0);
  expect(fixture.publishes.filter(p=>p.topic==='/control/joint_target')).toHaveLength(0);
});

test('限位服务已受理但无真实回读时不能保存或显示生效',async({page})=>{
  const fixture=await connectFixture(page,true);fixture.setLimitReply(true,false);
  await page.goto('/workbench/#/run');
  const row=page.locator('[data-joint="joint_link8"]');
  await row.locator('[data-bound="lower"]').click();
  const input=row.locator('[data-bound-editor="lower"]');
  await input.fill('-1.5');await input.press('Enter');
  await expect(row.locator('.joint-message')).toContainText('回读未确认');
  await expect(row.locator('input[type="range"]')).toHaveAttribute('min','-1.047');
  expect(await page.evaluate(()=>localStorage.getItem('web_host.joint_limits.v1.half_body_robot'))).toBeNull();
  expect(fixture.limitRequests).toHaveLength(1);
});

test('使能因主机反馈或控制器未就绪而禁用时显示原因',async({page})=>{
  const fixture=await connectFixture(page,true);
  await page.goto('/workbench/#/run');
  await expect(page.locator('#jointControlEnabled')).toBeEnabled();
  const status={model:'half_body_robot',joint_names:['joint_link8','joint_link9'],
    mode:'stopped',gravity_enabled:false,effort_gain:1,
    output_topic:'/motor/host_command_array',trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
    feedback_fresh:false,controllers_ready:true};
  fixture.sendFeedback('/control/status',{data:JSON.stringify(status)});
  await expect(page.locator('#jointControlEnabled')).toBeDisabled();
  await expect(page.locator('#jointControlResult')).toContainText('反馈未就绪');
  fixture.sendFeedback('/control/status',{data:JSON.stringify({...status,feedback_fresh:true,controllers_ready:false})});
  await expect(page.locator('#jointControlResult')).toContainText('控制器未就绪');
  expect(fixture.publishes.filter(p=>p.topic==='/motor/policy_array')).toHaveLength(0);
});

test("关键参数面板缺失时不建立 ROS 连接", async ({ page }) => {
  const fixture = await connectFixture(page);
  const html = readFileSync(
    new URL("../../../web/panel_templates.html", import.meta.url),
    "utf8",
  );
  await page.route("**/panel_templates.html", (route) =>
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
    new URL("../../../web/panel_templates.html", import.meta.url),
    "utf8",
  );
  await page.route("**/panel_templates.html", (route) =>
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
  await expect(page.locator("[data-workbench-panel]")).toHaveCount(9);
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

test("全局软停显式停用运动控制器，不重新发布手动目标", async ({ page }) => {
  const fixture = await connectFixture(page);
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("0.150");
  await page
    .getByRole("button", { name: /软停|停止运动|停止动作/ })
    .first()
    .click();
  await expect
    .poll(() =>
      fixture.publishes.some((p) => p.topic === "/control/stop"),
    )
    .toBe(true);
  expect(
    fixture.publishes.find((p) => p.topic === "/control/trajectory")?.msg,
  ).toMatchObject({ count: 0, target_count: 0 });
  expect(fixture.publishes.find(p => p.topic === '/control/stop')?.msg).toEqual({});
  expect(fixture.publishes.filter(p => p.topic === '/control/setpoint')).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test("静态工作台在没有 ROS 连接时显示离线，不生成模拟设备", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.route("**/api/**", (route) =>
    route.fulfill({
      json: new URL(route.request().url()).pathname === '/api/runtime'
        ? {control_mode: 'bench', robot_model: ''}
        : { models: [], actions: [], joints: {}, motor_types: {} },
    }),
  );
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => socket.close());
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("等待");
  await expect(page.locator("#selRows tr[data-id]")).toHaveCount(0);
  await expect(page.locator("#btnSendOnce")).toHaveCount(0);
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
  await page.getByRole('checkbox',{name:'使能控制',exact:true}).click();
  await expect(page.getByRole('checkbox',{name:'使能控制',exact:true})).toHaveJSProperty('indeterminate',true);
  await expect.poll(() => fixture.policyRequests.length).toBe(1);
  const policy=fixture.policyRequests[0];
  expect(policy.source_time_ns).toBeGreaterThan(0);
  expect(policy).not.toHaveProperty('policy_generation');
  expect((policy.targets as {motor_id:number}[]).map(target=>target.motor_id)).toEqual([8,9]);
  fixture.sendControlState(true);
  await expect(slider).toBeDisabled();
  fixture.ackPolicy();
  await expect(slider).toBeEnabled();
  await expect(page.getByRole('checkbox',{name:'使能控制',exact:true})).toBeChecked();
  await slider.evaluate(e => {e.value = '0.3'; e.dispatchEvent(new Event('change', {bubbles:true}));});
  await expect.poll(() => fixture.publishes.filter(p => p.topic === '/control/joint_target').length).toBe(1);
  await page.getByRole('checkbox',{name:'使能控制',exact:true}).click();
  await expect.poll(()=>fixture.publishes.filter(p=>p.topic==='/control/stop').length).toBe(1);
  await expect.poll(()=>fixture.policyRequests.length).toBe(2);
  const disabled=fixture.policyRequests.at(-1)!;
  expect((disabled.targets as {enabled:boolean}[]).every(target=>!target.enabled)).toBe(true);
  fixture.sendControlState(false);
  await expect(page.getByRole('checkbox',{name:'使能控制',exact:true})).not.toBeChecked();
  const stoppedStatus = {model:'half_body_robot',
    joint_names:['joint_link8','joint_link9'],mode:'stopped',reject:'JTC 到位误差超限',
    gravity_enabled:false,effort_gain:1,
    output_topic:'/motor/host_command_array',
    trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory'};
  fixture.sendFeedback('/control/status', {data:JSON.stringify(stoppedStatus)});
  await expect(row.locator('.joint-message')).toHaveText('主机拒绝：JTC 到位误差超限');
  await expect(row.locator('.joint-message')).toHaveClass(/warn/);
  fixture.sendFeedback('/control/status', {data:JSON.stringify({...stoppedStatus,reject:''})});
  await expect(row.locator('.joint-message')).toContainText('已提交 0.300 rad');
  fixture.sendFeedback('/motor/status_array', {statuses: [8,9].map(motor_id =>
    ({motor_id,online:true,lifecycle:6,unified_fault_code:2}))});
  fixture.sendControlState(false, 2);
  await expect(slider).toBeDisabled();
  expect(fixture.errors).toEqual([]);
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
  const sent=()=>fixture.publishes.filter(p=>p.topic==='/control/joint_target');
  await expect.poll(()=>sent().length).toBe(2);
  expect(sent()[1].msg.position).toEqual([.27]);
  await slider.evaluate(e=>e.dispatchEvent(new Event('change',{bubbles:true})));
  fixture.sendFeedback('/robot_transmission/status',{data:JSON.stringify({model:'half_body_robot',
    trajectory_enabled:false,joint_names:['joint_link8','joint_link9'],joint_limits:{joint_link8:{lower:-1.047,upper:1.047},joint_link9:{lower:-1.22,upper:1.22}},last_error:''})});
  await expect(page.getByRole('checkbox',{name:'使能控制',exact:true})).not.toBeChecked();
  await expect(slider).toBeDisabled();
  await slider.evaluate(e=>{e.value='0.3';e.dispatchEvent(new Event('input',{bubbles:true}));});
  expect(sent()).toHaveLength(2);
  expect(fixture.errors).toEqual([]);
});


test('设备工作台只读显示电机参数，不提供主机直连', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await expect(page.locator('[data-workbench-panel="deviceParams"]')).toBeVisible();
  await expect(page.locator('#paramRows > p')).toHaveCount(0);
  await expect(page.getByLabel('设备通信来源')).toHaveCount(0);
  await expect(page.getByRole('button',{name:'连接 CAN',exact:true})).toHaveCount(0);
  await expect(page.getByRole('button',{name:'下发修改',exact:true})).toHaveCount(0);
  expect(fixture.errors).toEqual([]);
});

test('固件频率只读查询，分类切换不产生参数写入', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await page.getByRole('button',{name:'固件参数',exact:true}).click();
  const firmware = page.getByRole('region',{name:'固件系统参数'});
  await expect(firmware).toContainText('已读取控制板固定频率');
  await expect(firmware).toContainText('200 Hz');
  await expect(firmware.locator('select,input')).toHaveCount(0);
  await expect(firmware.getByRole('button',{name:'读取固件参数'})).toBeEnabled();
  await expect(firmware.getByRole('button',{name:/下发|保存/})).toHaveCount(0);
  expect(fixture.publishes.filter(p=>p.topic==='/motor/param_set' || p.topic==='/motor/config_request')).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test('真实缺失参数不显示已核对，手动读取使用只读回读并显示未绑定原因', async ({page}) => {
  const fixture = await connectFixture(page);
  fixture.parameterReplies.enabled = true;
  await page.goto('/workbench/#/teach');
  await expect.poll(() => fixture.connections.count).toBe(1);
  fixture.sendFeedback('/motor/topology', {entries: [{motor_id: 1,
    online: true, model_name: '', bus_index: 0, lifecycle: 2}]});
  fixture.sendFeedback('/motor/status_array', {statuses: [{motor_id: 1, online: true, lifecycle: 2}]});
  fixture.sendFeedback('/motor/device_param_report', {count: 1, entries: [{motor_id: 1,
    dev_init_state: 0, rec_pos_min: null, rec_pos_max: null, rec_vel_limit: null,
    rec_torque_limit: null, rec_safety: null, rec_direction: null, rec_t_max: null, rec_node_id: null,
    dev_pos_min: null, dev_pos_max: null, dev_vel_limit: null, dev_torque_limit: null,
    dev_safety: null, dev_direction: null, dev_t_max: null, dev_node_id: null}]});
  await expect(page.locator('#paramSummary')).toHaveCount(0);
  await expect(page.locator('.motor-parameter-body')).toContainText('型号尚未识别');
  await expect(page.locator('.motor-parameter-body')).toContainText('未返回有效参数');
  await expect(page.getByLabel('位置上限（rad）', {exact: true})).toHaveAttribute('readonly', '');
  await page.getByRole('button', {name: '读取电机参数', exact: true}).click();
  const reads = () => fixture.publishes.filter(p => p.topic === '/motor/system_command' && p.msg.command === 1);
  await expect.poll(() => reads().length).toBe(1);
  await expect(page.locator('#paramResult')).toContainText('尚未识别型号或未绑定');
  await expect(page.locator('#paramResult')).toHaveClass(/err/);
  expect(reads()[0]!.msg.motor_id).toBe(1);
  expect(reads()[0]!.msg.sync_time_ns).toBe(9000000000);
  expect(fixture.publishes.filter(p => p.topic === '/motor/system_command' && ![1,4].includes(Number(p.msg.command)))).toEqual([]);
  expect(fixture.publishes.filter(p => ['/motor/param_set', '/motor/policy_array', '/control/trajectory', '/control/setpoint', '/control/stop', '/control/joint_target'].includes(p.topic))).toEqual([]);
  fixture.sendFeedback('/motor/topology', {entries: [{motor_id: 1,
    online: true, model_name: 'cerb_r48_101', bus_index: 0, lifecycle: 3}]});
  fixture.sendFeedback('/motor/device_param_report', {count: 1, entries: [{motor_id: 1,
    dev_init_state: 3, mismatch_mask: 3, rec_pos_min: -Math.PI, rec_pos_max: 3,
    rec_vel_limit: 3.44, rec_torque_limit: 5, rec_safety: 0, rec_direction: 0, rec_t_max: 300, rec_node_id: 1,
    dev_pos_min: -Math.PI, dev_pos_max: Math.PI, dev_vel_limit: 3.44,
    dev_torque_limit: 5, dev_safety: 0, dev_direction: 0, dev_t_max: 300, dev_node_id: 1}]});
  await expect(page.locator('#paramSummary')).toHaveCount(0);
  await expect(page.getByLabel('位置上限（rad）', {exact: true})).toHaveAttribute('title', /记录 3.*驱动板 3.141593/);
  await expect(page.getByLabel('位置上限（rad）', {exact: true})).toHaveValue('3.141593');
  expect(fixture.errors).toEqual([]);
});

test('旧状态夹具清错请求使用同步MCU时间，失败回执不显示成功',async({page})=>{
  const fixture=await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await expect.poll(()=>fixture.connections.count).toBe(1);
  fixture.sendFeedback('/motor/device_param_report',{count:1,entries:[{motor_id:1,dev_init_state:0}]});
  const clear=page.locator('.motor-parameter-item[data-motor="1"] [data-clearfault="1"]');
  await expect(clear).toBeVisible();
  page.once('dialog',dialog=>dialog.accept());await clear.click();
  const requests=()=>fixture.publishes.filter(p=>p.topic==='/motor/system_command' && p.msg.command===3);
  await expect.poll(()=>requests().length).toBe(1);
  expect(requests()[0].msg).toMatchObject({motor_id:1,sync_time_ns:9000000000});
  fixture.sendFeedback('/motor/config_result',{session_id:9,request_id:requests()[0].msg.request_id,
    kind:1,command:3,status:2,error_code:10,completed_targets:0,failed_targets:1});
  await expect(page.locator('#paramResult')).toContainText('设备拒绝');
  await expect(page.locator('#paramResult')).not.toContainText('已受理');
  expect(requests()).toHaveLength(1);expect(fixture.errors).toEqual([]);
});

test('电机参数只读，ID与T_max没有设置和保存入口', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await expect.poll(()=>fixture.connections.count).toBe(1);
  fixture.sendFeedback('/motor/device_param_report',{count:1,entries:[{motor_id:1,
    rec_node_id:1,rec_t_max:300,rec_pos_min:-1,rec_pos_max:1,rec_vel_limit:2,rec_torque_limit:3,rec_safety:0,rec_direction:0}]});
  const panel=page.locator('[data-workbench-panel="deviceParams"]');
  await expect(panel.getByLabel('电机 ID',{exact:true})).toHaveAttribute('readonly','');
  await expect(panel.getByLabel('T_max（N·m）',{exact:true})).toHaveAttribute('readonly','');
  await expect(panel.locator('[data-dir],[data-setlim]')).toHaveCount(0);
  await expect(panel.getByRole('button',{name:'设零',exact:true})).toBeDisabled();
  await expect(panel.getByRole('button',{name:/下发|保存/})).toHaveCount(0);
  expect(fixture.publishes.filter(p=>p.topic==='/motor/param_set')).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test('所选电机失能后可设零，取消不发送，设备终态才显示成功', async ({page}) => {
  const fixture=await connectFixture(page,true,true);
  await page.goto('/workbench/#/teach');
  await expect.poll(()=>fixture.connections.count).toBe(1);
  fixture.sendFeedback('/motor/device_param_report',{count:2,entries:[8,9].map(motor_id=>({motor_id,
    rec_node_id:motor_id,rec_t_max:300,rec_pos_min:-1,rec_pos_max:1,rec_vel_limit:2,
    rec_torque_limit:3,rec_safety:1,rec_direction:0}))});
  fixture.sendFeedback('/control/status',{data:JSON.stringify({model:'half_body_robot',
    joint_names:['joint_link8','joint_link9'],mode:'stopped',active_controllers:[],pending_goal_ids:[],
    gravity_enabled:false,effort_gain:1,output_topic:'/motor/host_command_array',
    trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory'})});
  await page.getByRole('group',{name:'电机列表',exact:true}).getByRole('button',{name:/电机 8/}).click();
  const zero=page.locator('.motor-parameter-item[data-motor="8"]').getByRole('button',{name:'设零',exact:true});
  await expect(zero).toBeEnabled();
  const requests=()=>fixture.publishes.filter(p=>p.topic==='/motor/system_command' && p.msg.command===2);
  expect(requests()).toHaveLength(0);
  fixture.sendControlState(true);await expect(zero).toBeDisabled();
  fixture.sendControlState(false);await expect(zero).toBeEnabled();
  page.once('dialog',dialog=>dialog.dismiss());await zero.click();
  expect(requests()).toHaveLength(0);
  page.once('dialog',dialog=>dialog.accept());await zero.click();
  await expect.poll(()=>requests().length).toBe(1);
  expect(requests()[0].msg.motor_id).toBe(8);
  expect(Number(requests()[0].msg.sync_time_ns)).toBeGreaterThan(0);
  await expect(zero).toBeDisabled();
  const request_id=requests()[0].msg.request_id;
  const reply={session_id:7001,request_id,kind:1,command:2,error_code:0,completed_targets:1,failed_targets:0};
  fixture.sendFeedback('/motor/config_result',{...reply,status:0});
  await expect(page.locator('#paramResult')).toContainText('等待设备确认');
  fixture.sendFeedback('/motor/config_result',{...reply,status:1});
  await expect(page.locator('#paramResult')).toContainText('电机 8 设零成功（设备已确认）');
  await expect(zero).toBeEnabled();
  expect(fixture.publishes.filter(p=>['/motor/param_set','/motor/policy_array','/control/setpoint',
    '/control/trajectory','/control/stop','/control/joint_target'].includes(p.topic))).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test('左侧电机参数展开子列表，选择切换只读详情', async ({page}) => {
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await expect.poll(() => fixture.connections.count).toBe(1);
  fixture.sendFeedback('/motor/topology', {entries: [1, 2].map(motor_id => ({motor_id,
    online: true, model_name: 'cerb_r48_101', bus_index: 0, lifecycle: 3}))});
  const report = {count: 2, entries: [1, 2].map(motor_id => ({motor_id,
    rec_pos_min: -1, rec_pos_max: 1, rec_vel_limit: 2, rec_torque_limit: 3,
    rec_safety: 0, rec_direction: 0}))};
  fixture.sendFeedback('/motor/device_param_report', report);
  const first = page.locator('.motor-parameter-item[data-motor="1"]');
  const second = page.locator('.motor-parameter-item[data-motor="2"]');
  const list = page.getByRole('group', {name: '电机列表', exact: true});
  const toggle = page.getByRole('button', {name: '电机参数', exact: true});
  await expect(list.getByRole('button')).toHaveCount(2);
  await expect(first).toBeVisible();
  await expect(second).toBeHidden();
  await toggle.click();
  await expect(list).toBeHidden();
  await toggle.click();
  await list.getByRole('button', {name: /电机 2/}).click();
  fixture.sendFeedback('/motor/device_param_report', report);
  await expect(first).toBeHidden();
  await expect(second).toBeVisible();
  await expect(page.locator('.device-details-area > .panel-title')).toContainText('电机 2 详细参数');
  await expect(page.locator('#paramSelectionHint')).toBeHidden();
  await expect(second.getByRole('heading', {name: '电机参数', exact: true})).toBeVisible();
  await expect(second.locator('[data-dir],[data-setlim]')).toHaveCount(0);
  await expect(second.getByRole('button',{name:'设零',exact:true})).toBeDisabled();
  const input = second.getByLabel('位置下限（rad）', {exact: true});
  fixture.sendFeedback('/motor/device_param_report', {count: 2, entries: report.entries.map(entry => (
    entry.motor_id === 2 ? {...entry, rec_pos_min: null} : entry))});
  await expect(input).toHaveValue('');
  await expect(input).toHaveAttribute('placeholder', '未读取');
  await expect(second.locator('dd').filter({has: input}).locator('.rawval')).toHaveCount(0);
  fixture.sendFeedback('/motor/device_param_report', report);
  await expect(input).toHaveValue('-1');
  await expect(input).toHaveAttribute('readonly','');
  await list.getByRole('button', {name: /电机 1/}).click();
  await expect(first).toBeVisible();
  await expect(second).toBeHidden();
  await list.getByRole('button', {name: /电机 2/}).click();
  await expect(second).toBeVisible();
  fixture.sendFeedback('/motor/device_param_report', report);
  fixture.sendFeedback('/motor/motion_states', {states: [
    {motor_id: 2, position: 0.42, velocity: 0, effort: 0, state_age_ms: 5}]});
  await expect(second.locator('.curpos')).toHaveText('0.4200');
  await expect(input).toHaveValue('-1');
  await expect(first).toBeHidden();
  fixture.sendFeedback('/motor/topology', {entries: [1, 2, 3].map(motor_id => ({motor_id,
    online: true, model_name: 'cerb_r48_101', bus_index: 0, lifecycle: 3}))});
  await list.getByRole('button', {name: /电机 3/}).click();
  await expect(first).toBeHidden();
  await expect(second).toBeHidden();
  await expect(page.locator('#paramSelectionHint')).toContainText('电机 3 暂无参数');
  await expect(page.locator('#paramSelectionHint')).toBeVisible();
  await list.getByRole('button', {name: /电机 2/}).click();
  await expect(input).toHaveValue('-1');
  expect(await second.evaluate(e => e.scrollWidth <= e.clientWidth)).toBe(true);
  fixture.sendFeedback('/motor/topology', {entries: Array.from({length: 20}, (_, i) => ({motor_id: i + 1,
    online: true, model_name: 'cerb_r48_101', bus_index: 0, lifecycle: 3}))});
  await expect(list.getByRole('button')).toHaveCount(20);
  await list.getByRole('button', {name: /电机 20/}).click();
  await expect(page.locator('#paramSelectionHint')).toContainText('电机 20 暂无参数');
  await expect(page.locator('.tree-footer')).toBeInViewport();
  expect(fixture.publishes.filter(p => ['/motor/param_set', '/motor/policy_array', '/control/trajectory', '/control/setpoint', '/control/stop', '/control/joint_target'].includes(p.topic))).toEqual([]);
  expect(fixture.errors).toEqual([]);
});

test('缺少固件固定配置报告时不伪造默认回读值', async ({page}) => {
  const fixture=await connectFixture(page);
  delete fixture.firmware.values['system.agent_ping_hz'];
  await page.goto('/workbench/#/teach');
  await page.getByRole('button',{name:'固件参数',exact:true}).click();
  const firmware=page.getByRole('region',{name:'固件系统参数'});
  await expect(firmware).toContainText('未收到新的固件参数');
  await expect(firmware).not.toContainText('200 Hz');
  await expect(firmware.locator('select,input')).toHaveCount(0);
  expect(fixture.errors).toEqual([]);
});

test('控制板会话更换后清除旧只读配置，重新回读', async ({page}) => {
  const fixture=await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await page.getByRole('button',{name:'固件参数',exact:true}).click();
  const firmware=page.getByRole('region',{name:'固件系统参数'});
  await expect(firmware).toContainText('已读取控制板固定频率');
  fixture.sendFeedback('/motor/control_states',{schema_version:2,session_id:90001,sequence:1,states:[]});
  await expect(firmware).not.toContainText('200 Hz');
  await firmware.getByRole('button',{name:'读取固件参数'}).click();
  await expect(firmware).toContainText('200 Hz');
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

test('旧状态夹具动作库下发使用同步MCU时间，取消不伪造完成',async({page})=>{
  const fixture=await connectFixture(page);
  await page.route('**/api/actions**',route=>new URL(route.request().url()).pathname==='/api/actions'
    ? route.fulfill({json:{actions:[{name:'source-time.json',target_ids:[1],target_count:1,point_count:2,duration_ms:100}]}})
    : route.fulfill({json:{target_ids:[1],points:[
      {motor_id:1,time_from_start_ms:0,position:.15,velocity:0},
      {motor_id:1,time_from_start_ms:100,position:.16,velocity:0}]}}));
  await page.goto('/workbench/#/run');
  await page.locator('#actEnable').uncheck();
  await page.locator('#actList').getByRole('button',{name:'播放',exact:true}).click();
  const requests=()=>fixture.publishes.filter(p=>p.topic==='/control/trajectory' && Number(p.msg.count)>0);
  await expect.poll(()=>requests().length).toBe(1);
  expect(requests()[0].msg.sync_time_ns).toBe(9000000000);
  await page.locator('#btnActStop').click();
  await expect(page.locator('#actResult')).not.toContainText('播放完成');
  expect(fixture.publishes.filter(p=>p.topic==='/motor/policy_array')).toHaveLength(0);
  expect(fixture.policyRequests).toHaveLength(0);expect(fixture.errors).toEqual([]);
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
    if (viewport.width > 950) {
      const stage=(await page.locator('.stage-area').boundingBox())!;
      expect(Math.abs(stage.x+stage.width-grid.x-grid.width)).toBeLessThan(2);
    }
    if (viewport.width <= 650)
      await page.locator('.workspace-grid').evaluate(el => {el.scrollTop = el.scrollHeight;});
    if (viewport.width <= 950)
      await page.getByRole('button', {name: '观察', exact: true}).click();
    await expect(page.locator('.observer-head')).toBeInViewport({ratio: 1});
    await expect(page.locator('.app-footer')).toHaveCount(0);
    await page.locator('.workspace-heading').hover();
    await page.mouse.wheel(0, 400);
    expect(await main.evaluate(el => el.scrollTop)).toBe(0);
    expect(await page.evaluate(() => window.scrollY)).toBe(0);
  });
}

test('设备工作台在矮窗口中保持详情位于内部滚动区', async ({page}) => {
  await page.setViewportSize({width:1063, height:650});
  const fixture = await connectFixture(page);
  await page.goto('/workbench/#/teach');
  await expect(page.locator('.startup-status')).toHaveCount(0);
  await expect.poll(() => fixture.connections.count).toBe(1);
  fixture.sendFeedback('/motor/device_param_report', {count: 1, entries: [{motor_id: 1,
    rec_pos_min: -1, rec_pos_max: 1, rec_vel_limit: 2, rec_torque_limit: 3,
    rec_safety: 0, rec_direction: 0}]});
  await expect(page.locator('.motor-parameter-item')).toHaveCount(1);
  const main = (await page.locator('.main-workspace').boundingBox())!;
  const inspector = (await page.locator('.device-details-area').boundingBox())!;
  expect(inspector.y + inspector.height).toBeLessThanOrEqual(main.y + main.height);
  const pane = page.locator('.inspector-scroll');
  expect(await pane.evaluate(el => getComputedStyle(el).overflowY)).toBe('auto');
  await pane.evaluate(el => {el.scrollTop = el.scrollHeight;});
  await expect(page.locator('.motor-saved-record')).toHaveCount(0);
  await expect(page.getByLabel('设备通信来源')).toHaveCount(0);
  await expect(page.getByRole('navigation', {name:'设备参数查询'})).toBeVisible();
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
  for (const selector of ['.stage-body', '#actList']) {
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
