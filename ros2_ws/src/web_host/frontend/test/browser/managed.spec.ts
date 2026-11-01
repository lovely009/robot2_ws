import { test, expect } from '@playwright/test';

for (const cancel of [false, true]) test(`托管页面显式控制与标准 Action ${cancel ? '取消保留使能' : '成功'}，隐藏后释放旧会话`, async ({ page }) => {
  const requests: any[] = [], published: any[] = [];
  const state = {epoch: 17, session_id: 1_800_000_000_000, ready: true,
    armed: false, mcu_authorized: false, fault_latched: false, owner: '', blockers: []};
  let publishState = () => {};
  let finishCanceled = () => {};
  // Geometry/rendering is covered by integration-preview. Keep control-state
  // timing independent of the 70MB CAD mesh decoding in this safety fixture.
  await page.route(/\.stl(?:\?|$)/i, route => route.fulfill({status:404, body:''}));
  await page.route('**/api/**', route => {
    const url = new URL(route.request().url());
    if (url.pathname === '/api/runtime') return route.fulfill({json: {
      control_mode: 'managed', robot_model: 'half_body_robot', robot_id: 'fixture-browser'}});
    if (url.pathname === '/api/models') return route.fulfill({json: {models: ['half_body_robot']}});
    if (url.pathname === '/api/model-transmissions') return route.fulfill({json: {
      model: 'half_body_robot', groups: [{name: 'direct', motors: [{name: 'm1', id: 1}],
        joints: ['joint_link1'], direct_mapping: true}], direct_joints: {1: 'joint_link1'}}});
    if (url.pathname === '/api/model-joints') return route.fulfill({json: {joints: {1: 'joint_link1'}}});
    if (url.pathname === '/api/motor-specs') return route.fulfill({json: {motor_types: {}}});
    return route.fulfill({json: {actions: []}});
  });
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => {
    const subscribed = new Set<string>();
    publishState = () => {
      if (subscribed.has('/robot/state')) socket.send(JSON.stringify({op: 'publish', topic: '/robot/state', msg: state}));
    };
    const timer = setInterval(publishState, 100);
    page.once('close', () => clearInterval(timer));
    socket.onMessage(raw => {
      const message = JSON.parse(String(raw));
      if (message.op === 'subscribe') { subscribed.add(message.topic); publishState(); }
      if (message.op === 'publish') published.push(message);
      if (message.op !== 'call_service') return;
      requests.push(message);
      let values: any = {success: true};
      if (message.service === '/robot/acquire_control') {
        state.owner = message.args.owner;
        values = {...values, lease_id: 'browser-private-token', epoch: state.epoch,
          session_id: state.session_id, lease_ms: 1000};
      } else if (message.service === '/robot/set_armed') state.armed = message.args.armed;
      else if (message.service === '/robot/renew_control') values.lease_remaining_ms = 1000;
      else if (message.service.endsWith('/send_goal')) values = {accepted: true, stamp: {sec: 1, nanosec: 0}};
      else if (message.service.endsWith('/get_result')) {
        if (cancel) {
          finishCanceled = () => socket.send(JSON.stringify({op:'service_response',id:message.id,service:message.service,values:{status:5,result:{error_code:0}},result:true}));
          return;
        }
        values = {status:4,result:{error_code:0}};
      } else if (message.service.endsWith('/cancel_goal')) { values={return_code:0,goals_canceling:[]};finishCanceled(); }
      socket.send(JSON.stringify({op: 'service_response', id: message.id,
        service: message.service, values, result: true}));
      publishState();
    });
  });
  await page.goto('/workbench/');
  const controls = page.getByLabel('整机托管控制');
  await expect(controls).toBeVisible();
  await expect(controls.getByRole('button', {name:'申请控制',exact:true})).toBeEnabled();
  await controls.getByRole('button', {name:'申请控制',exact:true}).evaluate((button:HTMLButtonElement) => {button.click();button.click();});
  await expect.poll(() => requests.filter(r => r.service === '/robot/acquire_control').length).toBe(1);
  await expect(controls.getByRole('button', {name: '武装', exact: true})).toBeEnabled();
  await controls.getByRole('button', {name: '武装', exact: true}).click();
  await expect(controls).toContainText('等待 MCU 授权确认');
  const attempt = () => page.evaluate(async () => {
    const ros = await import('/js/core/ros.js');
    return ros.pub('policy', {count: 1, targets: [{motor_id: 1, enabled: true, kp: 1, kd: .1}]});
  });
  expect(await attempt()).toBe(false);
  expect(published).toHaveLength(0);
  state.mcu_authorized = true; publishState();
  await expect(controls.getByText('等待 MCU 授权确认', {exact: true})).toHaveCount(0);
  expect(await attempt()).toBe(true);
  await expect.poll(() => published.length).toBe(1);
  expect(published[0]).toMatchObject({topic: '/robot/control/policy', msg: {
    lease_id: 'browser-private-token', epoch: 17, session_id: state.session_id, sequence: 1}});
  await page.evaluate(async () => {
    const ros = await import('/js/core/ros.js');
    ros.pub('jointTrajectory', {joint_names: ['joint_link1'], points: [
      {positions: [.1], time_from_start: {sec: 0, nanosec: 0}},
      {positions: [.2], time_from_start: {sec: 1, nanosec: 0}}]});
  });
  if (cancel) {
    await expect(controls).toContainText('轨迹已受理');
    await page.evaluate(async () => {const ros=await import('/js/core/ros.js');ros.pub('trajectory',{count:0,points:[]});ros.pub('trajectory',{count:0,points:[]});});
    await expect(controls).toContainText('轨迹已取消');
    expect(requests.filter(r=>r.service.endsWith('/cancel_goal'))).toHaveLength(1);
    expect(requests.filter(r=>r.service==='/robot/stop')).toHaveLength(0);
    expect(state.armed).toBe(true);
  } else await expect(controls).toContainText('轨迹执行成功');
  const bind = requests.find(r => r.service === '/robot/authorize_motion');
  const goal = requests.find(r => r.service.endsWith('/send_goal'));
  expect(goal.args.goal_id.uuid).toEqual(bind.args.goal_id);
  expect(bind.args.sequence).toBe(2);
  expect(published.every(item => item.topic.startsWith('/robot/control/'))).toBe(true);
  await page.evaluate(() => {
    Object.defineProperty(document, 'hidden', {configurable: true, value: true});
    document.dispatchEvent(new Event('visibilitychange'));
  });
  await expect.poll(() => requests.filter(r => r.service === '/robot/release_control').length).toBe(1);
  await expect(controls.getByRole('button', {name: '武装', exact: true})).toBeDisabled();
  const renewals = requests.filter(r => r.service === '/robot/renew_control').length;
  await page.waitForTimeout(400);
  expect(requests.filter(r => r.service === '/robot/renew_control')).toHaveLength(renewals);
});
