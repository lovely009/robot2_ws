import { test, expect } from '@playwright/test';

// Offline review only. IDs and bus assignments are deliberately fictional.
test('整合预览仅消费模拟状态，导航与重连不产生运动授权', async ({page}) => {
  const publishes: unknown[] = [], services: string[] = [], errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.route('**/api/**', route => {
    const path = new URL(route.request().url()).pathname;
    if (path === '/api/runtime') return route.fulfill({json: {control_mode: 'managed', robot_model: 'half_body_robot', robot_id: 'offline-review'}});
    if (path === '/api/models') return route.fulfill({json: {models: ['half_body_robot']}});
    if (path === '/api/model-transmissions') return route.fulfill({json: {model: 'half_body_robot', groups: [], direct_joints: {}}});
    if (path === '/api/model-joints') return route.fulfill({json: {joints: {}}});
    if (path === '/api/device-can/options') return route.fulfill({status:410, json:{error:'统一经 MCU；预览不访问设备'}});
    return route.fulfill({json: {actions: [], motor_types: {}}});
  });
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => {
    const state = {epoch:17, session_id:1_800_000_000_000, ready:false, armed:false,
      mcu_authorized:false, fault_latched:false, owner:'', blockers:['integration_pending', 'calibration_uncommissioned']};
    const fixtures: Record<string, unknown> = {
      '/robot/state': state,
      '/motor/topology': {entries: Array.from({length:37}, (_, i) => ({motor_id:i+1, online:true, model_name:'模拟 CERB · 非实测', bus_index:i%3, lifecycle:3}))},
      '/motor/status_array': {statuses: Array.from({length:37}, (_, i) => ({motor_id:i+1, online:true, lifecycle:3, temperature:25, unified_fault_code:0}))},
      '/motor/motion_states': {states: Array.from({length:37}, (_, i) => ({motor_id:i+1, position:i*.01, velocity:0, effort:0, state_age_ms:5}))},
    };
    const subscribed = new Set<string>();
    const send = () => { for (const topic of subscribed) if (fixtures[topic]) socket.send(JSON.stringify({op:'publish', topic, msg:fixtures[topic]})); };
    const timer = setInterval(send, 100);
    page.once('close', () => clearInterval(timer));
    socket.onMessage(raw => {
      const message = JSON.parse(String(raw));
      if (message.op === 'subscribe') { subscribed.add(message.topic); send(); }
      if (message.op === 'publish') publishes.push(message);
      if (message.op === 'call_service') services.push(message.service);
    });
  });
  await page.goto('/workbench/#/run');
  await expect(page.locator('.startup-status')).toHaveCount(0);
  await expect(page.locator('.motor-card')).toHaveCount(37);
  await expect(page.getByRole('button', {name:'申请控制', exact:true})).toBeDisabled();
  await expect(page.getByRole('button', {name:'武装', exact:true})).toBeDisabled();
  await page.evaluate(() => {
    const badge = document.createElement('div');
    badge.textContent = '模拟数据 · 37轴/总线分配为虚构 · 非实测 · 整合未验收';
    badge.style.cssText = 'position:fixed;top:10px;right:12px;z-index:999;background:#443417;color:#ffe1a2;border:1px solid #b8954d;padding:6px 12px;font:12px sans-serif;max-width:75vw;border-radius:5px';
    document.body.append(badge);
  });
  await page.screenshot({path:'/workspace/robot2-ui-evidence/integration-before-desktop.png'});
  for (const scene of ['设备工作台','仿真工作台','动作工作台','调试工作台'])
    await page.getByRole('button', {name:scene, exact:true}).click();
  await page.getByRole('button', {name:'重新连接', exact:true}).click();
  await expect(page.getByRole('button', {name:'申请控制', exact:true})).toBeDisabled();
  await page.setViewportSize({width:390,height:844});
  await page.screenshot({path:'/workspace/robot2-ui-evidence/integration-before-mobile.png'});
  expect(publishes).toEqual([]);
  expect(services).toEqual([]);
  expect(errors).toEqual([]);
});
