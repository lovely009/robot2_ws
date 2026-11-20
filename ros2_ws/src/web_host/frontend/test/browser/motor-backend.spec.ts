import { expect, test } from '@playwright/test';

test('多路电机后端提交双通道配置并按通道添加节点', async ({ page }) => {
  const requests: {service: string; args: any}[] = [];
  const errors: string[] = [];
  let failedSecondChannel = false;
  page.on('pageerror', e => errors.push(e.message));
  await page.route('**/api/**', route => route.fulfill({json:
    new URL(route.request().url()).pathname === '/api/runtime'
      ? {control_mode:'bench',robot_model:'half_body_robot'} : {groups:[],models:['half_body_robot']}}));
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => {
    socket.onMessage(raw => {
      const msg = JSON.parse(String(raw));
      if (msg.op !== 'call_service') return;
      requests.push({service:msg.service,args:msg.args});
      const values: any = {ok:true,error:'',status_json:'{}',text:''};
      if (['/bench/options','/bench/get_options'].includes(msg.service)) Object.assign(values, {
        can_interfaces_json:'[]',serial_ports_json:'[]',simulation_models:['half_body_robot'],
        mujoco_models:['half_body_robot'],usbcanfd_devices_json:JSON.stringify([
          {index:0,label:'创芯双通道',accessible:true}])});
      if (msg.service === '/bench/start_motor_backend') values.status_json = JSON.stringify({
        motor_backend:{state:'running',config:{transport:'multi',model:'half_body_robot',
          channels:JSON.parse(msg.args.sim_channels_json)}}});
      if (msg.service === '/bench/motor_backend_command') {
        if (JSON.parse(msg.args.request_json).cmd === 'add') failedSecondChannel = true;
        values.response_json = JSON.stringify({
          ok:true,motors:[{id:1,kind:'cerb',model:'r48',linked:true,enabled:false,bus:0}],
          channels:[{bus:0},{bus:1,...(failedSecondChannel
            ? {state:'failed',error:'USB disconnected'} : {})}]});
      }
      socket.send(JSON.stringify({op:'service_response',id:msg.id,service:msg.service,result:true,values}));
    });
  });
  await page.goto('/workbench/#/sim');
  await expect(page.locator('#simModel')).toHaveValue('half_body_robot');
  await page.locator('#motorBackendTransport').selectOption('multi');
  await expect(page.locator('#simTransport')).toHaveValue('multi');
  const rows = page.locator('[data-sim-route]');
  await expect(rows).toHaveCount(4);
  await expect(rows.nth(3)).toHaveAccessibleName('MCU CAN4');
  await rows.nth(1).locator('[data-route-enabled]').check();
  await rows.nth(1).locator('[data-route-ids]').fill('10-18');
  await page.locator('#simMultiFields').screenshot({path:test.info().outputPath('motor-routes.png')});
  await page.locator('#simBusIsolated').check();
  await page.locator('#btnMotorBackendStart').click();
  await expect.poll(() => requests.filter(r => r.service === '/bench/start_motor_backend').length).toBe(1);
  const request = requests.find(r => r.service === '/bench/start_motor_backend')!.args;
  expect(JSON.parse(request.sim_channels_json)).toEqual([
    {bus:0,transport:'usbcanfd',device:0,channel:0,ids:'*'},
    {bus:1,transport:'usbcanfd',device:0,channel:1,ids:'10-18'}]);
  await expect(page.locator('#simMotorList')).toContainText('CAN1');
  await page.locator('#simMotorAddId').fill('37');
  await page.locator('#simMotorAddBus').selectOption('1');
  await page.locator('#btnSimMotorAdd').click();
  await expect.poll(() => requests.filter(r => r.service === '/bench/motor_backend_command'
    && JSON.parse(r.args.request_json).cmd === 'add').length).toBe(1);
  const add = requests.find(r => r.service === '/bench/motor_backend_command'
    && JSON.parse(r.args.request_json).cmd === 'add')!;
  expect(JSON.parse(add.args.request_json).bus).toBe(1);
  await expect(page.locator('#simMotorHint')).toContainText('CAN2 故障：USB disconnected');
  await expect(page.locator('#simMotorAddBus option[value="1"]')).toHaveJSProperty('disabled', true);
  await page.screenshot({path:test.info().outputPath("motor-backend.png"),fullPage:true});
  expect(errors).toEqual([]);
});
