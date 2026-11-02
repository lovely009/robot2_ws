import {test, expect} from '@playwright/test';

test('工作台模型可选，普通策略直接发布，无托管流程', async ({page}) => {
  const publishes:any[]=[];
  await page.route(/\.stl(?:\?|$)/i, route=>route.fulfill({status:404,body:''}));
  await page.route('**/api/**',route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/runtime') return route.fulfill({json:{control_mode:'bench'}});
    if(path==='/api/models') return route.fulfill({json:{models:['half_body_robot','robot_leg']}});
    if(path==='/api/model-transmissions') return route.fulfill({json:{groups:[],direct_joints:{}}});
    return route.fulfill({json:{actions:[],motor_types:{}}});
  });
  await page.routeWebSocket('ws://127.0.0.1:9090',socket=>socket.onMessage(raw=>{
    const m=JSON.parse(String(raw));if(m.op==='publish')publishes.push(m);
    if(m.op==='call_service')socket.send(JSON.stringify({op:'service_response',id:m.id,service:m.service,result:true,values:{ok:true,status_json:'{}'}}));
  }));
  await page.goto('/workbench/');
  await expect(page.getByLabel('整机托管控制')).toHaveCount(0);
  await expect(page.locator('#armModel')).toBeEnabled();
  await page.locator('#armModel').selectOption('robot_leg');
  await expect(page.locator('#armModel')).toHaveValue('robot_leg');
  await expect(page.locator('#polRows, #cmdRows, .object-heading')).toHaveCount(0);
  await expect(page.getByRole('button',{name:'检查器',exact:true})).toHaveCount(0);
  await expect(page.locator('.device-details-area')).toBeHidden();
  expect(publishes.filter(m=>m.topic==='/motor/policy_array')).toEqual([]);
  expect(publishes.every(m=>!m.topic.startsWith('/robot/'))).toBe(true);
});
