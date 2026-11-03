import {test, expect} from '@playwright/test';

const mapping = {mode:'generic', source_root:'Hips', reference_frame:0,
  source_to_robot:[[1,0,0],[0,1,0],[0,0,1]], limit_policy:'urdf',
  targets:[{target:'unknown_link', source:'', confidence:'unknown', reason:'无法从名称确定人体部位'}]};
const job = {id:'generic-fixture', state:'mapping', progress:0, bvh_name:'sample.bvh',
  urdf_name:'unseen.urdf', robot_name:'unseen', frames:3, frame_time:.02, joint_count:1,
  source_joints:['Hips','LeftArm'], robot_links:['base','unknown_link'], mapping,
  mapping_origin:'suggested', warnings:['axis：缺少有效位置/速度限位']};
const preview = {times:[0,.02,.04], duration:.04, source_to_robot:[[1,0,0],[0,1,0],[0,0,1]],
  source:{names:['Hips','LeftArm'],parents:[-1,0],frames:[[[0,0,0],[1,0,0]],[[0,0,0],[1,0,0]],[[0,0,0],[1,0,0]]]},
  robot:{names:['base','unknown_link'],parents:[-1,0],frames:[[[0,0,0],[1,0,0]],[[0,0,0],[1,0,0]],[[0,0,0],[1,0,0]]]}};

test.beforeEach(async ({page}) => {
  await page.routeWebSocket('**', socket => socket.close());
  await page.route('**/api/runtime', route => route.fulfill({json:{control_mode:'readonly',robot_model:'unseen'}}));
  await page.route('**/api/models', route => route.fulfill({json:{models:['unseen']}}));
  await page.route('**/api/motion/sources', route => route.fulfill({json:{sources:['sample.bvh']}}));
  await page.route('**/robot_sim/models/unseen/urdf/unseen.urdf', route => route.fulfill({body:'<robot name="unseen"/>'}));
});

test('通用候选需要确认，明确选择几何预览并记住映射', async ({page}) => {
  await page.setViewportSize({width:1280,height:720});
  let confirmed = false;
  await page.route('**/api/motion/jobs', route => {
    expect(route.request().postDataJSON().retargeting).toBe('generic');
    return route.fulfill({json:job});
  });
  await page.route('**/api/motion/jobs/generic-fixture/convert', route => {
    const body = route.request().postDataJSON();
    expect(body.remember_mapping).toBe(true);
    expect(body.mapping.targets[0]).toMatchObject({target:'unknown_link',source:'LeftArm'});
    expect(body.mapping.limit_policy).toBe('preview');
    expect(body.mapping.source_to_robot[0][1]).toBeCloseTo(-1);
    confirmed = true;
    return route.fulfill({json:{...job,state:'done',mapping_saved:true,
      summary:{mechanical_limits_enforced:false,covered_joints:['axis'],unmapped_joints:[]}}});
  });
  await page.route('**/api/motion/jobs/generic-fixture/preview.json', route => route.fulfill({json:preview}));
  await page.goto('/workbench/#/motion');
  await page.getByRole('button',{name:'导入并转换',exact:true}).click();
  await expect(page.getByRole('region',{name:'通用映射确认'})).toBeVisible();
  expect((await page.getByRole('region',{name:'通用映射确认'}).boundingBox())!.height).toBeGreaterThan(250);
  await expect(page.getByRole('button',{name:'确认映射并转换'})).toBeDisabled();
  await expect(page.getByLabel('限位策略')).toHaveValue('urdf');
  await page.getByLabel('unknown_link 人体对应').selectOption('LeftArm');
  await page.getByLabel('限位策略').selectOption('preview');
  await page.getByLabel('对齐 Z 角度').fill('90');
  await page.getByRole('button',{name:'确认映射并转换'}).click();
  await expect(page.getByText('转换完成',{exact:true})).toBeVisible();
  await expect(page.getByText('覆盖 1/1 个关节 · 几何预览，部分机械限位缺失')).toBeVisible();
  expect(confirmed).toBe(true);
});

test('已保存映射自动转换，仍可请求重新确认', async ({page}) => {
  let count=0;
  await page.route('**/api/motion/jobs', route => {
    const body=route.request().postDataJSON();
    count++;
    expect(body.review_mapping).toBe(count>1);
    return route.fulfill({json:{...job, mapping_origin:'saved',
      mapping:{...mapping,targets:[{target:'unknown_link',source:'LeftArm'}]},
      state:count===1?'done':'mapping'}});
  });
  await page.route('**/api/motion/jobs/generic-fixture/preview.json', route => route.fulfill({json:preview}));
  await page.goto('/workbench/#/motion');
  await page.getByRole('button',{name:'导入并转换',exact:true}).click();
  await expect(page.getByText('转换完成',{exact:true})).toBeVisible();
  await expect(page.getByText('复用已确认映射',{exact:true})).toBeVisible();
  await page.getByLabel('重新确认已记住的映射').check();
  await page.getByRole('button',{name:'导入并转换',exact:true}).click();
  await expect(page.getByRole('region',{name:'通用映射确认'})).toBeVisible();
  await expect(page.getByLabel('unknown_link 人体对应')).toHaveValue('LeftArm');
});
