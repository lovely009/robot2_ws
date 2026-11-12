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
      state:count===1?'done':'mapping', result_cached:count===1}});
  });
  await page.route('**/api/motion/jobs/generic-fixture/preview.json', route => route.fulfill({json:preview}));
  await page.goto('/workbench/#/motion');
  await page.getByRole('button',{name:'导入并转换',exact:true}).click();
  await expect(page.getByText('转换完成',{exact:true})).toBeVisible();
  await expect(page.getByText('复用已确认映射',{exact:true})).toBeVisible();
  await expect(page.getByText('已加载保存结果，无需重新转换')).toBeVisible();
  await page.getByLabel('重新确认已记住的映射').check();
  await page.getByRole('button',{name:'导入并转换',exact:true}).click();
  await expect(page.getByRole('region',{name:'通用映射确认'})).toBeVisible();
  await expect(page.getByLabel('unknown_link 人体对应')).toHaveValue('LeftArm');
});

test('整机模型提交注册名称并显示仿真范围，不再提示37项缺失限位', async ({page}) => {
  await page.route('**/api/runtime', route => route.fulfill({json:{control_mode:'readonly',robot_model:'full_body_robot'}}));
  await page.route('**/api/models', route => route.fulfill({json:{models:['full_body_robot']}}));
  await page.route('**/robot_sim/models/full_body_robot/urdf/full_body_robot.urdf', route => route.fulfill({body:'<robot name="full_body_robot"/>'}));
  const configured = {...job, robot_name:'full_body_robot', joint_count:37, warnings:[],
    constraint_source:'simulation',velocity_limit_rad_s:Math.PI,
    mapping:{...mapping,targets:[{target:'unknown_link',source:'LeftArm'}]}};
  await page.route('**/api/motion/jobs', route => {
    expect(route.request().postDataJSON().model).toBe('full_body_robot');
    return route.fulfill({json:configured});
  });
  await page.route('**/api/motion/jobs/generic-fixture/convert', route => {
    expect(route.request().postDataJSON().mapping.limit_policy).toBe('urdf');
    return route.fulfill({json:{...configured,state:'done',summary:{simulation_limits_enforced:true,
      mechanical_limits_enforced:false,direction_max_deg:12.3,orientation_max_deg:90,
      covered_joints:Array.from({length:37},(_,i)=>`axis_${i}`)}}});
  });
  await page.route('**/api/motion/jobs/generic-fixture/preview.json', route => route.fulfill({json:preview}));
  await page.goto('/workbench/#/motion');
  await page.getByRole('button',{name:'导入并转换',exact:true}).click();
  await expect(page.locator('details.motion-mapping-notes')).toHaveCount(0);
  await expect(page.getByText('仿真几何与耦合限位 · 3.14 rad/s')).toBeVisible();
  await expect(page.getByLabel('限位策略')).toHaveValue('urdf');
  await expect(page.getByLabel('限位策略').locator('option')).toHaveCount(1);
  await expect(page.getByText(/按人体骨段方向拟合姿态/)).toBeVisible();
  await page.getByRole('button',{name:'确认映射并转换'}).click();
  await expect(page.getByText('覆盖 37/37 个关节 · 已检查仿真位置、速度和耦合范围')).toBeVisible();
  await expect(page.getByText('最大骨段方向误差 12.3°；详细目标误差见报告')).toBeVisible();
  await expect(page.getByText(/最大朝向误差/)).toHaveCount(0);
  await expect(page.getByRole('link',{name:'导出预览 URDF'})).toHaveAttribute('href','/api/motion/jobs/generic-fixture/robot.urdf');
});

test('非双足预览按参考实体最低点放到地面并保持根高度', async ({page}) => {
  await page.goto('/workbench/#/motion');
  const result = await page.evaluate(async () => {
    // @ts-expect-error The preview and loader are served as browser modules.
    const {default: Loader} = await import('/vendor/URDFLoader.js');
    // @ts-expect-error The preview module is shared with the legacy renderer.
    const {createMotionPreview} = await import('/js/motion_preview.js');
    let model: any;
    const parse = Loader.prototype.parse;
    Loader.prototype.parse = function (...args: any[]) {
      model = parse.apply(this, args);
      return model;
    };
    const canvas = document.createElement('canvas');
    document.body.appendChild(canvas);
    const frames = [[[0,0,0],[0,0,-1]],[[0,0,0],[0,0,-1]]];
    let viewer;
    try {
      viewer = createMotionPreview(canvas, {
        times:[0,1], source_to_robot:[[1,0,0],[0,1,0],[0,0,1]],
        source:{names:['root','foot'],parents:[-1,0],frames},
        robot:{names:['root','foot'],parents:[-1,0],frames,
          joint_names:['slide'],joint_positions:[[0],[.3]],
          urdf:'<robot name="fixture"><link name="root"/><link name="foot"><visual><origin xyz="0 0 -1"/><geometry><box size="0.2 0.2 0.2"/></geometry></visual></link><joint name="slide" type="prismatic"><parent link="root"/><child link="foot"/><axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>'},
      });
      const height = model.parent.position.z;
      viewer.show(1);
      return {height, playbackHeight:model.parent.position.z, position:model.joints.slide.jointValue[0]};
    } finally {
      viewer?.dispose();
      Loader.prototype.parse = parse;
      canvas.remove();
    }
  });
  expect(result.height).toBeCloseTo(1.1);
  expect(result.playbackHeight).toBeCloseTo(1.1);
  expect(result.position).toBeCloseTo(.3);
});

test('双足预览随下蹲降低基座，保留腾空和单足抬起，不以手作为足底', async ({page}) => {
  await page.goto('/workbench/#/motion');
  const result = await page.evaluate(async () => {
    // @ts-expect-error Browser-served module.
    const {default: Loader} = await import('/vendor/URDFLoader.js');
    // @ts-expect-error Browser-served module.
    const {createMotionPreview} = await import('/js/motion_preview.js');
    let model: any;
    const parse = Loader.prototype.parse;
    Loader.prototype.parse = function (...args: any[]) {
      model = parse.apply(this, args);
      return model;
    };
    const canvas = document.createElement('canvas');
    document.body.appendChild(canvas);
    let viewer;
    try {
      const source = [
        [[0,0,2],[-.1,0,0],[.1,0,0],[0,0,-3]],
        [[0,0,1.5],[-.1,0,0],[.1,0,0],[0,0,-3]],
        [[0,0,2.4],[-.1,0,.4],[.1,0,.4],[0,0,-3]],
        [[0,0,2],[-.1,0,.2],[.1,0,0],[0,0,-3]],
      ];
      viewer = createMotionPreview(canvas, {
        times:[0,1,2,3],source_to_robot:[[1,0,0],[0,1,0],[0,0,1]],
        source:{names:['root','LeftFoot','RightFoot','hand'],parents:[-1,0,0,0],frames:source},
        robot:{names:['root','LeftFoot','RightFoot'],parents:[-1,0,0],
          frames:[[[0,0,0],[-.1,0,-1],[.1,0,-1]],[[0,0,0],[-.1,0,-.7],[.1,0,-.7]],
                  [[0,0,0],[-.1,0,-1],[.1,0,-1]],[[0,0,0],[-.1,0,-.8],[.1,0,-1]]],
          root_rotation_source:'root',joint_names:['left','right'],
          joint_positions:[[0,0],[.3,.3],[0,0],[.2,0]],
          urdf:'<robot name="feet"><link name="root"/><link name="LeftFoot"><visual><geometry><box size=".2 .2 .2"/></geometry></visual></link><link name="RightFoot"><visual><geometry><box size=".2 .2 .2"/></geometry></visual></link><link name="hand"><visual><origin xyz="0 0 -3"/><geometry><box size=".2 .2 .2"/></geometry></visual></link><joint name="left" type="prismatic"><parent link="root"/><child link="LeftFoot"/><origin xyz="-.1 0 -1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><joint name="right" type="prismatic"><parent link="root"/><child link="RightFoot"/><origin xyz=".1 0 -1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><joint name="hand_fixed" type="fixed"><parent link="root"/><child link="hand"/></joint></robot>'},
      });
      const sample = (index: number) => {
        viewer.show(index);
        return {height:model.parent.position.z,soles:['LeftFoot','RightFoot'].map(name =>
          model.links[name].getWorldPosition(model.links[name].position.clone()).z - .1)};
      };
      const standing = sample(0), crouching = sample(1), jumping = sample(2), oneFoot = sample(3);
      const repeatedJump = sample(2);
      return {standing,crouching,jumping,oneFoot,repeatedJump,joints:model.joints.left.jointValue};
    } finally {
      viewer?.dispose();
      Loader.prototype.parse = parse;
      canvas.remove();
    }
  });
  expect(result.standing.height).toBeCloseTo(1.1);
  expect(result.crouching.height).toBeCloseTo(.8);
  for (const sole of result.crouching.soles) expect(sole).toBeCloseTo(0);
  // The robot's reference root-to-sole height is 1.1m; the source's is 2m.
  for (const sole of result.jumping.soles) expect(sole).toBeCloseTo(.4 * 1.1 / 2);
  expect(result.oneFoot.soles[0]).toBeCloseTo(.2);
  expect(result.oneFoot.soles[1]).toBeCloseTo(0);
  expect(result.repeatedJump).toEqual(result.jumping);
  expect(result.joints[0]).toBeCloseTo(0);
});

test('机器人实体将机器人坐标中的根旋转恢复到人体世界坐标', async ({page}) => {
  await page.goto('/workbench/#/motion');
  const result = await page.evaluate(async () => {
    // @ts-expect-error Browser-served module.
    const {default: Loader} = await import('/vendor/URDFLoader.js');
    // @ts-expect-error Browser-served module.
    const {createMotionPreview} = await import('/js/motion_preview.js');
    let model: any;
    const parse = Loader.prototype.parse;
    Loader.prototype.parse = function (...args: any[]) {
      model = parse.apply(this, args);
      return model;
    };
    const canvas = document.createElement('canvas');
    document.body.appendChild(canvas);
    let viewer;
    try {
      const frames = [[[0,0,0],[1,0,0]],[[0,0,0],[1,0,0]]];
      viewer = createMotionPreview(canvas, {
        times:[0,1],source_to_robot:[[0,-1,0],[1,0,0],[0,0,1]],
        source:{names:['root','arm'],parents:[-1,0],frames},
        robot:{names:['root','arm'],parents:[-1,0],frames,
          root_quaternions:[[0,Math.SQRT1_2,0,Math.SQRT1_2],[0,0,0,1]],
          joint_names:['turn'],joint_positions:[[0],[0]],
          urdf:'<robot name="fixture"><link name="root"/><link name="arm"><visual><geometry><box size=".2 .2 .2"/></geometry></visual></link><joint name="turn" type="revolute"><parent link="root"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>'},
      });
      const first = model.parent.quaternion.toArray();
      const direction = () => model.links.arm.getWorldPosition(model.links.arm.position.clone())
        .sub(model.links.root.getWorldPosition(model.links.root.position.clone())).toArray();
      const firstDirection = direction();
      viewer.show(1);
      return {first,second:model.parent.quaternion.toArray(),firstDirection,secondDirection:direction()};
    } finally {
      viewer?.dispose();
      Loader.prototype.parse = parse;
      canvas.remove();
    }
  });
  // The root quaternion is in robot coordinates: source X rotation becomes robot Y rotation.
  expect(result.first[0]).toBeCloseTo(.5);
  expect(result.first[1]).toBeCloseTo(.5);
  expect(result.first[2]).toBeCloseTo(-.5);
  expect(result.first[3]).toBeCloseTo(.5);
  expect(result.second[2]).toBeCloseTo(-Math.SQRT1_2);
  expect(result.second[3]).toBeCloseTo(Math.SQRT1_2);
  // Check actual entity link positions in world space, not only the parent quaternion.
  for (const [i, value] of [0,-1,0].entries()) expect(result.firstDirection[i]).toBeCloseTo(value);
  for (const [i, value] of [0,0,1].entries()) expect(result.secondDirection[i]).toBeCloseTo(value);
});
