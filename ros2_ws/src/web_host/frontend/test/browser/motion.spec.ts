import { test, expect } from '@playwright/test';

const job = { id: 'fixture', state: 'running', progress: 30, bvh_name: '动作.bvh',
  urdf_name: 'robot.urdf', robot_name: 'half_body_robot', frames: 120, frame_time: .02,
  joint_count: 18, source_joints: ['Spine3'], robot_links: ['base_link'], mapping: null };
const preview = { times: [0, 1, 2], duration: 2,
  source: { names: ['Hips', 'Head'], parents: [-1, 0], frames: [[[0, 0, 1], [0, 0, 1.8]], [[0, 0, 1], [.2, 0, 1.8]], [[0, 0, 1], [.3, 0, 1.8]]] },
  robot: { names: ['base', 'wrist'], parents: [-1, 0], frames: [[[0, 0, 0], [.5, 0, 1]], [[0, 0, 0], [.7, 0, 1]], [[0, 0, 0], [.8, 0, 1]]] },
  source_to_robot: [[0, -1, 0], [1, 0, 0], [0, 0, 1]] };

test('动作工作台导入、进度、播放、导出与切页恢复', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => socket.close());
  let polls = 0;
  await page.route('**/api/motion/**', route => {
    const path = new URL(route.request().url()).pathname;
    if (path.endsWith('preview.json')) return route.fulfill({ json: preview });
    if (route.request().method() === 'POST') {
      const body = route.request().postDataJSON();
      expect(body.bvh_name).toBe('动作.bvh');
      expect(body.urdf_text).toContain('robot');
      return route.fulfill({ status: 202, json: job });
    }
    return route.fulfill({ json: ++polls < 2 ? job : { ...job, state: 'done', progress: 100 } });
  });
  await page.goto('/workbench/#/run');
  await page.getByRole('button', { name: '动作工作台', exact: true }).click();
  await expect(page).toHaveURL(/#\/motion$/);
  await expect(page.getByRole('heading', { name: /动作工作台/ })).toBeVisible();
  await expect(page.getByRole('button', { name: '导入并转换' })).toBeDisabled();
  await page.getByLabel('导入 BVH 文件', { exact: true }).setInputFiles({ name: '动作.bvh', mimeType: 'text/plain', buffer: Buffer.from('HIERARCHY') });
  await page.getByLabel('导入 URDF 文件', { exact: true }).setInputFiles({ name: 'robot.urdf', mimeType: 'text/xml', buffer: Buffer.from('<robot name="fixture"/>') });
  await page.getByRole('button', { name: '导入并转换' }).click();
  await expect(page.getByRole('progressbar', { name: '动作转换进度' })).toHaveJSProperty('value', 30);
  await expect(page.getByText('转换完成', { exact: true })).toBeVisible();
  await expect(page.getByLabel('人体与机器人动作三维预览')).toBeVisible();
  await expect(page.getByRole('link', { name: '导出关节动作' })).toHaveAttribute('href', '/api/motion/jobs/fixture/motion.npz');
  await page.getByRole('button', { name: '暂停预览', exact: true }).click();
  await expect(page.getByRole('button', { name: '播放预览', exact: true })).toBeEnabled();
  await page.getByLabel('动作播放进度', { exact: true }).fill('1.2');
  await expect(page.getByText('1.20 s / 2.00 s')).toBeVisible();
  await page.getByRole('button', { name: '设备工作台', exact: true }).click();
  await page.getByRole('button', { name: '动作工作台', exact: true }).click();
  await expect(page.getByText('转换完成', { exact: true })).toBeVisible();
  expect(errors).toEqual([]);
});

test('动作导入错误明确显示且可以重新提交', async ({ page }) => {
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => socket.close());
  await page.route('**/api/motion/jobs', route => route.fulfill({ status: 400, json: { error: 'BVH 文件格式错误' } }));
  await page.goto('/workbench/#/motion');
  for (const [label, name] of [['导入 BVH 文件', 'bad.bvh'], ['导入 URDF 文件', 'robot.urdf']]) {
    await page.getByLabel(label!, { exact: true }).setInputFiles({ name: name!, mimeType: 'text/plain', buffer: Buffer.from('invalid') });
  }
  await page.getByRole('button', { name: '导入并转换' }).click();
  await expect(page.getByRole('alert')).toContainText('BVH 文件格式错误');
  await expect(page.getByRole('button', { name: '导入并转换' })).toBeEnabled();
});

test('陌生模型先确认肩肘腕映射再转换', async ({ page }) => {
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => socket.close());
  const sources = ['Spine3', ...['Right', 'Left'].flatMap(s => ['UpperArm', 'LowerArm', 'Hand'].map(p => s + p))];
  const links = ['r_shoulder', 'r_elbow', 'r_wrist', 'l_shoulder', 'l_elbow', 'l_wrist'];
  let submitted = false;
  await page.route('**/api/motion/**', route => {
    const path = new URL(route.request().url()).pathname;
    if (path.endsWith('preview.json')) return route.fulfill({ json: preview });
    if (path.endsWith('convert')) {
      const { mapping } = route.request().postDataJSON();
      expect(mapping.arms.right.robot).toEqual(links.slice(0, 3));
      expect(mapping.arms.left.robot).toEqual(links.slice(3));
      submitted = true;
      return route.fulfill({ json: { ...job, state: 'done', progress: 100 } });
    }
    return route.fulfill({ json: { ...job, state: 'mapping', source_joints: sources, robot_links: links } });
  });
  await page.goto('/workbench/#/motion');
  for (const [label, name] of [['导入 BVH 文件', 'test.bvh'], ['导入 URDF 文件', 'new.urdf']])
    await page.getByLabel(label!, { exact: true }).setInputFiles({ name: name!, mimeType: 'text/plain', buffer: Buffer.from('fixture') });
  await page.getByRole('button', { name: '导入并转换' }).click();
  await expect(page.getByText('骨架对应关系', { exact: true })).toBeVisible();
  await expect(page.getByRole('button', { name: '确认映射并转换' })).toBeDisabled();
  for (const [a, side] of ['right', 'left'].entries())
    for (const [i, part] of ['肩', '肘', '腕'].entries())
      await page.getByLabel(`${side} 机器人${part}`, { exact: true }).selectOption(links[a * 3 + i]!);
  await page.getByRole('button', { name: '确认映射并转换' }).click();
  await expect(page.getByText('转换完成', { exact: true })).toBeVisible();
  expect(submitted).toBe(true);
});

test('转换期间服务丢失任务后可重新导入', async ({ page }) => {
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => socket.close());
  await page.route('**/api/motion/**', route => route.request().method() === 'POST'
    ? route.fulfill({ json: job })
    : route.fulfill({ status: 404, json: { error: '动作任务已过期，请重新导入' } }));
  await page.goto('/workbench/#/motion');
  for (const [label, name] of [['导入 BVH 文件', 'test.bvh'], ['导入 URDF 文件', 'robot.urdf']])
    await page.getByLabel(label!, { exact: true }).setInputFiles({ name: name!, mimeType: 'text/plain', buffer: Buffer.from('fixture') });
  await page.getByRole('button', { name: '导入并转换' }).click();
  await expect(page.getByRole('alert')).toContainText('动作任务已过期');
  await expect(page.getByRole('button', { name: '导入并转换' })).toBeEnabled();
});

const visualUrdf = `<robot name="fixture">
  <link name="base"><visual><origin xyz="0 0 .35"/><geometry><box size=".2 .2 .7"/></geometry></visual></link>
  <link name="wrist"><visual><origin xyz=".35 0 0"/><geometry><box size=".7 .08 .08"/></geometry></visual></link>
  <joint name="swivel" type="revolute"><parent link="base"/><child link="wrist"/>
    <origin xyz="0 0 .7"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
</robot>`;

async function modelPreview(page: import('@playwright/test').Page, urdf: string) {
  await page.routeWebSocket('ws://127.0.0.1:9090', socket => socket.close());
  const data = { ...preview, source: { ...preview.source, frames: preview.source.frames.map(() => preview.source.frames[0]) },
    robot: { ...preview.robot, frames: preview.robot.frames.map(() => preview.robot.frames[0]),
      urdf, joint_names: ['swivel'], joint_positions: [[0], [1], [1.5]] } };
  await page.route('**/api/motion/**', route => route.fulfill({ json:
    route.request().url().endsWith('preview.json') ? data : { ...job, state: 'done', progress: 100 } }));
  await page.addInitScript(() => sessionStorage.setItem('robot2.motionJob', 'fixture'));
  await page.goto('/workbench/#/motion');
}

test('URDF实体模型由关节角驱动，拖动进度改变实体姿态', async ({ page }) => {
  await modelPreview(page, visualUrdf);
  await expect(page.getByLabel('机器人模型状态', { exact: true })).toHaveText('机器人实体模型');
  await page.getByRole('button', { name: '暂停预览', exact: true }).click();
  await page.getByLabel('动作播放进度', { exact: true }).fill('0');
  const canvas = page.getByLabel('人体与机器人动作三维预览');
  const before = await canvas.screenshot();
  await page.getByLabel('动作播放进度', { exact: true }).fill('1.2');
  const after = await canvas.screenshot();
  // Source and robot wireframe samples are constant: only URDF joint geometry can move.
  expect(before.equals(after)).toBe(false);
});

test('URDF网格缺失时显示提示并保留骨架播放', async ({ page }) => {
  await page.route('**/robot_sim/missing.stl', route => route.fulfill({ status: 404, body: 'not found' }));
  await modelPreview(page, visualUrdf.replace('<box size=".7 .08 .08"/>', '<mesh filename="package://robot_sim/missing.stl"/>'));
  await expect(page.getByLabel('机器人模型状态', { exact: true })).toHaveText('部分网格缺失，保留骨架');
  await expect(page.getByLabel('人体与机器人动作三维预览')).toBeVisible();
  await page.getByLabel('动作播放进度', { exact: true }).fill('1.2');
  await expect(page.getByText('1.20 s / 2.00 s')).toBeVisible();
});
