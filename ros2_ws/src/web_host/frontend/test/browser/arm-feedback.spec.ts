import {test,expect} from '@playwright/test';
import {readFileSync} from 'node:fs';

test('3D 显示真实关节角，不按 URDF 默认范围截断或改写模型限位',async({page})=>{
  await page.route('**/arm-fixture',route=>route.fulfill({contentType:'text/html',body:
    '<script type="importmap">{"imports":{"three":"/vendor/three.module.min.js","three/examples/jsm/":"/vendor/jsm/"}}</script>'}));
  await page.route('**/js/panels/arm.js',route=>route.fulfill({contentType:'text/javascript',
    body:readFileSync('../web/js/panels/arm.js','utf8')+'\nexport {applyValues};'}));
  await page.goto('/arm-fixture');
  const result=await page.evaluate(async()=>{
    const {URDFJoint,URDFRobot}=await import('/vendor/URDFClasses.js');
    const {applyValues}=await import('/js/panels/arm.js');
    const joint=new URDFJoint(),robot=new URDFRobot();
    joint.jointType='revolute';joint.name='joint_link2';
    Object.assign(joint.limit,{lower:-.12,upper:.14,effort:10,velocity:2});
    joint.urdfNode=new DOMParser().parseFromString('<joint name="joint_link2"><limit lower="-.12" upper=".14"/></joint>','text/xml').documentElement;
    const node=joint.urdfNode,xml=node.outerHTML,limits={...joint.limit};
    robot.joints={joint_link2:joint};robot.add(joint);
    applyValues(robot,{joint_link2:1.952,unknown:2});
    const angle=joint.angle;
    applyValues(robot,{joint_link2:NaN});
    return {angle,afterInvalid:joint.angle,limits:joint.limit,originalLimits:limits,
      sameNode:joint.urdfNode===node,sameXml:joint.urdfNode.outerHTML===xml};
  });
  expect(result.angle).toBeCloseTo(1.952,9);
  expect(result.afterInvalid).toBeCloseTo(1.952,9);
  expect(result.limits).toEqual(result.originalLimits);
  expect(result.sameNode && result.sameXml).toBe(true);
});
