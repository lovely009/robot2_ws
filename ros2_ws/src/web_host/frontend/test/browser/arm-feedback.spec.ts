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

test('关节轴标不遮挡电机，保留真实轴向、状态色和透明点击球',async({page})=>{
  await page.route('**/arm-fixture',route=>route.fulfill({contentType:'text/html',body:
    '<script type="importmap">{"imports":{"three":"/vendor/three.module.min.js","three/examples/jsm/":"/vendor/jsm/"}}</script>'}));
  await page.route('**/js/panels/arm.js',route=>route.fulfill({contentType:'text/javascript',
    body:readFileSync('../web/js/panels/arm.js','utf8')+'\nexport {addMarkers,markers};'}));
  await page.goto('/arm-fixture');
  const result=await page.evaluate(async()=>{
    const {URDFJoint,URDFRobot}=await import('/vendor/URDFClasses.js');
    const {BoxGeometry,Mesh,MeshStandardMaterial,Raycaster,Vector3}=await import('/vendor/three.module.min.js');
    const {addMarkers,markers,setJointColors,setSelected}=await import('/js/panels/arm.js');
    const joint=new URDFJoint(),robot=new URDFRobot();
    joint.name='joint_link3';joint.axis.set(1,2,3).normalize();
    const motor=new Mesh(new BoxGeometry(.1,.1,.1),new MeshStandardMaterial({color:0x123456}));
    joint.add(motor);robot.joints={joint_link3:joint};robot.add(joint);
    addMarkers(robot,['joint_link3']);
    const marker=markers.get('joint_link3'),[arrow,ring]=marker.children;
    marker.updateMatrixWorld(true);
    const arrowDirection=new Vector3(0,1,0).applyQuaternion(arrow.quaternion);
    const ringNormal=new Vector3(0,0,1).applyQuaternion(ring.quaternion);
    const originalMotorMaterial=motor.material;
    setJointColors({joint_link3:'#37d67a'});
    setSelected('joint_link3');
    const selectedScale=marker.scale.x;
    setSelected(null);
    return {
      invisible:marker.material.transparent && marker.material.opacity===0
        && !marker.material.depthWrite && !marker.material.colorWrite,
      pickable:new Raycaster(joint.axis.clone().multiplyScalar(-1),joint.axis).intersectObject(marker,false).length>0,
      onlyHitSphere:marker.geometry.type==='SphereGeometry'
        && marker.children.every(part=>part.geometry?.type!=='SphereGeometry'),
      axis:arrow.type==='ArrowHelper' && ring.isLineLoop
        && arrowDirection.distanceTo(joint.axis)<1e-9 && ringNormal.distanceTo(joint.axis)<1e-9,
      colored:arrow.line.material.color.getHexString()==='37d67a'
        && arrow.cone.material.color.getHexString()==='37d67a'
        && ring.material.color.getHexString()==='37d67a',
      axisVisible:[arrow.line,arrow.cone,ring].every(part=>!part.material.depthTest && !part.material.depthWrite && part.renderOrder===20),
      motorUntouched:motor.material===originalMotorMaterial && motor.material.color.getHexString()==='123456',
      selectedScale,resetScale:marker.scale.x,
    };
  });
  expect(result.invisible && result.pickable && result.onlyHitSphere && result.axis && result.colored && result.axisVisible && result.motorUntouched).toBe(true);
  expect(result.selectedScale).toBe(1.35);
  expect(result.resetScale).toBe(1);
});

test('肩腕 CAD 法兰和连杆随仿真辅助轴运动并保持球头闭合',async({page})=>{
  await page.route('**/arm-fixture',route=>route.fulfill({contentType:'text/html',body:
    '<script type="importmap">{"imports":{"three":"/vendor/three.module.min.js","three/examples/jsm/":"/vendor/jsm/"}}</script>'}));
  await page.route('**/js/panels/arm.js',route=>route.fulfill({contentType:'text/javascript',
    body:readFileSync('../web/js/panels/arm.js','utf8')+'\nexport {applyValues};'}));
  await page.goto('/arm-fixture');
  const result=await page.evaluate(async()=>{
    const {default:URDFLoader}=await import('/vendor/URDFLoader.js');
    const {Mesh,BoxGeometry,Vector3}=await import('/vendor/three.module.min.js');
    const {applyValues}=await import('/js/panels/arm.js');
    const base='/robot_sim/models/full_body_robot';
    const [urdf,mjcf]=await Promise.all([
      fetch(base+'/urdf/full_body_robot.standalone.urdf').then(r=>r.text()),
      fetch(base+'/mujoco/full_body_robot.xml').then(r=>r.text()),
    ]);
    const loader=new URDFLoader();
    loader.packages={robot_description:'/robot_sim'};
    loader.loadMeshCb=(_path,_manager,material,done)=>done(new Mesh(new BoxGeometry(.001,.001,.001),material));
    const robot=loader.parse(urdf);
    const xml=new DOMParser().parseFromString(mjcf,'text/xml');
    const names=Array.from(robot.urdfRobotNode.querySelectorAll('joint'),j=>j.getAttribute('name'));
    const sites=new Map(Array.from(xml.querySelectorAll('body > site'),site=>[
      site.getAttribute('name'),{link:site.parentElement.getAttribute('name'),
        position:site.getAttribute('pos').split(/\s+/).map(Number)},
    ]));
    const prefixes=['left_shoulder_flange_linkage','right_shoulder_flange_linkage',
      'left_wrist_flange_proximal','left_wrist_flange_distal',
      'right_wrist_flange_proximal','right_wrist_flange_distal'];
    const endpointErrors=[];
    for(const key of Array.from(xml.querySelectorAll('keyframe > key')).slice(1)){
      const qpos=key.getAttribute('qpos').split(/\s+/).map(Number);
      applyValues(robot,Object.fromEntries(names.map((name,i)=>[name,qpos[i]])));
      robot.updateMatrixWorld(true);
      for(const prefix of prefixes){
        const start=sites.get(prefix+'_motor_ball'),tip=sites.get(prefix+'_rod_tip');
        const end=sites.get(prefix+'_platform_ball');
        const world=site=>robot.links[site.link].localToWorld(new Vector3(...site.position));
        endpointErrors.push(world(tip).distanceTo(world(end)));
        endpointErrors.push(robot.links[prefix+'_rod'].localToWorld(new Vector3()).distanceTo(world(start)));
      }
    }
    return {maxError:Math.max(...endpointErrors),meshCounts:prefixes.map(prefix=>
      ['flange','rod','platform'].map(part=>Object.values(robot.visual).filter(visual=>visual.name===prefix+'_'+part+'_visual').length)),
      totalJoints:names.length};
  });
  expect(result.totalJoints).toBe(67);
  expect(result.meshCounts).toEqual(Array.from({length:6},()=>[1,1,1]));
  expect(result.maxError).toBeLessThan(1e-6);
});
