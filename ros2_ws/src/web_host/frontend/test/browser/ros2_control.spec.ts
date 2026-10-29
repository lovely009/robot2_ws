import { test, expect, type Page } from '@playwright/test';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

async function fixture(page: Page) {
  const errors:string[]=[];
  page.on('pageerror',e=>errors.push(e.message));
  const html=readFileSync(resolve('../web/index.html'),'utf8');
  const templates=html.match(/<template data-panel-template="(?:command|feedforward|actionLibrary|jointList)"[\s\S]*?<\/template>/g)!.join('');
  await page.route('http://control.test/**',route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/') return route.fulfill({contentType:'text/html',body:`<meta charset="UTF-8"><input id="rosUrl" value="ws://127.0.0.1:9090">${templates}<main></main>`});
    if(path==='/api/models') return route.fulfill({json:{models:['half_body_robot']}});
    if(path==='/api/model-transmissions') return route.fulfill({json:{model:'half_body_robot',direct_joints:{1:'elbow'}}});
    if(path==='/api/actions') return route.fulfill({json:{actions:[]}});
    if(path.startsWith('/js/')) return route.fulfill({contentType:'text/javascript',body:readFileSync(resolve('../web'+path),'utf8')});
    return route.abort();
  });
  await page.addInitScript(()=>{
    const w=window as any;
    w.published=[];w.services=[];w.topics=[];
    w.ROSLIB={
      Ros:class {handlers:Record<string,()=>void>={};on(key:string,fn:()=>void){this.handlers[key]=fn;if(key==='connection')queueMicrotask(fn);}close(){}},
      Topic:class {name:string;callback:any;constructor(o:any){this.name=o.name;w.topics.push(this);}subscribe(fn:any){this.callback=fn;}unsubscribe(){}advertise(){}unadvertise(){}publish(msg:any){w.published.push({topic:this.name,msg});}},
      Message:class {constructor(msg:any){Object.assign(this,msg);}},
      Service:class {name:string;constructor(o:any){this.name=o.name;}callService(args:any,done:any){w.services.push({name:this.name,args});done({ok:true});}},
    };
  });
  await page.goto('http://control.test/');
  await page.evaluate(async()=>{
    const w=window as any;
    const data=await import('/js/core/store.js');
    w.data=data;w.store=data.store;w.ros=await import('/js/core/ros.js');
    w.ros.connect();
    await new Promise(resolve=>setTimeout(resolve,0));
    const now=Date.now(),s=w.store;
    s.modelTransmissions={model:data.MODEL,groups:[{name:'elbow',motors:[{id:1}],joints:['elbow']}]};
    s.modelJoints=[{name:'elbow',type:'revolute',lower:-1,upper:1,velocity:1}];
    s.motors.set(1,{status:{online:true,lifecycle:4},motion:{position:.1}});
    s.jointStates={last:now,byName:{elbow:{position:.1,at:now}}};
    s.transmissionStatus={last:now,data:{model:data.MODEL,joint_names:['elbow'],trajectory_enabled:true,last_error:''}};
    setInterval(()=>{
      s.jointStates.last=Date.now();s.jointStates.byName.elbow.at=Date.now();
      s.transmissionStatus.last=Date.now();
    },100);
    w.controlStatus={model:data.MODEL,joint_names:['elbow'],mode:'trajectory',reason:'ok',reject:'',
      output_topic:'/motor/host_command_array',trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
      gravity_enabled:true,effort_gain:.8};
    data.applyCommandStatus(w.controlStatus);
    w.panels=[];
    for(const id of ['command','feedforward','actionLibrary','jointList']){
      const root=document.createElement('section');root.dataset.panel=id;
      root.append((document.querySelector(`template[data-panel-template="${id}"]`) as HTMLTemplateElement).content.cloneNode(true));
      document.querySelector('main')!.append(root);
      const panel=await import(`/js/panels/${id}.js`);
      w.panels.push(panel.mount(root,{}));
    }
  });
  return errors;
}

test('ros2_control 模式与重力回执据实展示，服务成功不冒充状态已改变',async({page})=>{
  const errors=await fixture(page);
  await expect(page.locator('#ffModel')).toHaveValue('half_body_robot');
  await expect(page.locator('#ffRoute')).toBeChecked();
  await expect(page.locator('#ffState')).toContainText('实际增益 0.8');
  await expect(page.locator('#cmdStatus')).toContainText('模式 trajectory');
  await page.locator('#ffRoute').click();
  await expect(page.locator('#ffResult')).toContainText('等待 /control/status 确认');
  await expect(page.locator('#ffRoute')).toBeChecked();
  expect(await page.evaluate(()=>(window as any).services)).toContainEqual({name:'/bench/stop_gravity_feedforward',args:{}});
  await page.evaluate(()=>{
    const w=window as any;w.data.applyCommandStatus({...w.controlStatus,gravity_enabled:false,effort_gain:.4,mode:'manual'});
    for(const panel of w.panels)panel.render(new Set([w.data.DIRTY.COMMAND]));
  });
  await expect(page.locator('#ffRoute')).not.toBeChecked();
  await expect(page.locator('#ffSummary')).toHaveText('补偿已关闭');
  await expect(page.locator('#ffState')).toContainText('模式 manual');
  await expect(page.locator('#ffState')).toContainText('实际增益 0.4');
  await page.evaluate(()=>{
    const w=window as any;w.data.applyCommandStatus({...w.controlStatus,gravity_enabled:null,effort_gain:null});
    for(const panel of w.panels)panel.render(new Set([w.data.DIRTY.COMMAND]));
  });
  await expect(page.locator('#ffSummary')).toHaveText('补偿状态未知');
  await expect(page.locator('#ffRoute')).toBeDisabled();
  await expect(page.locator('#ffState')).toContainText('实际增益 未知');
  expect(await page.evaluate(()=>(window as any).published)).toEqual([]);
  expect(errors).toEqual([]);
});

test('关节门禁拒绝旧模型、缺少同组关节和过期控制状态',async({page})=>{
  const errors=await fixture(page),send=page.locator('[data-joint-send]');
  await expect(send).toBeEnabled();
  for(const patch of [{model:'old_arm'},{joint_names:[]},null]){
    await page.evaluate(patch=>{
      const w=window as any;
      w.store.commandStatus={last:patch ? Date.now() : Date.now()-3000,data:{...w.controlStatus,...patch}};
    },patch);
    await expect(send).toBeDisabled();
  }
  expect(await page.evaluate(()=>(window as any).published)).toEqual([]);
  expect(errors).toEqual([]);
});

test('显式停止使用 Empty，动作取消只向客户端发空轨迹',async({page})=>{
  const errors=await fixture(page);
  await page.evaluate(async()=>{(await import('/js/panels/actionLibrary.js')).stopPlayback();});
  let published=await page.evaluate(()=>(window as any).published);
  expect(published).toHaveLength(1);
  expect(published[0]).toMatchObject({topic:'/control/trajectory',msg:{count:0,target_ids:[],points:[]}});
  await page.locator('#btnStopCmd').click();
  published=await page.evaluate(()=>(window as any).published);
  expect(published.at(-1)).toEqual({topic:'/control/stop',msg:{}});
  expect(published.filter((p:any)=>p.topic==='/control/setpoint')).toEqual([]);
  await expect(page.locator('#cmdResult')).toContainText('全部软停');
  await expect(page.locator('#btnActStop')).toHaveText('取消轨迹');
  expect(errors).toEqual([]);
});

test('滑块仅提交未来目标，不将浏览器反馈作为 t=0 路点',async({page})=>{
  const errors=await fixture(page);
  const slider=page.locator('[data-joint="elbow"] input[type="range"]');
  await expect(slider).toBeEnabled();
  await slider.evaluate((element:HTMLInputElement)=>{
    element.value='0.3';element.dispatchEvent(new Event('input',{bubbles:true}));
  });
  await expect.poll(()=>page.evaluate(()=>(window as any).published.length)).toBe(1);
  const [{topic,msg}]=await page.evaluate(()=>(window as any).published);
  expect(topic).toBe('/arm_controller/joint_trajectory');
  expect(msg.points).toHaveLength(1);
  expect(msg.points[0].positions).toEqual([.3]);
  expect(msg.points[0].time_from_start.sec+msg.points[0].time_from_start.nanosec/1e9).toBeGreaterThan(0);
  expect(errors).toEqual([]);
});
