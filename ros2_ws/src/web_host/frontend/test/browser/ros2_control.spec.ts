import { test, expect, type Page } from '@playwright/test';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

async function fixture(page: Page) {
  const errors:string[]=[];
  page.on('pageerror',e=>errors.push(e.message));
  const html=readFileSync(resolve('../web/panel_templates.html'),'utf8');
  const templates=html.match(/<template data-panel-template="(?:jointList)"[\s\S]*?<\/template>/g)!.join('');
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
    w.ros.configureControl({control_mode:'bench'});
    w.ros.connect();
    await new Promise(resolve=>setTimeout(resolve,0));
    const now=Date.now(),s=w.store;
    s.modelTransmissions={model:data.MODEL,groups:[{name:'elbow',motors:[{id:1}],joints:['elbow']}]};
    s.modelJoints=[{name:'elbow',type:'revolute',lower:-1,upper:1,velocity:1}];
    s.motors.set(1,{status:{online:true,lifecycle:4},motion:{position:.1}});
    s.jointStates={last:now,byName:{elbow:{position:.1,at:now}}};
    s.transmissionStatus={last:now,data:{model:data.MODEL,joint_names:['elbow'],joint_limits:{elbow:{lower:-1,upper:1}},trajectory_enabled:true,last_error:''}};
    setInterval(()=>{
      s.jointStates.last=Date.now();s.jointStates.byName.elbow.at=Date.now();
      s.transmissionStatus.last=Date.now();
    },100);
    w.controlStatus={model:data.MODEL,joint_names:['elbow'],mode:'trajectory',reason:'ok',reject:'',
      output_topic:'/motor/host_command_array',trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
      gravity_enabled:true,effort_gain:.8};
    data.applyCommandStatus(w.controlStatus);
    w.panels=[];
    for(const id of ['jointList']){
      const root=document.createElement('section');root.dataset.panel=id;
      root.append((document.querySelector(`template[data-panel-template="${id}"]`) as HTMLTemplateElement).content.cloneNode(true));
      document.querySelector('main')!.append(root);
      const panel=await import(`/js/panels/${id}.js`);
      w.panels.push(panel.mount(root,{}));
    }
  });
  return errors;
}

test('关节门禁拒绝旧模型、缺少同组关节和过期控制状态',async({page})=>{
  const errors=await fixture(page),send=page.locator('[data-joint="elbow"] input[type="range"]');
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

test('滑块提交连续位置目标，由主机生成平滑指令',async({page})=>{
  const errors=await fixture(page);
  const slider=page.locator('[data-joint="elbow"] input[type="range"]');
  await expect(slider).toBeEnabled();
  await slider.evaluate((element:HTMLInputElement)=>{
    element.value='0.3';element.dispatchEvent(new Event('input',{bubbles:true}));
  });
  await expect.poll(()=>page.evaluate(()=>(window as any).published.length)).toBe(1);
  const [{topic,msg}]=await page.evaluate(()=>(window as any).published);
  expect(topic).toBe('/control/joint_target');
  expect(msg.name).toEqual(['elbow']);
  expect(msg.position).toEqual([.3]);
  expect(msg.velocity).toEqual([]);
  expect(msg).not.toHaveProperty('points');
  expect(errors).toEqual([]);
});

test('18关节模型仅接入一个完整组时，只开放该关节',async({page})=>{
  const errors=await fixture(page);
  await page.evaluate(()=>{
    const w=window as any,s=w.store;
    for(let id=2;id<=18;id++){
      const name=`unplugged_${id}`;
      s.modelTransmissions.groups.push({name,motors:[{id}],joints:[name]});
      s.modelJoints.push({name,type:'revolute',lower:-1,upper:1,velocity:1});
    }
    for(const panel of w.panels)panel.render(new Set(['model','status','motion']));
  });
  await expect(page.locator('#sumJoints')).toHaveText('1/18 关节');
  await expect(page.getByRole('checkbox',{name:'使能控制',exact:true})).toBeEnabled();
  await expect(page.locator('[data-joint="elbow"] input[type="range"]')).toBeEnabled();
  await expect(page.locator('[data-joint="unplugged_2"] input[type="range"]')).toBeDisabled();
  expect(errors).toEqual([]);
});
