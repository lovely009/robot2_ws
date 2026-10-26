import { store, MODEL } from '../core/store.js';
import { buildJointTrajectory, jointControlError } from '../core/joint_control.js';
import { pub, ensurePublisher, isConnected, getConnectionGeneration } from '../core/ros.js';

export function mount(root, {onStop}) {
  const box = document.createElement('div');
  box.className = 'card';
  box.innerHTML = '<div class="panel-title">关节目标</div>'
    + '<div class="ctrl-row"><label>关节 <select data-joint-name aria-label="控制关节"></select></label>'
    + '<label>目标 rad <input data-joint-target type="number" step="0.01" aria-label="关节目标 rad"></label></div>'
    + '<div class="ctrl-row"><label>到达时间 s <input data-joint-duration type="number" min="0.01" max="60" step="0.1" value="1"></label>'
    + '<button data-joint-read class="secondary">读取当前位置</button>'
    + '<button data-joint-send class="primary">发送关节目标</button>'
    + '<button data-joint-stop class="danger">停止全部</button></div>'
    + '<p data-joint-hint class="tiny"></p><p data-joint-result class="result tiny"></p>';
  root.prepend(box);
  const select=box.querySelector('[data-joint-name]');
  const target=box.querySelector('[data-joint-target]');
  const duration=box.querySelector('[data-joint-duration]');
  const send=box.querySelector('[data-joint-send]');
  const hint=box.querySelector('[data-joint-hint]');
  const result=box.querySelector('[data-joint-result]');
  let key='', selected='', generation=-1, advertised=0;
  const read=() => {
    const value=store.jointStates?.byName?.[select.value]?.position;
    target.value=Number.isFinite(value) ? String(value) : '';
  };
  select.onchange=() => {store.selectedJoint=select.value;selected=select.value;read();};
  box.querySelector('[data-joint-read]').onclick=read;
  box.querySelector('[data-joint-stop]').onclick=onStop;
  send.onclick=() => {
    try {
      if (!target.value.trim()) throw new Error('请先输入关节目标');
      const message=buildJointTrajectory(store, MODEL, select.value, Number(target.value), Number(duration.value));
      if (!pub('jointTrajectory',message)) throw new Error('连接已断开，未提交目标');
      result.textContent=`已提交 ${select.value} 目标，请核对实测反馈与轨迹回执`;
      result.className='result tiny ok';
    } catch (error) {
      result.textContent=String(error.message || error);result.className='result tiny err';
    }
  };
  const render=() => {
    const now=Date.now();
    if (isConnected() && generation!==getConnectionGeneration()) {
      ensurePublisher('jointTrajectory');generation=getConnectionGeneration();advertised=now;
    }
    const groups=store.modelTransmissions?.model===MODEL ? store.modelTransmissions.groups || [] : [];
    const names=groups.flatMap(g=>g.joints);
    const next=MODEL+'|'+names.join('|');
    if (next!==key) {
      key=next;select.replaceChildren(...names.map(name=>{
        const option=document.createElement('option');option.value=name;option.textContent=name;return option;
      }));
      selected='';
    }
    if (names.includes(store.selectedJoint) && selected!==store.selectedJoint) {
      select.value=store.selectedJoint;selected=select.value;read();
    } else if (!selected && select.value) {
      selected=select.value;store.selectedJoint=selected;read();
    }
    const error=jointControlError(store, MODEL, select.value, now);
    send.disabled=!!error || now-advertised<300;
    box.querySelector('[data-joint-stop]').disabled=!isConnected();
    const group=groups.find(g=>g.joints.includes(select.value));
    hint.textContent=error || (group ? `${group.name} · 关联 ${group.motors.map(m=>'m'+m.id).join(' + ')} · 同组其他关节保持已有目标` : '等待模型传动配置');
    const backendError=store.transmissionStatus?.data?.last_error;
    if (backendError) {result.textContent=`主机拒绝：${backendError}`;result.className='result tiny err';}
  };
  render();
  const timer=setInterval(render,250);
  return {render,unmount(){clearInterval(timer);box.remove();}};
}
