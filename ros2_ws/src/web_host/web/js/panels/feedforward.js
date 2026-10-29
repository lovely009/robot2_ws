// bench 服务保留名称；实际状态只认 /control/status，页面不切换运动路由。
import { $ } from '../core/format.js';
import { store, DIRTY, MODEL, availableModels, modelJointsFor, jointMapSpec } from '../core/store.js';
import * as bench from '../core/agent.js';

const STORAGE_KEY = 'robot2.feedforward.options.v1';

export function mount(root, ctx) {
  const ui = {route:$('ffRoute'), model:$('ffModel'), gain:$('ffGain'),
    revert:$('btnFfRevert'), summary:$('ffSummary'), state:$('ffState'),
    alarm:$('ffAlarm'), result:$('ffResult'), hint:$('ffHint')};
  let busy = false;
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}') || {}; }
  catch (_error) { /* 存储损坏不影响控制状态。 */ }
  const summary = root.querySelector('summary');
  if (summary) summary.title = '重力由 ros2_control gravity_controller 计算';
  ui.route.closest('label').title = '启停 gravity_controller；结果以控制状态回执为准';
  ui.gain.title = 'gravity_controller 的力矩增益；结果以控制状态回执为准';
  ui.revert.hidden = true;

  function status() {
    const {data,last} = store.commandStatus, age = Date.now()-last;
    return store.conn.connected && last && age>=0 && age<=2500 ? data : null;
  }
  function setResult(text, kind='') {
    ui.result.textContent = text;
    ui.result.className = `result tiny${kind ? ` ${kind}` : ''}`;
  }
  function readGain() {
    const value = Number(ui.gain.value);
    if (!ui.gain.value.trim() || !Number.isFinite(value) || value<0) {
      throw new Error('增益必须是不小于 0 的数字');
    }
    return value;
  }
  function saveOptions() {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify({model:ui.model.value,effort_gain:ui.gain.value})); }
    catch (_error) { /* 禁用存储时仍可操作。 */ }
  }
  function render() {
    const data = status(), enabled = data?.gravity_enabled === true;
    const known = typeof data?.gravity_enabled === 'boolean' && Number.isFinite(data?.effort_gain);
    const active = enabled && data.mode === 'trajectory';
    ui.route.checked = enabled;
    ui.route.indeterminate = Boolean(data && !known);
    ui.route.disabled = busy || !known || !data || data.model!==MODEL || data.model!==ui.model.value;
    ui.gain.disabled = busy || !enabled || !known || data.model!==ui.model.value;
    ui.model.disabled = busy;
    if (!busy && document.activeElement!==ui.gain) ui.gain.value = known ? String(data.effort_gain) : '';
    ui.summary.textContent = !data ? '状态未知' : !known ? '补偿状态未知' : active ? '补偿已启用'
      : enabled ? `已启用 · ${data.mode} 模式` : '补偿已关闭';
    ui.summary.className = `sum${active ? ' ok' : ''}`;
    ui.state.textContent = data
      ? `ros2_control ${data.model} · 模式 ${data.mode} · gravity_controller ${!known ? '未知' : enabled ? '已启用' : '已关闭'}`
        + ` · 实际增益 ${data.effort_gain ?? '未知'} · 输出 ${data.output_topic}`
      : 'ros2_control 状态未上报或已过期';
    const alarms = [];
    if (data && (data.model!==MODEL || data.model!==ui.model.value)) alarms.push(`控制模型 ${data.model} 与页面所选模型不一致`);
    if (data?.reject) alarms.push(`控制错误：${data.reject}`);
    if (enabled && data.mode!=='trajectory') alarms.push('重力未处于轨迹模式；请核对实际控制模式');
    const age = Date.now()-store.jointStates.last;
    if (active && (!store.jointStates.last || age<0 || age>1000)) alarms.push('/joint_states 缺少新鲜反馈，请核对控制器状态');
    if (store.bench.gravity_feedforward?.gain_error) alarms.push(store.bench.gravity_feedforward.gain_error);
    ui.alarm.hidden = !alarms.length;
    ui.alarm.textContent = alarms.join('；');
    ui.hint.textContent = '重力补偿由 ros2_control 与轨迹在同一周期执行；开关和增益以 /control/status 回执为准。全部软停会停用运动控制器，不自动恢复旧目标。';
  }
  async function request(operation, pending, success) {
    busy = true;setResult(pending);render();
    try { await operation();saveOptions();setResult(`${success}，等待 /control/status 确认`); }
    catch (error) { setResult(error.message, 'err'); }
    finally { busy = false;render(); }
  }
  ui.route.addEventListener('change', async () => {
    const enabled = ui.route.checked;
    const data = status();
    if (busy || !data || data.model!==MODEL || data.model!==ui.model.value) {render();return;}
    if (!enabled) {
      await request(()=>bench.stopGravityFeedforward(),'正在请求关闭补偿…','关闭请求已接受');
      return;
    }
    let gain;
    try { gain = readGain(); }
    catch (error) { setResult(error.message,'err');render();return; }
    const model = ui.model.value;
    await request(async () => {
      const joints = await modelJointsFor(model);
      if (status()?.model!==model || MODEL!==model) throw new Error('控制模型已更换，请重新确认');
      await bench.startGravityFeedforward({model,joint_map:jointMapSpec(joints),effort_gain:gain});
    },'正在请求开启补偿…','开启请求已接受');
  });
  ui.gain.addEventListener('change', async () => {
    const data = status();
    if (busy || !data?.gravity_enabled || data.model!==MODEL || data.model!==ui.model.value) {render();return;}
    let gain;
    try { gain = readGain(); }
    catch (error) { setResult(error.message,'err');return; }
    await request(()=>bench.setGravityFeedforwardGain(gain),'正在请求修改增益…','增益请求已接受');
  });
  ui.model.addEventListener('change',()=>{saveOptions();render();});
  ui.gain.value = saved.effort_gain!==undefined ? String(saved.effort_gain) : '1';
  async function loadModels() {
    try {
      const models = await availableModels();
      ui.model.replaceChildren();
      for (const name of models) {
        const option = document.createElement('option');option.value=name;option.textContent=name;ui.model.append(option);
      }
      ui.model.value = models.includes(saved.model) ? saved.model : MODEL;
    } catch (_error) {
      ui.model.replaceChildren();
      const option = document.createElement('option');option.value='';option.textContent='取不到模型列表';ui.model.append(option);
    }
    render();
  }
  render();loadModels();
  const timer = setInterval(render, 1000);
  return {render(dirty) {
    if ([DIRTY.BENCH,DIRTY.CONN,DIRTY.COMMAND,DIRTY.MODEL].some(key=>dirty.has(key))) render();
  },unmount(){clearInterval(timer);}};
}
