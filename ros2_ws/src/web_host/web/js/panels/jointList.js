// 关节索引（TF 侧）：一行一个关节——模型 URDF 的关节表 × /tf 的帧变换。
// 「关节」与「电机」不是一类东西：本面板不读固件的电机流；点击行 = 选中并置顶
// 该关节对应的电机（3D 小球与右栏电机状态表随之联动，行序共用 store.pinned）。
import { store, DIRTY, pinMotor, JOINT_OF_MOTOR } from '../core/store.js';
import { $, fmt, ageText } from '../core/format.js';
import { jointReading } from '../core/tf.js';

// 新鲜度阈值：动态 TF 超过 1.5 s 没更新算陈旧（sim 与真机都是 ≥10 Hz 级）。
const STALE_MS = 1500;

function motorOfJoint(name) {
  for (const [id, joint] of Object.entries(JOINT_OF_MOTOR)) {
    if (joint === name) return parseInt(id, 10);
  }
  return 0;
}

// 置顶优先（按关节对应的电机），其余保持模型里的顺序。
function orderedJoints() {
  return store.modelJoints
    .map((joint, i) => {
      const motor = motorOfJoint(joint.name);
      const at = store.pinned.indexOf(motor);
      return {joint, rank: at < 0 ? Infinity : at, i};
    })
    .sort((a, b) => (a.rank === b.rank ? a.i - b.i : a.rank - b.rank))
    .map((e) => e.joint);
}

function titleOf(joint, reading, motor) {
  const parts = [`关节 ${joint.name}（${joint.type}）`, `${joint.parent} → ${joint.child}`];
  if (motor) parts.push(`电机 m${motor}`);
  if (reading) {
    parts.push(`位置 ${fmt(reading.value, 4)} rad`);
    parts.push(reading.dynamic ? `TF 年龄 ${ageText(reading.ageMs)}` : 'TF 静态帧（/tf_static）');
  } else {
    parts.push('总线没有该帧对的 TF（/tf、/tf_static）');
  }
  return parts.join(' · ');
}

function rowHtml(joint, now) {
  const r = jointReading(joint, now);
  const motor = motorOfJoint(joint.name);
  const stale = !!r && r.dynamic && r.ageMs >= STALE_MS;
  const dot = !r ? 'var(--dim-2)' : (stale ? 'var(--warn)' : 'var(--ok)');
  const age = !r ? '无 TF' : (r.dynamic ? ageText(r.ageMs) : '静态');
  return `<li class="${motor && motor === store.selectedId ? 'on' : ''}"
      ${motor ? `data-id="${motor}"` : ''} title="${titleOf(joint, r, motor)}">
    <span class="jdot" style="background:${dot}"></span>
    <span class="jid">${joint.name}</span>
    <span class="jpos">${r ? fmt(r.value, 3) : '--'}</span>
    <span class="jstate${stale ? ' warn' : ''}">${age}</span>
  </li>`;
}

export function mount(root, ctx) {
  const list = $('jointRows');
  const summary = $('sumJoints');

  // 事件委托：列表每次渲染都整体重建。
  list.addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (!li) return;
    const id = parseInt(li.dataset.id, 10);
    pinMotor(id);
    ctx.ui.selectMotor(id);
  });

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.TF) || dirty.has(DIRTY.SELECTED))) return;
    const now = Date.now();
    const joints = orderedJoints();
    list.innerHTML = joints.length
      ? joints.map((j) => rowHtml(j, now)).join('')
      : '<li class="empty">等待模型 URDF…（关节表来自臂视图加载的模型）</li>';

    let fresh = 0;
    for (const j of joints) {
      const r = jointReading(j, now);
      if (r && (!r.dynamic || r.ageMs < STALE_MS)) fresh++;
    }
    summary.textContent = joints.length ? `${fresh}/${joints.length} TF` : '--';
  };

  render(new Set([DIRTY.TF, DIRTY.SELECTED]));
  // TF 停发后脏帧也停，年龄会冻在最后一次的值上——新鲜度指标冻结最误导，
  // 用 1 s 定时器补算（与 imu 面板的年龄刷新同一口径）。
  const ageTimer = setInterval(() => render(new Set([DIRTY.TF])), 1000);
  return {render, intervalMs: 200, unmount() { clearInterval(ageTimer); }};
}
