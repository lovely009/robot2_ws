// 曲线面板：canvas 2D 绘制 store.chartBuf；写入侧每通道最多保留 1500 样本。
import { store, DIRTY } from '../core/store.js';
import { fmt, CHART_COLORS } from '../core/format.js';

export const CHART_WINDOW_MIN_MS = 1000;
export const CHART_WINDOW_MAX_MS = 60000;
export const DEFAULT_CHART_WINDOW_MS = 10000;
export const DEFAULT_CHART_MOTOR_IDS = Object.freeze([1]);
export const CHART_CHANNELS = Object.freeze([
  Object.freeze({ key: 'position', label: '位置', unit: 'rad' }),
  Object.freeze({ key: 'velocity', label: '速度', unit: 'rad/s' }),
  Object.freeze({ key: 'effort', label: '力矩', unit: 'N·m' }),
  Object.freeze({ key: 'current', label: '电流', unit: 'A' }),
]);
const DEFAULT_CHANNEL_KEYS = Object.freeze(['position']);

const q = (root, selector) => root.querySelector(selector);
const qa = (root, selector) => [...root.querySelectorAll(selector)];

export function clampChartWindowMs(value) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return DEFAULT_CHART_WINDOW_MS;
  const seconds = Math.round(numeric / 1000);
  return Math.min(
    CHART_WINDOW_MAX_MS,
    Math.max(CHART_WINDOW_MIN_MS, seconds * 1000),
  );
}

export function nextChartWindowMs(currentMs, deltaY) {
  const current = clampChartWindowMs(currentMs);
  if (deltaY < 0) return clampChartWindowMs(current - 1000);
  if (deltaY > 0) return clampChartWindowMs(current + 1000);
  return current;
}

export function mergeMotorChoices(motorIds, selectedIds, requiredIds = DEFAULT_CHART_MOTOR_IDS) {
  const ids = new Set();
  for (const value of [...motorIds, ...selectedIds, ...requiredIds]) {
    const id = Number(value);
    if (Number.isInteger(id) && id > 0) ids.add(id);
  }
  return [...ids].sort((a, b) => a - b);
}

export function collectChartSeries(chartBuf, motorIds, channelKeys) {
  const ids = mergeMotorChoices(motorIds, [], [])
    .filter((id, index, all) => all.indexOf(id) === index);
  const wantedChannels = new Set(channelKeys);
  const channels = CHART_CHANNELS.filter((channel) => wantedChannels.has(channel.key));
  const series = [];
  for (const id of ids) {
    const buckets = chartBuf.get(id) || {};
    for (const channel of channels) {
      const bucket = buckets[channel.key];
      series.push({
        id,
        channel: channel.key,
        label: channel.label,
        unit: channel.unit,
        bucket: bucket && Array.isArray(bucket.t) && Array.isArray(bucket.values)
          ? bucket
          : null,
      });
    }
  }
  return series;
}

export function seriesLegendText(series) {
  return `电机 ${series.id} · ${series.label} (${series.unit})`;
}

function selectedChannelsFromDom(root) {
  return new Set(
    qa(root, 'input[name="chart-channel"]:checked').map((input) => input.value),
  );
}

function selectionSummary(motorIds, channels) {
  const motors = [...motorIds].sort((a, b) => a - b);
  const motorText = motors.length === 1
    ? `电机 ${motors[0]}`
    : `电机 ${motors.join(', ')}（${motors.length}）`;
  const channelText = channels.length === 1
    ? `${channels[0].label} ${channels[0].unit}`
    : `${channels.map((channel) => channel.label).join(' + ')}（${channels.length}）`;
  return { motorText, channelText };
}

function createMotorOption(id) {
  const label = document.createElement('label');
  label.className = 'curve-option';
  const input = document.createElement('input');
  input.type = 'checkbox';
  input.name = 'chart-motor';
  input.value = String(id);
  input.dataset.motorId = String(id);
  const text = document.createElement('span');
  text.textContent = `电机 ${id}`;
  label.append(input, text);
  return { label, input, text };
}

function reorderChildren(parent, desired) {
  for (let i = 0; i < desired.length; i++) {
    const current = parent.children[i];
    if (current !== desired[i]) parent.insertBefore(desired[i], current || null);
  }
}

export function mount(root) {
  const canvas = q(root, '#chart');
  const hint = q(root, '#chartHint');
  const legend = q(root, '#chartLegend');
  const pauseButton = q(root, '#btnChartPause');
  const clearButton = q(root, '#btnChartClear');
  const motorDetails = q(root, '#chartMotor');
  const channelDetails = q(root, '#chartCh');
  const motorOptions = q(root, '[data-chart-motor-options]');
  const motorSummary = q(root, '[data-chart-motor-summary]');
  const channelSummary = q(root, '[data-chart-channel-summary]');
  if (!canvas || !hint || !legend || !motorOptions || !motorSummary || !channelSummary) {
    return { render() {}, intervalMs: 200, unmount() {} };
  }

  const selectedMotors = new Set(DEFAULT_CHART_MOTOR_IDS);
  let selectedChannels = selectedChannelsFromDom(root);
  if (!selectedChannels.size) {
    selectedChannels = new Set(DEFAULT_CHANNEL_KEYS);
    const first = q(root, 'input[name="chart-channel"][value="position"]');
    if (first) first.checked = true;
  }

  // 已有模板节点也进入索引，后续只增删变化的行，不替换整个菜单。
  const motorChoiceNodes = new Map();
  for (const label of qa(motorOptions, 'label')) {
    const input = q(label, 'input[name="chart-motor"]');
    if (!input) continue;
    const id = Number(input.value);
    if (!Number.isInteger(id) || id <= 0) continue;
    motorChoiceNodes.set(id, {
      label,
      input,
      text: q(label, 'span') || label,
    });
  }
  let lastMotorChoiceKey = '';

  store.chart.windowMs = clampChartWindowMs(store.chart.windowMs);

  function orderedChannels() {
    return CHART_CHANNELS.filter((channel) => selectedChannels.has(channel.key));
  }

  function updateSummaries() {
    const { motorText, channelText } = selectionSummary(selectedMotors, orderedChannels());
    motorSummary.textContent = motorText;
    channelSummary.textContent = channelText;
    motorSummary.closest('summary')?.setAttribute('title', motorText);
    channelSummary.closest('summary')?.setAttribute('title', channelText);
  }

  function syncMotorChoices() {
    const choices = mergeMotorChoices(store.motors.keys(), selectedMotors);
    const key = choices.join(',');
    if (key === lastMotorChoiceKey) {
      for (const [id, entry] of motorChoiceNodes) entry.input.checked = selectedMotors.has(id);
      updateSummaries();
      return;
    }

    for (const id of choices) {
      let entry = motorChoiceNodes.get(id);
      if (!entry) {
        entry = createMotorOption(id);
        motorChoiceNodes.set(id, entry);
      }
      entry.text.textContent = `电机 ${id}`;
      entry.input.checked = selectedMotors.has(id);
    }
    // 保留已选电机，即使拓扑暂时消失也保留其历史曲线与复选状态。
    for (const [id, entry] of motorChoiceNodes) {
      if (!choices.includes(id)) {
        entry.label.remove();
        motorChoiceNodes.delete(id);
      }
    }
    reorderChildren(motorOptions, choices.map((id) => motorChoiceNodes.get(id).label));
    lastMotorChoiceKey = key;
    updateSummaries();
  }

  function renderLegend(entries) {
    legend.replaceChildren();
    for (const entry of entries) {
      const item = document.createElement('span');
      item.className = 'curve-legend-item';
      item.title = seriesLegendText(entry.series);
      const swatch = document.createElement('i');
      swatch.className = 'curve-legend-swatch';
      swatch.style.background = entry.color;
      const name = document.createElement('span');
      name.textContent = `电机 ${entry.series.id} · ${entry.series.label}`;
      const unit = document.createElement('span');
      unit.className = 'curve-unit';
      unit.textContent = entry.series.unit;
      item.append(swatch, name, unit);
      if (!entry.hasData) {
        const empty = document.createElement('span');
        empty.className = 'curve-legend-empty';
        empty.textContent = '无数据';
        item.append(empty);
      }
      legend.append(item);
    }
  }

  function drawChart() {
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const W = canvas.width;
    const H = canvas.height;
    const padL = 52;
    const padR = 10;
    const padT = 12;
    const padB = 22;
    const plotW = W - padL - padR;
    const plotH = H - padT - padB;
    const windowMs = clampChartWindowMs(store.chart.windowMs);
    store.chart.windowMs = windowMs;
    const now = Date.now();
    const start = now - windowMs;
    const channels = orderedChannels();
    const series = collectChartSeries(
      store.chartBuf,
      [...selectedMotors],
      channels.map((channel) => channel.key),
    );
    const entries = series.map((item, index) => {
      const points = [];
      if (item.bucket) {
        for (let i = 0; i < item.bucket.t.length; i++) {
          const t = item.bucket.t[i];
          const value = item.bucket.values[i];
          if (t >= start && t <= now && Number.isFinite(value)) points.push({ t, value });
        }
      }
      return {
        series: item,
        points,
        hasData: points.length > 0,
        color: CHART_COLORS[index % CHART_COLORS.length],
      };
    });
    renderLegend(entries);

    ctx.fillStyle = '#0a0f1e';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#1c2540';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = padT + plotH * i / 4;
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(W - padR, y);
      ctx.stroke();
    }
    for (let i = 0; i <= 6; i++) {
      const x = padL + plotW * i / 6;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + plotH);
      ctx.stroke();
    }

    let min = Infinity;
    let max = -Infinity;
    for (const entry of entries) {
      for (const point of entry.points) {
        if (point.value < min) min = point.value;
        if (point.value > max) max = point.value;
      }
    }

    ctx.font = '12px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillStyle = '#7e8bab';
    for (let i = 0; i <= 2; i++) {
      const x = padL + plotW * i / 2;
      const offset = Math.round(windowMs * (1 - i / 2) / 1000);
      ctx.fillText(`-${offset}s`, x, H - 6);
    }

    if (!entries.length || !Number.isFinite(min)) {
      ctx.textAlign = 'left';
      ctx.font = '12px sans-serif';
      ctx.fillStyle = '#7e8bab';
      ctx.fillText(
        entries.length ? '暂无数据（等待所选电机/通道的数据流）' : '请选择至少一台电机和一个通道',
        padL + 8,
        padT + 18,
      );
      hint.textContent = `${Math.round(windowMs / 1000)} s · ${entries.length} 个组合 · 滚轮调整 1–60 s`;
      return;
    }

    if (min === max) {
      min -= 1;
      max += 1;
    }
    const span = max - min;
    min -= span * 0.08;
    max += span * 0.08;
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
      const value = max - (max - min) * i / 4;
      ctx.fillText(fmt(value, 2), padL - 6, padT + plotH * i / 4 + 4);
    }

    const px = (t) => padL + (t - start) / windowMs * plotW;
    const py = (value) => padT + (max - value) / (max - min) * plotH;
    for (const entry of entries) {
      if (!entry.points.length) continue;
      ctx.strokeStyle = entry.color;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      entry.points.forEach((point, index) => {
        const x = px(point.t);
        const y = py(point.value);
        if (index) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
      });
      ctx.stroke();
    }

    const withData = entries.filter((entry) => entry.hasData).length;
    hint.textContent = `${Math.round(windowMs / 1000)} s · ${entries.length} 个组合`
      + ` · 范围 [${fmt(min, 2)}, ${fmt(max, 2)}] · 滚轮调整 1–60 s`;
    if (withData < entries.length) hint.textContent += ` · ${withData} 个有数据`;
    if (!store.chart.running) hint.textContent += ' · 已暂停';
  }

  function setWindow(ms) {
    store.chart.windowMs = clampChartWindowMs(ms);
    drawChart();
  }

  function onCanvasWheel(event) {
    event.preventDefault();
    setWindow(nextChartWindowMs(store.chart.windowMs, event.deltaY));
  }

  function onRootChange(event) {
    const input = event.target;
    if (!(input instanceof HTMLInputElement)) return;
    const numeric = Number(input.value);
    if (input.name === 'chart-motor') {
      if (input.checked) selectedMotors.add(numeric);
      else selectedMotors.delete(numeric);
      if (!selectedMotors.size) {
        selectedMotors.add(numeric);
        input.checked = true;
      }
      updateSummaries();
      drawChart();
      return;
    }
    if (input.name === 'chart-channel') {
      const next = selectedChannelsFromDom(root);
      if (!next.size) {
        next.add(input.value);
        input.checked = true;
      }
      selectedChannels = next;
      updateSummaries();
      drawChart();
    }
  }

  function onRootClick(event) {
    const target = event.target;
    if (!(target instanceof Element)) return;
    if (target.closest('#btnChartPause')) {
      store.chart.running = !store.chart.running;
      pauseButton.textContent = store.chart.running ? '暂停' : '继续';
      drawChart();
    } else if (target.closest('#btnChartClear')) {
      store.chartBuf.clear();
      drawChart();
    }
  }

  function closeMenus() {
    if (motorDetails) motorDetails.open = false;
    if (channelDetails) channelDetails.open = false;
  }

  function onDocumentPointerDown(event) {
    if (!(event.target instanceof Node)) return;
    if (motorDetails?.open && !motorDetails.contains(event.target)) motorDetails.open = false;
    if (channelDetails?.open && !channelDetails.contains(event.target)) channelDetails.open = false;
  }

  function onRootKeydown(event) {
    if (event.key === 'Escape' && (motorDetails?.open || channelDetails?.open)) {
      closeMenus();
      event.stopPropagation();
    }
  }

  pauseButton.textContent = store.chart.running ? '暂停' : '继续';
  canvas.addEventListener('wheel', onCanvasWheel, { passive: false });
  root.addEventListener('change', onRootChange);
  root.addEventListener('click', onRootClick);
  root.addEventListener('keydown', onRootKeydown);
  document.addEventListener('pointerdown', onDocumentPointerDown);
  const sizeObserver = new ResizeObserver(() => {
    const bounds = canvas.getBoundingClientRect();
    if (bounds.width < 1 || bounds.height < 1) return;
    const width = Math.round(bounds.width);
    const height = Math.round(bounds.height);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
      drawChart();
    }
  });
  sizeObserver.observe(canvas);

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY))) return;
    syncMotorChoices();
    drawChart();
  };
  render(new Set([DIRTY.MOTION, DIRTY.STATUS, DIRTY.TOPOLOGY]));
  return {
    render,
    intervalMs: 200,
    unmount() {
      sizeObserver.disconnect();
      canvas.removeEventListener('wheel', onCanvasWheel);
      root.removeEventListener('change', onRootChange);
      root.removeEventListener('click', onRootClick);
      root.removeEventListener('keydown', onRootKeydown);
      document.removeEventListener('pointerdown', onDocumentPointerDown);
    },
  };
}
