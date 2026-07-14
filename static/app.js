// Front-end for the vc2 pneumatic station web app (controller branch).
// Each of the 16 valve cells now shows its muscle's LENGTH (mm, from the mapped
// sensor) as a bar and the regulator's PRESSURE as text. The standalone encoder
// table is gone; sensors are zeroed per-cell or globally.

const NUM_VALVES = 16;
let MAX_VALUE = 4000;         // valve setpoint limit, overwritten by server state

const socket = io();
const valveEls = [];          // index -> { input, bar, label, cell, id, zeroBtn, sensor }

// ------------------------------------------------------------------
// Build the two rows of 8 valves
// ------------------------------------------------------------------
function buildValves() {
  for (let row = 0; row < 2; row++) {
    const container = document.getElementById(`row${row}`);
    const lo = row * 8;
    for (let i = lo; i < lo + 8; i++) {
      const cell = document.createElement('div');
      cell.className = 'valve';

      const barArea = document.createElement('div');
      barArea.className = 'bar-area';
      const bar = document.createElement('div');
      bar.className = 'bar';
      const label = document.createElement('div');
      label.className = 'bar-label';
      const press = document.createElement('div');
      press.className = 'press-val';
      const len = document.createElement('div');
      len.className = 'len-val';
      label.appendChild(press);
      label.appendChild(len);
      barArea.appendChild(bar);
      barArea.appendChild(label);

      const id = document.createElement('div');
      id.className = 'valve-id';
      id.textContent = `V${i}`;

      const input = document.createElement('input');
      input.type = 'text';
      input.placeholder = '–';
      input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') submitSingle(i, input);
      });

      const zeroBtn = document.createElement('button');
      zeroBtn.className = 'zero-btn';
      zeroBtn.textContent = 'Z';
      zeroBtn.title = 'Zero this sensor';
      zeroBtn.addEventListener('click', () => {
        const s = valveEls[i].sensor;
        if (s !== null && s !== undefined) socket.emit('zero_sensor', { slave: s });
      });

      cell.appendChild(barArea);
      cell.appendChild(id);
      cell.appendChild(input);
      cell.appendChild(zeroBtn);
      container.appendChild(cell);

      valveEls[i] = { input, bar, label, press, len, cell, id, zeroBtn, sensor: null };
    }
  }
}

// ------------------------------------------------------------------
// Valve actions
// ------------------------------------------------------------------
function getRamp() {
  const raw = document.getElementById('timeInput').value.trim();
  if (raw === '') return 0;
  const t = parseFloat(raw);
  return Number.isFinite(t) && t > 0 ? t : 0;
}

function submitSingle(valve, input) {
  const t = input.value.trim().toLowerCase();
  if (t === '') return;
  const ramp = getRamp();

  if (t === 'off' || t === 'o' || t === 'x') {
    socket.emit('set_valve', { valve, value: 'off', ramp });
    input.value = '';
    return;
  }
  const value = parseInt(t, 10);
  if (Number.isNaN(value)) {
    logLine(`[ERR] V${valve}: invalid value '${input.value}'`);
    return;
  }
  if (value > MAX_VALUE) {
    logLine(`[ERR] V${valve}: ${value} exceeds limit (max ${MAX_VALUE})`);
    return;
  }
  socket.emit('set_valve', { valve, value, ramp });
  input.value = '';
}

function applyAll() {
  const entries = [];
  const toClear = [];
  const invalid = [];

  for (let i = 0; i < NUM_VALVES; i++) {
    const raw = valveEls[i].input.value.trim();
    if (raw === '') continue;
    const t = raw.toLowerCase();
    if (t === 'off' || t === 'o' || t === 'x') {
      entries.push({ valve: i, value: 'off' });
      toClear.push(i);
      continue;
    }
    const val = parseInt(t, 10);
    if (Number.isNaN(val) || val > MAX_VALUE) {
      invalid.push(`V${i}='${raw}'`);
      continue;
    }
    entries.push({ valve: i, value: val });
    toClear.push(i);
  }

  if (invalid.length) {
    logLine(`[ERR] APPLY ALL aborted, invalid: ${invalid.join(', ')}`);
    return;
  }
  if (!entries.length) {
    logLine('[INFO] APPLY ALL: no values to apply');
    return;
  }
  socket.emit('apply_all', { entries, ramp: getRamp() });
  toClear.forEach((i) => { valveEls[i].input.value = ''; });
}

document.getElementById('btnStop').addEventListener('click', () => socket.emit('stop'));
document.getElementById('btnApply').addEventListener('click', applyAll);
document.getElementById('btnStatus').addEventListener('click', () => socket.emit('status'));

// Sensor actions (shared controls)
document.getElementById('btnZero').addEventListener('click', () => socket.emit('zero'));
document.getElementById('btnClear').addEventListener('click', () => socket.emit('clear_zero'));
document.getElementById('btnRescan').addEventListener('click', () => socket.emit('rescan'));

// ------------------------------------------------------------------
// Rendering — valves (length bar + pressure text) joined with sensors
// ------------------------------------------------------------------
function renderValves(state) {
  const vstate = state.valves;
  const muscles = state.muscles || [];
  const minMM = state.length_bar_min_mm ?? -10;
  const maxMM = state.length_bar_max_mm ?? 2;
  const span = (maxMM - minMM) || 1;

  const rangeHint = document.getElementById('rangeHint');
  if (rangeHint) rangeHint.textContent = `${minMM}…${maxMM}`;

  const pill = document.getElementById('valvePill');
  const text = document.getElementById('valveText');
  const panel = document.getElementById('valvePanel');

  if (vstate) {
    panel.classList.remove('disabled');
    MAX_VALUE = vstate.max_value || MAX_VALUE;
    document.getElementById('maxHint').textContent = MAX_VALUE;
  } else {
    panel.classList.add('disabled');
  }

  const valves = (vstate && vstate.valves) || {};
  for (let i = 0; i < NUM_VALVES; i++) {
    const el = valveEls[i];
    const m = muscles[i] || {};

    // Length bar from the mapped sensor's mm, scaled to [minMM, maxMM].
    // The bar saturates outside the band, so the exact length is shown as text.
    const pos = m.position_mm;
    if (pos === null || pos === undefined) {
      el.bar.style.height = '0%';
      el.bar.classList.toggle('stale', true);
      el.len.textContent = '– mm';
    } else {
      const pct = Math.max(0, Math.min(100, ((pos - minMM) / span) * 100));
      el.bar.style.height = pct + '%';
      el.bar.classList.toggle('stale', !m.sensor_online);
      el.len.textContent = `${pos.toFixed(2)} mm`;
    }
    el.len.classList.toggle('stale', !m.sensor_online);

    // Pressure text (dash when the valve has no active setpoint).
    const v = valves[String(i)];
    if (v) {
      el.press.textContent = `${v.bar.toFixed(2)} bar`;
      el.cell.classList.add('on');
    } else {
      el.press.textContent = '–';
      el.cell.classList.remove('on');
    }

    // Per-cell zero target.
    const sensor = (m.sensor === undefined) ? null : m.sensor;
    el.sensor = sensor;
    const mapped = sensor !== null;
    el.zeroBtn.disabled = !mapped;
    el.zeroBtn.title = mapped
      ? `Zero sensor ${sensor}${m.sensor_online ? '' : ' (offline)'}`
      : 'No sensor mapped';
  }

  if (!vstate) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'VALVES OFF';
  } else if (!vstate.connected) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'VALVES LOST';
  } else if (vstate.active > 0) {
    pill.className = 'status-pill active';
    text.textContent = `VALVES (${vstate.active})`;
  } else {
    pill.className = 'status-pill';
    text.textContent = 'VALVES IDLE';
  }
}

// ------------------------------------------------------------------
// Activity log
// ------------------------------------------------------------------
const logEl = document.getElementById('log');
const logLines = [];
function logLine(line) {
  logLines.push(line);
  while (logLines.length > 200) logLines.shift();
  logEl.textContent = logLines.join('\n');
  logEl.scrollTop = logEl.scrollHeight;
}

// ------------------------------------------------------------------
// Socket wiring
// ------------------------------------------------------------------
socket.on('state', (state) => renderValves(state));
socket.on('messages', (data) => (data.lines || []).forEach(logLine));
socket.on('connect', () => logLine('[web] connected to server'));
socket.on('disconnect', () => {
  const pill = document.getElementById('valvePill');
  pill.className = 'status-pill disconnected';
  document.getElementById('valveText').textContent = 'SERVER LOST';
});

buildValves();
