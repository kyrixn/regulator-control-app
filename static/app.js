// Front-end for the vc2 pneumatic station web app (len_control branch).
// Each of the 16 valve cells shows its muscle's LENGTH (mm, from the mapped
// sensor) as a bar and the regulator's PRESSURE as text. Each cell has two
// inputs: a PRESSURE box (kPa) and a target-LENGTH box (mm) that runs a PID
// (keyed by sensor id). Global HOLD stops all PID loops and holds pressure.

const NUM_VALVES = 16;
let MAX_VALUE = 140;          // valve setpoint limit (kPa), overwritten by server state

const socket = io();
const valveEls = [];          // index -> { input, lenInput, bar, press, len, cell, id, sensor, maxkpa }

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
      input.className = 'press-input';
      input.placeholder = 'kPa';
      input.title = 'Set pressure (kPa)';
      input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') submitSingle(i, input);
      });

      const lenInput = document.createElement('input');
      lenInput.type = 'text';
      lenInput.className = 'len-input';
      lenInput.placeholder = '→mm';
      lenInput.title = 'Set target length (mm) — runs PID';
      lenInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') submitLength(i);
      });

      cell.appendChild(barArea);
      cell.appendChild(id);
      cell.appendChild(input);
      cell.appendChild(lenInput);
      container.appendChild(cell);

      valveEls[i] = { input, lenInput, bar, label, press, len, cell, id,
                      sensor: null, maxkpa: MAX_VALUE };
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
  const value = parseFloat(t);
  if (Number.isNaN(value)) {
    logLine(`[ERR] V${valve}: invalid value '${input.value}'`);
    return;
  }
  const mx = valveEls[valve].maxkpa || MAX_VALUE;
  if (value > mx) {
    logLine(`[ERR] V${valve}: ${value} kPa exceeds limit (max ${mx} kPa)`);
    return;
  }
  socket.emit('set_valve', { valve, value, ramp });
  input.value = '';
}

// Set a target length (mm) for this cell's muscle — keyed by SENSOR id.
function submitLength(i) {
  const el = valveEls[i];
  const sensor = el.sensor;
  if (sensor === null || sensor === undefined) {
    logLine(`[ERR] V${i}: no mapped sensor for length control`);
    return;
  }
  const t = el.lenInput.value.trim().toLowerCase();
  if (t === '') return;
  if (t === 'off' || t === 'o' || t === 'x') {
    socket.emit('set_length', { sensor, length: 'off' });
    el.lenInput.value = '';
    return;
  }
  const mm = parseFloat(t);
  if (!Number.isFinite(mm)) {
    logLine(`[ERR] V${i}: invalid length '${el.lenInput.value}'`);
    return;
  }
  socket.emit('set_length', { sensor, length: mm });
  el.lenInput.value = '';
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
    const val = parseFloat(t);
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
document.getElementById('btnHold').addEventListener('click', () => socket.emit('hold'));
document.getElementById('btnApply').addEventListener('click', applyAll);
document.getElementById('btnStatus').addEventListener('click', () => socket.emit('status'));

// Sensor actions (shared controls)
document.getElementById('btnZero').addEventListener('click', () => socket.emit('zero'));
document.getElementById('btnClear').addEventListener('click', () => socket.emit('clear_zero'));
document.getElementById('btnRescan').addEventListener('click', () => socket.emit('rescan'));

// PID gains — apply Kp/Ki live.
const kpInput = document.getElementById('kpInput');
const kiInput = document.getElementById('kiInput');
let gainsInit = false;   // seed the fields once from the server, then leave them to the user
document.getElementById('btnGains').addEventListener('click', () => {
  const kp = parseFloat(kpInput.value);
  const ki = parseFloat(kiInput.value);
  const payload = {};
  if (Number.isFinite(kp)) payload.kp = kp;
  if (Number.isFinite(ki)) payload.ki = ki;
  if (Object.keys(payload).length) socket.emit('set_gains', payload);
});

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
      el.press.textContent = `${v.kpa.toFixed(1)} kPa`;
      el.cell.classList.add('on');
    } else {
      el.press.textContent = '–';
      el.cell.classList.remove('on');
    }

    // Length-control target box (keyed by the muscle's SENSOR id).
    const sensor = (m.sensor === undefined || m.sensor === null) ? null : m.sensor;
    el.sensor = sensor;
    el.maxkpa = m.max_kpa || MAX_VALUE;
    const mapped = sensor !== null;
    el.lenInput.disabled = !mapped;
    if (!mapped) {
      el.lenInput.placeholder = '—';
      el.cell.classList.remove('len-ctrl', 'len-ok');
    } else if (m.controlled) {
      const tgt = Number(m.target_mm).toFixed(2);
      el.lenInput.placeholder = `${m.at_target ? '✓' : '▶'} ${tgt}`;
      el.lenInput.title = `Length PID → ${tgt} mm (out ${m.output_kpa} kPa`
        + `${m.at_target ? ', on target' : ''}). Type a value to change, 'off' to release.`;
      el.cell.classList.add('len-ctrl');
      el.cell.classList.toggle('len-ok', !!m.at_target);
    } else {
      el.lenInput.placeholder = '→mm';
      el.lenInput.title = 'Set target length (mm) — runs PID';
      el.cell.classList.remove('len-ctrl', 'len-ok');
    }
  }

  // Seed the PID gain fields once from the server.
  const gains = (state.control && state.control.gains) || null;
  if (gains && !gainsInit) {
    if (kpInput.value === '') kpInput.value = gains.kp;
    if (kiInput.value === '') kiInput.value = gains.ki;
    gainsInit = true;
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
