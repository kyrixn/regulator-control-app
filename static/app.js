// Front-end for the vc2 pneumatic station web app (controller branch).
// Each of the 32 valve cells now shows its muscle's LENGTH (mm, from the mapped
// sensor) as a bar and the regulator's PRESSURE as text. The standalone encoder
// table is gone; sensors are zeroed per-cell or globally.
// A strip below lists every sensor (mapped or answering) with its port.

// Layout defaults; reconciled with the server's reported num_valves/row_size/
// bus_split so the grid follows the Arduino build (16 DACs -> 32 valves).
let NUM_VALVES = 32;
let ROW_SIZE = 8;
let BUS_SPLIT = 16;           // valves below this are on Wire, at/above on Wire2
let MAX_VALUE = 140;          // valve setpoint limit (kPa), overwritten by server state

const socket = io();
const valveEls = [];          // index -> { input, bar, label, cell, id, zeroBtn, sensor }

// ------------------------------------------------------------------
// Build the valve grid: ceil(NUM_VALVES / ROW_SIZE) rows of ROW_SIZE cells.
// ------------------------------------------------------------------
function busName(valve) {
  return valve < BUS_SPLIT ? 'Wire' : 'Wire2';
}

function buildValves() {
  const rows = document.getElementById('rows');
  rows.innerHTML = '';
  valveEls.length = 0;

  const numRows = Math.ceil(NUM_VALVES / ROW_SIZE);
  for (let row = 0; row < numRows; row++) {
    const lo = row * ROW_SIZE;
    const hi = Math.min(lo + ROW_SIZE - 1, NUM_VALVES - 1);

    const block = document.createElement('div');
    block.className = 'row-block';
    const title = document.createElement('h3');
    title.className = 'row-title';
    title.innerHTML = `Valves ${lo}–${hi} <span class="bus">(${busName(lo)})</span>`;
    const valveRow = document.createElement('div');
    valveRow.className = 'valve-row';
    block.appendChild(title);
    block.appendChild(valveRow);
    rows.appendChild(block);

    for (let i = lo; i <= hi; i++) {
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
      const vname = document.createElement('span');
      vname.textContent = `V${i}`;
      const mname = document.createElement('span');
      mname.className = 'muscle-name';
      id.append(vname, mname);

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
      valveRow.appendChild(cell);

      valveEls[i] = { input, bar, label, press, len, cell, id, mname, zeroBtn, sensor: null };
    }
  }
}

// Rebuild the grid if the server reports a different geometry than we drew.
function reconcileLayout(vstate) {
  const n = vstate.num_valves || NUM_VALVES;
  const rs = vstate.row_size || ROW_SIZE;
  const bs = (vstate.bus_split ?? BUS_SPLIT);
  if (n !== NUM_VALVES || rs !== ROW_SIZE || bs !== BUS_SPLIT) {
    NUM_VALVES = n;
    ROW_SIZE = rs;
    BUS_SPLIT = bs;
    buildValves();
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
  if (value > MAX_VALUE) {
    logLine(`[ERR] V${valve}: ${value} kPa exceeds limit (max ${MAX_VALUE} kPa)`);
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
document.getElementById('btnApply').addEventListener('click', applyAll);
document.getElementById('btnStatus').addEventListener('click', () => socket.emit('status'));

// Command box: one line to the server (kPa pairs, s, ?, p, a, i1/i0/i, ps <name>);
// Up/Down walk through the lines sent this session.
const cmdInput = document.getElementById('cmdInput');
const cmdHistory = [];
let cmdPos = 0;
cmdInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') {
    const text = cmdInput.value.trim();
    if (text === '') return;
    socket.emit('command', { text, ramp: getRamp() });
    if (cmdHistory[cmdHistory.length - 1] !== text) cmdHistory.push(text);
    cmdPos = cmdHistory.length;
    cmdInput.value = '';
  } else if (e.key === 'ArrowUp' && cmdPos > 0) {
    cmdInput.value = cmdHistory[--cmdPos];
    e.preventDefault();
  } else if (e.key === 'ArrowDown' && cmdPos < cmdHistory.length) {
    cmdPos++;
    cmdInput.value = cmdHistory[cmdPos] ?? '';
    e.preventDefault();
  }
});

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
  const maxMM = state.length_bar_max_mm ?? 10;

  const rangeHint = document.getElementById('rangeHint');
  if (rangeHint) {
    const bands = Object.entries(state.length_bar_bands_mm || {})
      .map(([region, [lo, hi]]) => `, ${region} ${lo}…${hi}`).join('');
    rangeHint.textContent = `${minMM}…${maxMM}${bands}`;
  }

  const pill = document.getElementById('valvePill');
  const text = document.getElementById('valveText');
  const panel = document.getElementById('valvePanel');

  if (vstate) {
    panel.classList.remove('disabled');
    reconcileLayout(vstate);
    MAX_VALUE = vstate.max_value || MAX_VALUE;
    document.getElementById('maxHint').textContent = MAX_VALUE;
  } else {
    panel.classList.add('disabled');
  }

  const valves = (vstate && vstate.valves) || {};
  for (let i = 0; i < NUM_VALVES; i++) {
    const el = valveEls[i];
    const m = muscles[i] || {};

    // Length bar from the mapped sensor's mm, scaled to the muscle's own band
    // (palmar muscles use a narrower one). The bar saturates outside the band,
    // so the exact length is shown as text.
    const lo = m.bar_min_mm ?? minMM;
    const hi = m.bar_max_mm ?? maxMM;
    const span = (hi - lo) || 1;
    const pos = m.position_mm;
    if (pos === null || pos === undefined) {
      el.bar.style.height = '0%';
      el.bar.classList.toggle('stale', true);
      el.len.textContent = '– mm';
    } else {
      const pct = Math.max(0, Math.min(100, ((pos - lo) / span) * 100));
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

    // Muscle driven by this regulator, next to the valve id.
    const muscle = m.muscle || '';
    el.mname.textContent = muscle;

    // Per-cell zero target.
    const sensor = (m.sensor === undefined) ? null : m.sensor;
    el.sensor = sensor;
    el.cell.title = muscle
      ? `V${i} · ${muscle} · ${sensor !== null ? `sensor ${sensor}` : 'no sensor'}`
      : `V${i} · not in the mapping`;
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
// Rendering — all sensors: every mapped id plus any other id that answers
// ------------------------------------------------------------------
function sensorChip(id, e, info, dup) {
  const chip = document.createElement('div');
  chip.className = 'chip';
  if (!info) chip.classList.add('unmapped');
  if (!e) chip.classList.add('missing');
  else if (!e.online) chip.classList.add('offline');
  if (dup) chip.classList.add('dup');

  const head = document.createElement('div');
  head.className = 'chip-head';
  head.textContent = `${id} ${info ? info.muscle : 'unmapped'}`;
  const sub = document.createElement('div');
  sub.className = 'chip-sub';
  const reg = info && info.regulator !== null ? `V${info.regulator}` : 'no V';
  sub.textContent = `${reg} · ${e ? e.port : 'not found'}`;
  const val = document.createElement('div');
  val.className = 'chip-val';
  if (!e) val.textContent = '—';
  else if (e.online && e.position_mm !== null) val.textContent = `${e.position_mm.toFixed(2)} mm`;
  else val.textContent = e.message || 'offline';

  chip.append(head, sub, val);
  chip.title = `${head.textContent} · ${sub.textContent} · ${val.textContent}`;
  return chip;
}

function renderSensors(state) {
  const enc = state.encoders;
  const index = state.sensor_index || {};
  const pill = document.getElementById('sensorPill');
  const text = document.getElementById('sensorText');
  const summary = document.getElementById('sensorSummary');
  const strip = document.getElementById('sensorStrip');

  const found = {};
  ((enc && enc.encoders) || []).forEach((e) => { found[e.slave] = e; });
  const ids = [...new Set([...Object.keys(found), ...Object.keys(index)].map(Number))]
    .sort((a, b) => a - b);
  const dups = new Set((enc && enc.duplicates) || []);

  strip.innerHTML = '';
  ids.forEach((id) => strip.appendChild(sensorChip(id, found[id], index[String(id)], dups.has(id))));

  const expected = Object.keys(index).length || ids.length;
  if (!enc) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'SENSORS OFF';
    summary.textContent = '';
    return;
  }
  if (!enc.connected) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'SENSORS LOST';
  } else {
    pill.className = enc.online > 0 ? 'status-pill active' : 'status-pill';
    text.textContent = `SENSORS ${enc.online}/${expected}`;
  }
  const perPort = (enc.ports || []).map((p) => `${p.port} ${p.online}`).join(' · ');
  summary.textContent = perPort + (dups.size ? ` · duplicate ids: ${[...dups].join(', ')}` : '');
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
socket.on('state', (state) => { renderValves(state); renderSensors(state); });
socket.on('messages', (data) => (data.lines || []).forEach(logLine));
socket.on('connect', () => logLine('[web] connected to server'));
socket.on('disconnect', () => {
  ['valvePill', 'sensorPill'].forEach((id) => {
    document.getElementById(id).className = 'status-pill disconnected';
  });
  document.getElementById('valveText').textContent = 'SERVER LOST';
  document.getElementById('sensorText').textContent = 'SERVER LOST';
});

buildValves();
