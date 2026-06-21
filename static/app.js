// Front-end for the vc2 16-valve web controller.
// Mirrors the behaviour of vc2/vc2_gui.py over a SocketIO connection.

const NUM_VALVES = 16;
const Y_MAX = 10500;          // matches the matplotlib GUI y-limit (mV)
let MAX_VALUE = 4000;         // overwritten by server state

const socket = io();
const valveEls = [];          // index -> { input, bar, label, cell }

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
      barArea.appendChild(bar);
      barArea.appendChild(label);

      const id = document.createElement('div');
      id.className = 'valve-id';
      id.textContent = `V${i}`;

      const input = document.createElement('input');
      input.type = 'text';
      input.placeholder = '–';
      input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
          submitSingle(i, input);
        }
      });

      cell.appendChild(barArea);
      cell.appendChild(id);
      cell.appendChild(input);
      container.appendChild(cell);

      valveEls[i] = { input, bar, label, cell, id };
    }
  }
}

// ------------------------------------------------------------------
// Actions
// ------------------------------------------------------------------

// Ramp duration (seconds) from the TIME box; 0 (instant) if blank/invalid.
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

// ------------------------------------------------------------------
// Rendering
// ------------------------------------------------------------------
function renderState(state) {
  MAX_VALUE = state.max_value || MAX_VALUE;
  document.getElementById('maxHint').textContent = MAX_VALUE;

  const valves = state.valves || {};
  for (let i = 0; i < NUM_VALVES; i++) {
    const el = valveEls[i];
    const v = valves[String(i)];
    if (v) {
      const pct = Math.max(0, Math.min(100, (v.mV / Y_MAX) * 100));
      el.bar.style.height = pct + '%';
      el.label.textContent = `${v.bar.toFixed(2)} bar`;
      el.cell.classList.add('on');
    } else {
      el.bar.style.height = '0%';
      el.label.textContent = '';
      el.cell.classList.remove('on');
    }
  }

  const pill = document.getElementById('statusPill');
  const text = document.getElementById('statusText');
  if (!state.connected) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'DISCONNECTED';
  } else if (state.active > 0) {
    pill.className = 'status-pill active';
    text.textContent = `ACTIVE (${state.active})`;
  } else {
    pill.className = 'status-pill';
    text.textContent = 'IDLE';
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
socket.on('state', renderState);
socket.on('messages', (data) => (data.lines || []).forEach(logLine));
socket.on('connect', () => logLine('[web] connected to server'));
socket.on('disconnect', () => {
  document.getElementById('statusPill').className = 'status-pill disconnected';
  document.getElementById('statusText').textContent = 'SERVER LOST';
});

buildValves();
