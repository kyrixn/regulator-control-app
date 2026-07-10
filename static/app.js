// Front-end for the vc2 pneumatic station web app.
// Handles BOTH the valve regulator (Giga R1) and the RS-485 encoders,
// which arrive together in a single {valves, encoders} state message.

const NUM_VALVES = 16;
const Y_MAX = 10500;          // matches the matplotlib GUI y-limit (mV)
let MAX_VALUE = 4000;         // overwritten by server state

const socket = io();
const valveEls = [];          // index -> { input, bar, label, cell, id }

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
        if (e.key === 'Enter') submitSingle(i, input);
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

// ------------------------------------------------------------------
// Encoder actions
// ------------------------------------------------------------------
document.getElementById('btnZero').addEventListener('click', () => socket.emit('zero'));
document.getElementById('btnClear').addEventListener('click', () => socket.emit('clear_zero'));
document.getElementById('btnRescan').addEventListener('click', () => socket.emit('rescan'));

// ------------------------------------------------------------------
// Rendering — valves
// ------------------------------------------------------------------
function renderValves(vstate) {
  const pill = document.getElementById('valvePill');
  const text = document.getElementById('valveText');
  const panel = document.getElementById('valvePanel');

  if (!vstate) {
    panel.classList.add('disabled');
    pill.className = 'status-pill disconnected';
    text.textContent = 'VALVES OFF';
    return;
  }
  panel.classList.remove('disabled');
  MAX_VALUE = vstate.max_value || MAX_VALUE;
  document.getElementById('maxHint').textContent = MAX_VALUE;

  const valves = vstate.valves || {};
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

  if (!vstate.connected) {
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
// Rendering — encoders
// ------------------------------------------------------------------
function fmt(v) { return (v === null || v === undefined) ? '–' : v; }

function renderEncoders(estate) {
  const pill = document.getElementById('encPill');
  const text = document.getElementById('encText');
  const panel = document.getElementById('encPanel');
  const meta = document.getElementById('encMeta');
  const tbody = document.getElementById('encRows');

  if (!estate) {
    panel.classList.add('disabled');
    pill.className = 'status-pill disconnected';
    text.textContent = 'ENCODERS OFF';
    meta.textContent = 'RS-485 adapter not opened.';
    return;
  }
  panel.classList.remove('disabled');

  const range = estate.scanned_range || [];
  meta.textContent =
    `Port ${estate.port || '?'}  ·  ${estate.baudrate || '?'} 8${estate.parity || 'N'}1  ·  ` +
    `Scanned ${range[0]}–${range[1]}  ·  Counts/turn ${estate.counts_per_turn}  ·  ` +
    `Online ${estate.online || 0}  ·  Zeroed ${estate.zeroed || 0}`;

  const encoders = estate.encoders || [];
  if (!encoders.length) {
    tbody.innerHTML = '<tr class="empty"><td colspan="9">' +
      'No encoders found in the scanned slave-id range.</td></tr>';
  } else {
    tbody.innerHTML = encoders.map((e) => {
      const cls = e.online ? 'online' : 'offline';
      const stateTxt = e.online ? 'ONLINE' : 'OFFLINE';
      const zeroBadge = e.zeroed ? ' <span class="zbadge">Z</span>' : '';
      return `<tr class="${cls}">
        <td class="num id">${e.slave}</td>
        <td class="state">${stateTxt}${zeroBadge}</td>
        <td class="num pos">${fmt(e.display_position)}</td>
        <td class="num">${fmt(e.absolute_position)}</td>
        <td class="num">${fmt(e.turns)}</td>
        <td class="num">${fmt(e.speed)}</td>
        <td class="num">${fmt(e.status_code)}</td>
        <td class="num">${fmt(e.error_count)}</td>
        <td class="msg">${e.message || ''}</td>
      </tr>`;
    }).join('');
  }

  if ((estate.online || 0) > 0) {
    pill.className = 'status-pill active';
    text.textContent = `ENCODERS (${estate.online})`;
  } else {
    pill.className = 'status-pill';
    text.textContent = 'ENCODERS SCAN…';
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
socket.on('state', (state) => {
  renderValves(state.valves);
  renderEncoders(state.encoders);
});
socket.on('messages', (data) => (data.lines || []).forEach(logLine));
socket.on('connect', () => logLine('[web] connected to server'));
socket.on('disconnect', () => {
  for (const id of ['valvePill', 'encPill']) {
    document.getElementById(id).className = 'status-pill disconnected';
  }
  document.getElementById('valveText').textContent = 'SERVER LOST';
  document.getElementById('encText').textContent = 'SERVER LOST';
});

buildValves();
