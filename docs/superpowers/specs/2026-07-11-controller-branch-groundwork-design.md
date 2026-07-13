# Controller Branch — Groundwork Design

**Date:** 2026-07-11
**Branch:** `controller`
**Status:** Approved (design), pre-implementation

## Purpose

Lay the groundwork for **closed-loop length control** of the bionic hand. Each
*muscle* (tendon) is driven by one pneumatic *regulator* (valve `V0`–`V15`) and
its length is measured by one draw-wire *sensor* (GJW RS-485 encoder, slave id
50–80). The eventual goal is to command pressure to reach a target length.

**This branch does not implement the control algorithm.** It only:

1. Introduces a persistent sensor↔regulator mapping file.
2. Reshapes the web UI so each regulator shows its muscle's **length (mm)** as a
   bar and its **pressure** as text, removing the standalone sensor table.
3. Makes each sensor independently zeroable while keeping a rescan control.

Control logic (target length input, PID, etc.) is deliberately out of scope and
will build on this in a later step.

## Physical model

| Concept   | Hardware                        | Identifier        |
|-----------|---------------------------------|-------------------|
| Muscle    | Tendon / actuator               | label, e.g. `muscle_00` |
| Regulator | Pneumatic valve on the Giga R1  | valve id `0`–`15` |
| Sensor    | GJW draw-wire encoder on RS-485 | slave id `50`–`80` |

## 1. Sensor mapping file

New file `sensor_mapping.json` at the repo root — the join key between a
regulator and the sensor watching its muscle. Ships with 16 placeholder entries
(regulators 0–15) that the user edits to match wiring.

```json
{
  "_comment": "Maps each regulator (valve id) to the sensor (RS-485 slave id) measuring its muscle's length. Edit sensor ids to match wiring; use null to leave a regulator unmapped.",
  "muscles": [
    { "muscle": "muscle_00", "regulator": 0,  "sensor": 50 },
    { "muscle": "muscle_01", "regulator": 1,  "sensor": 51 },
    { "muscle": "muscle_02", "regulator": 2,  "sensor": 52 }
    // ... through regulator 15 ...
  ]
}
```

- `regulator` — valve id (0–15).
- `sensor` — RS-485 slave id, or `null` to leave a regulator unmapped.
- `muscle` — human-readable label (display/debug only for now).

The file is user-editable; edits take effect on the next RESCAN (see §4) without
a full server restart.

## 2. Architecture & data flow

The backend performs the join (recommended over shipping the raw mapping to the
browser and joining client-side — keeps the frontend a dumb renderer and keeps
the single source of truth server-side).

- `app.py` loads `sensor_mapping.json` once at startup, and re-reads it whenever
  a RESCAN is requested, so mapping edits apply without restarting the server.
- When assembling the combined `{valves, encoders}` state, `app.py` **augments
  each valve entry** with:
  - `sensor` — mapped slave id, or `null`.
  - `position_mm` — the mapped sensor's current mm reading, or `null` if the
    regulator is unmapped or its sensor is offline.
  - `sensor_online` — boolean.
- Length-bar scaling is governed by two module-level constants (configurable in
  code, **not** via CLI):

  ```python
  LENGTH_BAR_MIN_MM = -30.0
  LENGTH_BAR_MAX_MM = -10.0
  ```

  These are exposed in the state payload so the frontend scales bars correctly.

## 3. Backend changes

### `sensor_mapping.py` (new)
- `load_mapping(path) -> dict[int, dict]` returning `{valve_id: {"muscle": str,
  "sensor": int | None}}`.
- Tolerant of a missing or malformed file: logs a warning and returns an empty
  mapping (app still runs; all bars simply show no length).

### `encoder_controller.py`
- Add `zero(slave)` — capture the current absolute position of a **single**
  sensor as its display zero (independent per-sensor zeroing).
- Keep existing `zero_all` (global zero) and `clear_zero` (global clear).
- Per-sensor `zero(slave)` and global `zero_all` write into the same
  `zero_offsets` map, so they **overwrite each other per sensor, last write
  wins**: a global ZERO ALL captures offsets for every online sensor; a
  subsequent per-cell ZERO replaces just that sensor's offset, and another
  global ZERO ALL replaces them all again.

### `app.py`
- Load the mapping at startup; store it for state assembly.
- New SocketIO event `zero_sensor` with payload `{ slave: int }` → calls
  `encoder_controller.zero(slave)`. The existing `zero` event (global ZERO ALL,
  → `zero_all`) and `clear_zero` event (global CLEAR ZERO) are retained.
- On the existing `rescan` event, additionally re-read `sensor_mapping.json`.
- Build the joined state (per §2) and include `length_bar_min_mm`,
  `length_bar_max_mm`, and per-valve `sensor` / `position_mm` / `sensor_online`.

## 4. Frontend changes

### `templates/index.html`
- **Remove the entire Encoders `<section>`** (the independent sensor table) and
  its `encPill` header status pill.
- Each valve cell gains a small **ZERO** button (zeros only that cell's mapped
  sensor — the *individual* zero).
- The shared valve controls keep STOP / APPLY ALL / ? STATUS / TIME and gain
  **ZERO ALL** (global zero), **CLEAR ZERO** (global clear), and **RESCAN**.

### `static/app.js`
- `renderValves` now drives each bar from `position_mm` scaled to
  `[length_bar_min_mm, length_bar_max_mm]` instead of pressure/mV.
- The bar label becomes **pressure text**: `X.XX bar` when the valve is set, or
  a dash `–` when it is off/unset.
- Per-cell ZERO button emits `zero_sensor` with the cell's mapped `sensor` id;
  it is disabled/no-op for unmapped regulators.
- Wire ZERO ALL → `zero`, CLEAR ZERO → `clear_zero`, RESCAN → `rescan`.
- Delete `renderEncoders` and all encoder-table DOM references.

### `static/style.css`
- Restyle the bar to read as a length gauge (a mm value that can sit anywhere in
  the [−30, −10] band rather than a 0→max fill).
- Style the small per-cell ZERO button.

## Behavioral details

- **Length bar fill:** `pct = (position_mm − MIN) / (MAX − MIN) × 100`, clamped
  to `[0, 100]`. A `null` `position_mm` renders an empty bar.
- **Pressure text:** derived from the valve's commanded value as today
  (`mv_to_bar`); `–` when the valve has no active setpoint.
- **Unmapped regulator:** empty bar, `–` pressure until a setpoint is sent, ZERO
  button inert.
- **Zeroing precedence:** individual (per-cell) ZERO and global ZERO ALL both
  write offsets into the same map and overwrite each other per sensor, last
  write wins. CLEAR ZERO drops every offset (back to absolute mm).

## Out of scope (YAGNI)

- Closed-loop control algorithm (target length, PID, feedforward).
- Target-length input fields.
- Per-sensor bar ranges (single global range for now).
- CLI configuration of the bar range.

## Testing / verification

- App launches with `--no-valves` / `--no-encoders` and with neither device
  without crashing (mapping load must not hard-fail).
- Malformed / missing `sensor_mapping.json` → warning logged, app runs.
- With a mapped, online sensor: bar reflects mm, per-cell ZERO zeros only that
  sensor, CLEAR ZERO resets all, RESCAN re-reads the mapping.
- Encoder table is fully gone from the rendered page.
