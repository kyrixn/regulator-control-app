# vc2_webapp — valves + RS-485 encoders

> **Branch `rs485-encoder-reader`.** The `main` branch is valve-only. This
> branch drives **both serial devices of the station at once**: the Arduino
> Giga R1 valve regulator *and* a USB↔RS-485 adapter reading GJW encoders —
> two ports, one browser page.

The encoder half is a port of the `rs485-reader` project's terminal dashboard
(`gjw_encoder_dashboard.py`), which originally ran on **Windows** (COM ports +
`msvcrt` keys). Here it runs alongside the valve controller on Linux/Windows,
with the display and zeroing moved into the browser.

## Two devices, two ports

| Device                         | Role          | Enumerates as (this station)          |
|--------------------------------|---------------|---------------------------------------|
| Arduino Giga R1 (regulator)    | Valve control | `/dev/ttyACM0` — `Arduino Giga` (2341) |
| EKU081 8-port USB↔RS-485       | Encoder read  | `/dev/ttyCH9344USB0` … `USB7` (WCH CH9344) |

The adapter's vendor driver (`ch9344`) creates one node per port, so the
encoder bus is found **by port name**: the app takes the lowest-numbered
`ttyCH9344USB*` node. Those nodes come from an out-of-tree driver and are not
always listed by pyserial, so `/dev` is globbed as well. The Giga is still
matched **by device identity** (`Arduino`/`Giga`/`2341`), and CH9344 nodes are
excluded from valve detection. If no CH9344 node exists, detection falls back
to the old behaviour (known bridge chips WCH/FTDI/CP210x, then any other USB
serial port).

Pass `--valve-port` / `--rs485-port` to override — use `--rs485-port` if the
encoders are on a port other than `USB0`, e.g. `--rs485-port
/dev/ttyCH9344USB3` — and `--no-valves` / `--no-encoders` to run just one side.

If no `/dev/ttyCH9344USB*` nodes appear at all, the CH9344 kernel module isn't
loaded (check `lsmod | grep ch9344`; the WCH driver must be built/installed for
the running kernel).

## Features

> **Branch `len_control`.** Closed-loop length control. Each muscle can be
> driven either by **pressure** (mV) or by a **target length** (mm) via a
> conservative PID, joined to a sensor through `sensor_mapping.json`.

**Regulators + muscles** — each of the 16 cells shows its muscle's
**length (mm)** as the bar (from the mapped sensor, scaled to a configurable
band, default **−10…2 mm**) and that regulator's **pressure** as text. Each
cell has **two inputs**: a top **pressure** box (mV) and a bottom
**target-length** box (mm). APPLY ALL, STOP, ? STATUS, and TIME ramping are
unchanged.

**Length control (PID)** — type a target length (mm) in a cell's bottom box to
run a **conservative PI** loop (Kd=0 by default) that trims the regulator
pressure until the mapped sensor reads that length — tuned for **no
steady-state error**, slow response accepted. A **±0.1 mm deadband**
(`--tolerance`) stops it hunting once on target (the box turns green / shows
`✓`); the working range is only a few mm. Targets are keyed by **sensor id**
(the sensor lives on the muscle; the valve may be re-wired). Setting a pressure
on a cell cancels its length control. Muscle regulators are hard-capped at
**3000 mV** (PID output and manual commands); unmapped valves keep 4000.
Gains are live-tunable via the **PID** fields (**SET PID**) or `--kp/--ki/--kd`,
and the plant direction via `--pid-sign` (default `-1`: pressure ↑ shortens the
muscle).

- **HOLD** — stop every PID loop and hold the current pressures (also cancels
  ramps). Nothing vents.
- **STOP** — emergency vent (all valves off; also clears all PID targets).

**Sensor mapping** — `sensor_mapping.json` ties each regulator (valve id 0–15)
to the sensor (RS-485 slave id) measuring its muscle. Edit it to match wiring;
edits apply on the next **RESCAN** (no restart). Unmapped regulators show an
empty bar.

**Encoders (RS-485)** — a draw-wire setup: the encoder magnet rides a drum
(Ø **14 mm** by default) with a thread wound on it, so one turn pays out one
circumference of thread. The app scans slave ids (default **50–80**) for GJW
encoders and live-polls the ones that answer:

```
position_mm = (counts − zero) / counts_per_turn × π × drum_diameter
```

**ZERO ALL** zeros every online sensor (sets the length datum), **CLEAR ZERO**
returns to absolute, **RESCAN** re-sweeps the id range and reloads the mapping.
(The former per-cell zero button is replaced by the target-length box.)

## Install

```bash
pip install -r requirements.txt
```

## Run

```bash
python app.py                                        # auto-detect both ports
python app.py --valve-port /dev/ttyACM0 --rs485-port /dev/ttyCH9344USB0
python app.py --rs485-port /dev/ttyCH9344USB3 --no-valves  # encoders only
python app.py --ids 50-80 --baud 115200 --parity N
```

Then open <http://localhost:5000>.

### Options

| Option              | Default   | Description                                     |
|---------------------|-----------|-------------------------------------------------|
| `--valve-port`      | auto      | Giga R1 regulator port                          |
| `--rs485-port`      | auto      | EKU081 port (auto = lowest `ttyCH9344USB*`)     |
| `--no-valves`       | off       | Don't open the valve regulator                  |
| `--no-encoders`     | off       | Don't open the RS-485 encoder bus               |
| `--ids`             | `50-80`   | Encoder slave ids (ranges + lists: `50-80,90`)  |
| `--baud`            | `115200`  | RS-485 baud rate                                |
| `--parity`          | `N`       | RS-485 parity (`N`/`E`/`O`)                      |
| `--counts-per-turn` | `2097152` | Encoder single-turn resolution                  |
| `--drum-diameter`   | `14.0`    | Draw-wire drum diameter in mm (position scaling) |
| `--mapping`         | `sensor_mapping.json` | Path to the sensor↔regulator mapping JSON |
| `--timeout`         | `0.06`    | RS-485 per-read timeout (s)                      |
| `--interval`        | `0.2`     | Seconds between encoder poll cycles             |
| `--host`            | `0.0.0.0` | Web server bind host                            |
| `--http-port`       | `5000`    | Web server port                                 |

## Protocols

- **Valves** — Giga R1 running `vc2/vc2.ino` over USB serial @115200
  (`valve,value`, `s`, `?`, `p`, …). See `valve_controller.py`.
- **Encoders** — GJW Modbus RTU over RS-485: read-holding-registers (`0x03`)
  of 16 registers at `0x0380`;
  `absolute_position = turns × counts_per_turn + single_turn_position`.
  Framing/decoding in `modbus_rtu.py`, serial polling in `encoder_controller.py`.

## Project layout

```
vc2_webapp/  (branch: len_control)
├── app.py                  # Flask + SocketIO server, opens both ports
├── valve_controller.py     # Giga R1 valve serial layer (from main)
├── encoder_controller.py   # RS-485 encoder serial layer + poll thread
├── length_controller.py    # conservative PI length control (keyed by sensor)
├── modbus_rtu.py           # pure Modbus RTU + GJW decode (no hardware dep)
├── sensor_mapping.py       # loads sensor↔regulator mapping (tolerant)
├── sensor_mapping.json     # regulator → sensor map (user-editable)
├── requirements.txt
├── templates/index.html
└── static/{style.css, app.js}
```
