# vc2_webapp — valves + RS-485 encoders

> **Branch `hand_win`.** The full hand on Windows: the 32-regulator Giga plus
> 32 draw-wire sensors split over the EKU081's channels and read in parallel.
> Linux still works. Design:
> `docs/superpowers/specs/2026-09-29-hand-win-system-bringup-design.md`.
> The `main` branch is valve-only.

The encoder half is a port of the `rs485-reader` project's terminal dashboard
(`gjw_encoder_dashboard.py`), which originally ran on **Windows** (COM ports +
`msvcrt` keys). Here it runs alongside the valve controller on Linux/Windows,
with the display and zeroing moved into the browser.

## Two devices, two ports

| Device                         | Role          | Enumerates as (this station)          |
|--------------------------------|---------------|---------------------------------------|
| Arduino Giga R1 (regulator)    | Valve control | `/dev/ttyACM0` or `COMn` — VID 2341 |
| EKU081 8-port USB↔RS-485 (WCH CH348) | Sensor read | `/dev/ttyCH9344USB0` … `USB7`, or `WCH USB-SERIAL Ch A (COMn)` … `Ch H` |

The adapter's channels are found **by name**: on Linux the vendor driver's
`ttyCH9344USB*` nodes (out of tree and not always listed by pyserial, so `/dev`
is globbed as well), on Windows the COM ports named `WCH USB-SERIAL Ch X`,
channel A first. The app opens **every** channel, one poll thread each, and
merges them by sensor id (`sensor_hub.py`); channels with no sensors go idle.
The Giga is matched **by device identity** (`Arduino`/`Giga`/`2341`), and
EKU081 channels are excluded from valve detection. If no EKU081 is found,
detection falls back to a generic bridge chip (never the Giga or a Bluetooth
port).

Pass `--valve-port` / `--rs485-ports` to override (e.g. `--rs485-ports
/dev/ttyCH9344USB0,/dev/ttyCH9344USB1`), and `--no-valves` / `--no-encoders`
to run just one side.

If no `/dev/ttyCH9344USB*` nodes appear at all, the CH9344 kernel module isn't
loaded (check `lsmod | grep ch9344`; the WCH driver must be built/installed for
the running kernel).

## Features

> **Branch `controller`.** Groundwork for closed-loop length control. The
> standalone encoder table is gone; each regulator's bar now shows its muscle's
> **length**, joined to a sensor through `sensor_mapping.json`.

**Regulators + muscles** — the 32-cell grid (4 rows of 8: V0–15 on Wire, V16–31
on Wire2) is built from the server's reported `num_valves`, and each cell shows its
muscle's **length (mm)** as the bar (from the mapped sensor, scaled to a
configurable band, default **−10…2 mm**) and that regulator's **pressure in
kPa** as text (a dash `–` when the valve is off). Per-valve set boxes, APPLY
ALL, STOP, ? STATUS, and TIME ramping are unchanged.

**Units** — setpoints are entered in **kPa** (max 140 kPa, i.e. 4000 mV; decimals
allowed). The Giga sketch stays in its VOLTAGE (mV) mode: `valve_controller.py`
converts with `mV = (kPa + 100) × 1000 / 60` right before sending, so the
firmware, touchscreen and the desktop `vc2_*.py` scripts are untouched.

**Sensor mapping** — `sensor_mapping.json` ties each regulator (valve id 0–31)
to the sensor (RS-485 slave id) measuring its muscle. Edit it to match wiring;
edits apply on the next **RESCAN** (no restart). Unmapped regulators show an
empty bar.

**Encoders (RS-485)** — a draw-wire setup: the encoder magnet rides a drum
(Ø **14 mm** by default) with a thread wound on it, so one turn pays out one
circumference of thread. The app scans slave ids (default: **the sensor ids in
the mapping**) for GJW encoders on every channel and live-polls the ones that
answer:

```
position_mm = (counts − zero) / counts_per_turn × π × drum_diameter
```

**ZERO** (per cell) zeros just that sensor, **ZERO ALL** zeros every sensor,
**CLEAR ZERO** returns to absolute, **RESCAN** re-sweeps the id range and
reloads the mapping. Individual and global zero overwrite each other per
sensor (last write wins).

## Windows (hand_win)

1. **EKU081 driver.** Install WCH's `USBMSER.exe` (CH9344/CH348 multi-port
   driver): https://www.wch.cn/downloads/USBMSER_EXE.html. The channels appear
   as `WCH USB-SERIAL Ch A (COMn)` … `Ch H`. The app finds them by that name,
   because the driver hides the USB VID from pyserial. COM numbers change when
   the adapter moves to another USB socket; the channel letter does not.
2. **Python.** `C:\Users\14747\.conda\envs\py313\python.exe -m pip install -r requirements.txt`.
3. **One program per COM port.** Windows will not share a port: close the
   Arduino serial monitor, and stop `app.py` before running `scan_sensors.py`.
4. **Check the sensors first**, read-only (no valve I/O):

   ```
   python scan_sensors.py --expect 32
   python scan_sensors.py --ids 50-86,173-175 --seconds 60 --log runs/scan.csv
   ```

   It prints which ids answer on which channel, then per-sensor timeouts, CRC
   errors, latency, resting jitter, encoder status and the sweep rate per
   channel. It exits 1 if a sensor is missing or duplicated, or if any sensor
   fails a single read in the timing phase (`--max-error-rate` relaxes that).
   Being detected is not enough on a marginal bus; accept a channel only after
   a 60 s run with zero failed reads.
5. **Run the app.** `python app.py --max-kpa 50` for a first air-on session.
   Every EKU081 channel is opened and the empty ones go idle after their first
   scan; `--rs485-ports` limits the set.

`sensor_mapping.json` lists every muscle's regulator and sensor
(`"regulator": null` = sensor only, `"sensor": null` = regulator only). The app
scans exactly the mapped sensor ids unless `--ids` is given, and RESCAN reloads
the file and re-sweeps every channel. The **Sensors** panel shows every mapped
sensor plus any other id that answers, with its channel.

## Experiments (hand_win)

Scripted runs live in `experiments/` and use `station.py`: every sensor
reading, command, Giga pressure readback and phase mark goes to one CSV in
`runs/` (git-ignored). **Dry by default** (sensors read, nothing sent); `--arm`
sends. A sensor that stops answering, an error or Ctrl-C sends the Giga's
stop command, so every valve vents. Stop `app.py` first (and note that
stopping it vents every muscle).

```
python experiments/pair_sweep.py --pair FCR,ECR --hold FCU=30,ECU=30            # plan + sensor check
python experiments/pair_sweep.py --pair FCR,ECR --hold FCU=30,ECU=30 --arm --adc-hz 10
python experiments/analyze_sweep.py runs/<run>.csv
```

`pair_sweep.py` measures an antagonist pair's friction dead band (breakaway
after each turn, gap at zero) and how many sweeps it needs to settle; see
`docs/superpowers/specs/2026-10-01-pair-sweep-design.md`. The analysis also
fits the Giga's pressure readback scale when `--adc-hz` was used.

## Install

```bash
pip install -r requirements.txt
```

## Run

```bash
python app.py                                        # auto-detect everything
python app.py --max-kpa 50                           # lower the pressure cap
python app.py --valve-port /dev/ttyACM0 --rs485-ports /dev/ttyCH9344USB0
python app.py --no-valves                            # sensors only
python app.py --ids 50-86,173-175 --baud 115200 --parity N
```

Then open <http://localhost:5000>.

### Options

| Option              | Default   | Description                                     |
|---------------------|-----------|-------------------------------------------------|
| `--valve-port`      | auto      | Giga R1 regulator port                          |
| `--rs485-ports`     | auto      | Comma list of RS-485 ports (auto = every EKU081 channel) |
| `--rs485-port`      | –         | A single RS-485 port (alias of `--rs485-ports`) |
| `--no-valves`       | off       | Don't open the valve regulator                  |
| `--no-encoders`, `--ne` | off   | Don't open the RS-485 sensor bus                |
| `--max-kpa`         | `140`     | Session pressure cap in kPa; may only lower 140 |
| `--ids`             | mapping   | Sensor ids to scan (default: the sensor ids in `sensor_mapping.json`; ranges + lists: `50-86,173-175`) |
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
vc2_webapp/  (branch: hand_win)
├── app.py                  # Flask + SocketIO server, opens both devices
├── valve_controller.py     # Giga R1 valve serial layer (from main)
├── encoder_controller.py   # one RS-485 port: poll thread + EKU081 discovery
├── sensor_hub.py           # all RS-485 ports merged by sensor id
├── scan_sensors.py         # read-only scan + bus timing of every channel
├── modbus_rtu.py           # pure Modbus RTU + GJW decode (no hardware dep)
├── sensor_mapping.py       # loads the muscle/regulator/sensor mapping (tolerant)
├── sensor_mapping.json     # the hand's mapping (user-editable)
├── tests/                  # unittest, no hardware: python -m unittest discover -s tests -t .
├── finger/                 # single-finger tools (P0); not adapted to the full mapping
├── requirements.txt
├── templates/index.html
├── static/{style.css, app.js}
└── phase0_test/            # standalone Phase 0 characterisation (no web app)
    ├── Phase0test.py       # runs the hardware sequence, writes a CSV
    ├── phase0_analyze.py   # cycle-to-cycle repeatability plot
    ├── phase0_holds.py     # settled extremes vs ramp duration plot
    └── phase0_data/        # CSVs + PNGs (git-ignored)
```

Phase 0 scripts run from the repo root and resolve their own paths, so
`python phase0_test/Phase0test.py` works regardless of the working directory.
`Phase0test.py` talks to the hardware directly — stop `app.py` first, or the
serial ports will already be held.
