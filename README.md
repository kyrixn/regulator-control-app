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
| USB↔RS-485 adapter             | Encoder read  | `/dev/ttyACM1` — `USB Single Serial` (WCH 1A86) |

Both enumerate as `ttyACM*` here, so the app **auto-detects by device
identity** (VID / product string), not by port name — the Giga is matched by
`Arduino`/`Giga`/`2341`, the adapter by known bridge chips (WCH/FTDI/CP210x) or
a generic "USB serial". Pass `--valve-port` / `--rs485-port` to override, and
`--no-valves` / `--no-encoders` to run just one side.

## Features

**Valves (Giga R1)** — unchanged from `main`: live bar display of all 16
valves, per-valve set boxes, APPLY ALL, STOP, ? STATUS, and TIME ramping.

**Encoders (RS-485)** — scans slave ids (default **50–80**) for GJW encoders,
then live-polls the ones that answer, showing display position (relative to
zero), absolute multi-turn position, turns, speed, status and error count. **ZERO ALL** captures the current position as zero, **CLEAR ZERO**
returns to absolute, **RESCAN** re-sweeps the id range.

## Install

```bash
pip install -r requirements.txt
```

## Run

```bash
python app.py                                        # auto-detect both ports
python app.py --valve-port /dev/ttyACM0 --rs485-port /dev/ttyACM1
python app.py --rs485-port /dev/ttyACM1 --no-valves  # encoders only
python app.py --ids 50-80 --baud 115200 --parity N
```

Then open <http://localhost:5000>.

### Options

| Option              | Default   | Description                                     |
|---------------------|-----------|-------------------------------------------------|
| `--valve-port`      | auto      | Giga R1 regulator port                          |
| `--rs485-port`      | auto      | USB↔RS-485 adapter port                         |
| `--no-valves`       | off       | Don't open the valve regulator                  |
| `--no-encoders`     | off       | Don't open the RS-485 encoder bus               |
| `--ids`             | `50-80`   | Encoder slave ids (ranges + lists: `50-80,90`)  |
| `--baud`            | `115200`  | RS-485 baud rate                                |
| `--parity`          | `N`       | RS-485 parity (`N`/`E`/`O`)                      |
| `--counts-per-turn` | `2097152` | Encoder single-turn resolution                  |
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
vc2_webapp/  (branch: rs485-encoder-reader)
├── app.py                  # Flask + SocketIO server, opens both ports
├── valve_controller.py     # Giga R1 valve serial layer (from main)
├── encoder_controller.py   # RS-485 encoder serial layer + poll thread
├── modbus_rtu.py           # pure Modbus RTU + GJW decode (no hardware dep)
├── requirements.txt
├── templates/index.html
└── static/{style.css, app.js}
```
