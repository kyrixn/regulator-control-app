# vc2_webapp

Browser-based controller for the **vc2 16-valve pneumatic station**. It is a
web reimplementation of the matplotlib GUI (`vc2/vc2_gui.py`), providing the
same controls from any browser on the network.

The firmware (`vc2/vc2.ino`) runs on an **Arduino Giga R1**, which drives 16
GP8403 DACs (0–10 V) across two I2C buses:

- Valves **0–7** on `Wire` (SDA/SCL)
- Valves **8–15** on `Wire1` (SDA1/SCL1)

The web app talks to the Giga R1 over USB serial @115200 using the exact same
command protocol as the original Python scripts.

## Features

- Live bar display of all 16 valves (mV + converted bar pressure)
- Per-valve input box — type a value + **Enter** to fire that valve, or `off`
- **APPLY ALL** — fire every non-empty box in one batched serial command
- **TIME (s)** — ramp duration. Blank or `0` fires instantly (default
  behaviour); a positive value linearly ramps each commanded valve from its
  current value to the target over that many seconds. Applies to both single
  sets and APPLY ALL. Ramps are streamed as intermediate setpoints over serial
  (~25 Hz) from the backend, since the firmware sets the DAC directly.
- **STOP** — emergency stop (all valves off; cancels any ramps in progress)
- **? STATUS** — query all valve states
- Activity log of Arduino responses
- 4000 max value enforced (client + server), matching the original scripts

## Install

```bash
pip install -r requirements.txt
```

## Run

With the Giga R1 connected over USB:

```bash
python app.py                 # auto-detect serial port
python app.py /dev/ttyACM0    # explicit port
```

Then open <http://localhost:5000> (or the host machine's IP from another
device on the same network).

### Options

```
python app.py [port] [--host HOST] [--http-port PORT]
```

| Option        | Default   | Description                          |
|---------------|-----------|--------------------------------------|
| `port`        | auto      | Serial port (e.g. `/dev/ttyACM0`)    |
| `--host`      | `0.0.0.0` | Web server bind host                 |
| `--http-port` | `5000`    | Web server port                      |

## Command protocol (handled by the Giga R1 / `vc2.ino`)

| Command               | Meaning                          |
|-----------------------|----------------------------------|
| `valve,value`         | Set single valve (e.g. `0,3000`) |
| `v1,val1,v2,val2,...`  | Set multiple valves              |
| `valve,off`           | Turn off a valve                 |
| `s`                   | Emergency stop                   |
| `?`                   | Query status                     |
| `p`                   | Ping                             |

## Pressure conversion

The device maps 0–10 V to −100…500 kPa, so:

```
bar = ((mV / 1000) * 60 - 100) / 100   (clamped to >= 0)
```

## Project layout

```
vc2_webapp/
├── app.py                # Flask + SocketIO server
├── valve_controller.py   # serial layer (adapted from vc2/vc2_control.py)
├── requirements.txt
├── templates/index.html
└── static/{style.css, app.js}
```
