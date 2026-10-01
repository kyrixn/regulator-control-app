# hand_win: Windows bring-up of the 32-regulator, 32-sensor station

Date: 2026-09-29. Branch `hand_win`, cut from `origin/finger` (c8e93d2).

## Goal

Run the pneumatic station for the bionic hand from the Windows PC. The first
milestone is that everything is connected and set up, all 32 draw-wire
sensors read out live in the web app, and the user checks every channel by
hand-typed commands. No automatic actuation is part of this milestone.

## Decisions

| Topic | Decision |
|---|---|
| Giga firmware | `giga-regulator-control` `main` (9b93d81). Its firmware sources are identical to afa0e69, so no re-flash is needed if the boot banner reports `ADC: AD7606 up (4 board(s), 32 ch @ 200Hz)`. `compact6R` is the 6-regulator station and does not apply. |
| App branch | New `hand_win` off `origin/finger`. `controller` is an ancestor of `finger`, so this branch has all of `controller` plus the finger tools. `finger` itself stays untouched for P0. |
| PC | This Windows PC. Linux behaviour must keep working. |
| Python | py313, with the packages in `requirements.txt` (pyserial, Flask, Flask-SocketIO) installed on the user's explicit request. |
| Sensor bus | The 32 sensors are split over several EKU081 ports. |
| Sensor ids | Unique across all 32 sensors, so a sensor is identified by its id alone. |
| Pressure cap | New `--max-kpa` flag that can only lower the 140 kPa cap for a session. |

## Out of scope

- Automatic actuation, calibration sweeps, closed-loop control, logging for experiments.
- Reading the Giga's AD7606 pressures on the PC. The touchscreen tiles already show them.
- Changes to the Giga sketch or to `valve_controller.py` beyond the cap flag.

## 1. Windows setup

1. The user installs WCH's multi-port Windows driver, USBMSER.exe v2.2
   (2025-03-11, WHQL-signed, covers CH9344 and CH348):
   https://www.wch.cn/downloads/USBMSER_EXE.html. The old name CH9344SER.EXE
   now returns 404. Done 2026-09-29. Observed: COM7–COM14 are channels A–H,
   named `WCH USB-SERIAL Ch A (COM7)` … `Ch H (COM14)`, manufacturer
   `wch.cn`, on bus `USB2MCOM\USB55D9_COM_0n` (PID 55D9 = CH348). pyserial
   reports no VID/PID for them (`USB VID:PID=0000:0000`). The station uses
   channels A–C with about 11 sensors each (user, 2026-09-29; earlier A–D).
2. Giga check, read-only: open its COM port, read the banner, send `p`, `?`,
   `a`. Expect 16 DACs initialised, 4 ADC boards, all setpoints 0, and, with
   air off, every ADC channel near 1.67 V (0 kPa).
3. Install `requirements.txt` into py313.
4. A Windows COM port can only be open in one program at a time. The scan
   CLI, `app.py` and the Arduino IDE serial monitor must not run together on
   the same port.

## 2. App changes

**Port discovery (`encoder_controller.py`).** `list_ch9344_ports()` becomes
cross-platform: on Linux, keep the `/dev/ttyCH9344USB*` glob; on Windows,
return the COM ports whose manufacturer is `wch.cn` and whose description
contains `USB-SERIAL Ch`, sorted by channel letter. The WCH driver hides the
VID, so matching on VID 1A86 would find nothing. The Giga is still found by
VID 2341. Auto mode opens all eight channels, and E–H go idle after their
empty scan. `--rs485-ports` restricts the set; nothing in the code assumes a
number of sensors per channel.

**Sensor hub (new `sensor_hub.py`).**
- Owns one `EncoderController` per port, each with its own poll thread.
- Merges their readings into one table keyed by sensor id, and adds the port
  to each entry. The merged entries keep the existing `slave` and
  `position_mm` fields, so `_muscles_state` in `app.py` keeps working.
- An id found on two ports is listed as a duplicate and reported in the
  message log. The first port's reading is kept.
- `zero(id)` goes to the port that owns the id. `zero_all` and `clear_zero` go
  to every port.
- `get_state()` adds a per-port summary: port, connected, online count.
- The controllers are built through a factory argument, so tests can inject
  fakes.

**`EncoderController` changes.**
- RESCAN becomes a request (`threading.Event`) that the port's own poll
  thread carries out. Today `rescan()` runs a scan in the SocketIO handler
  thread while the poll thread reads the same port, so Modbus frames can
  interleave.
- A port where nothing answers stays idle until RESCAN, instead of rescanning
  every second.

**`app.py`.**
- `--rs485-ports` takes a comma list and defaults to all EKU081 ports.
  `--rs485-port` stays as a single-port alias.
- The `--ids` default becomes the sensor ids listed in `sensor_mapping.json`,
  and `--ids` overrides it. The wired ids are 50–86 and 173–175, so a fixed
  range such as 1–100 would miss the upper-arm sensors. With 32 ids the
  startup scan costs at most about 2 s per port at the 60 ms timeout, run in
  parallel across ports.
- `--max-kpa` sets the session cap. It must be > 0 and ≤ 140, and the app
  exits with an error otherwise. It reaches `ValveController` as an instance
  limit, and the web UI's max value follows it. `MAX_INPUT_KPA` stays 140,
  because `finger/config.py` tests against it.
- The zero, rescan and state events go through the hub.

**UI (`templates/index.html`, `static/app.js`, `static/style.css`).**
- A new "all sensors" strip lists every sensor that answers, sorted by id,
  with port, mm, an online dot and, if mapped, its regulator and muscle.
  Unmapped sensors are highlighted.
- The header shows `sensors online N` and a count per port.
- The regulator grid and the 5 Hz refresh are unchanged.

**Scan CLI (new `scan_sensors.py`).** Read-only; it never opens the Giga.
Built on `finger/rs485.py`.
- Phase 1: scan ids (default 1–247) on every EKU081 port in parallel. Print
  id → port, duplicates, and a shortfall against `--expect N`.
- Phase 2: back-to-back sweeps of the found sensors, one thread per port, for
  `--seconds` (default 10). Print per-sensor reads, timeouts, errors, latency
  (min/median/max), resting jitter in counts and mm, and the sweep rate per
  port.
- `--log` writes every sample to CSV.

**Mapping.** `sensor_mapping.json` is rewritten from the user's
`muscle_id.xlsx` (bionic_hand workspace, read 2026-09-29). Its `connected`
column says whether the sensor is wired (`nc` = not yet). In its regulator
column a `-` means no regulator yet; for example `8,9,10,-` gives FDP1–3 →
V8–V10 and leaves FDP4 without a regulator. Names drop the space: `FDP 1` →
`FDP1`.

Entry format: `{"muscle", "regulator", "sensor"}`, with two new cases:
- `regulator: null`: a sensor with no regulator yet (FDP4, FDS4, ED4, DI4, PI4).
- `sensor: null` plus a `note` giving the planned id: a regulator whose sensor
  is not wired (FPL, V11, sensor 58).

`sensor_mapping.py` keeps its regulator-keyed map and adds a sensor-keyed
index, so the "all sensors" strip can name sensor-only muscles. Muscles with
neither a sensor nor a regulator (EPB 59, APL 71, adductor 80, FPB 85) are
left out.

| Group | Muscle → regulator / sensor |
|---|---|
| Forearm flexors | FDP1 V8/50, FDP2 V9/51, FDP3 V10/52, FDP4 –/53; FDS1 V12/54, FDS2 V13/55, FDS3 V14/56, FDS4 –/57; FPL V11/(58 nc) |
| Forearm extensors | EPL V15/60; ED1 V16/61, ED2 V17/62, ED3 V18/63, ED4 –/64 |
| Forearm and wrist | PT V1/65, Sup V2/66, FCU V3/67, FCR V4/68, ECU V5/69, ECR V6/70 |
| Hand | DI1 V20/75, DI2 V21/76, DI3 V22/77, DI4 –/78; APB V23/79; PI1 V24/81, PI2 V25/82, PI3 V26/83, PI4 –/84; OP V27/86 |
| Upper arm | biceps brevis V29/173, biceps longus V30/174, brachialis V31/175 |

That is 32 wired sensors and 28 regulators with a muscle (27 with both); V0,
V7, V19 and V28 are unused.

`finger/config.py` looks up entries named ED/FDS/FDP/DI/PI, so the finger
tools stop loading with this mapping. They keep working on `finger`, and
adapting them (e.g. a `--finger N` option) is out of scope here.

**README.** Add a Windows section: driver, COM ports, py313, the
one-program-per-port rule, and the new flags.

## 3. Bring-up sequence (run by the user)

| Stage | Setup | Action | Pass |
|---|---|---|---|
| B1 Sensors | Air off, sensors powered, Giga not opened | `python scan_sensors.py --expect 32` | 32 unique ids, no duplicates, per-port counts as wired, about 0 timeouts in 10 s at rest. Keep the id → port table and the sweep rate. |
| B2 Giga | Air off | Banner, `p`, `?`, `a` | As in setup step 2. |
| B3 App, air off | Both USB links | `python app.py` | Giga connected; `sensors online 32` with the B1 per-port counts. Moving each finger and the wrist by hand changes the matching readings. Then ZERO ALL. |
| B4 Air on, manual | Supply set low; `python app.py --max-kpa 50` | One regulator at a time: low kPa with a 1–2 s ramp, then off | Only that regulator's touchscreen tile rises. One sensor in the strip moves; note its id and direction. |
| B5 Stops | Air on, a few channels pressurised | Web STOP, touchscreen E-STOP, then Ctrl-C of the app | All valves go off every time; on Ctrl-C the app sends `s` before exiting. |

In B3 and B4, every moving sensor and every regulator is checked against the
mapping table above; mismatches are fixed in `sensor_mapping.json` and in
`muscle_id.xlsx`. Results go in `docs/bringup/hand_win-bringup-log.md`.

## Testing

- The existing finger tests (`python -m unittest discover -s finger -p "test_*.py" -t .`) still pass.
- New unittest modules, no hardware needed:
  - port discovery, with a mocked `comports()` (Windows entries) and a
    patched glob (Linux);
  - hub merging, duplicate detection and zero routing, with fake
    controllers;
  - `--max-kpa` validation and the instance limit in `ValveController`.
- A dry run of `app.py --no-valves --ne` on Windows starts the web server
  without hardware.
- Hardware verification is stages B1–B5 above.

## Open items

- Which of channels A–C each sensor is on (about 11 per channel). B1 finds out.
- COM numbers follow the USB socket. After the adapter was re-plugged the same
  day, channels A–H became COM20, 16, 21, 15, 22, 19, 18, 17. The app and scan
  tool order channels by letter, so nothing needs editing.
- Giga check 2026-09-29, read-only (`p`, `?`, `a`) on COM8: PONG, all 32
  setpoints 0, and 32 ADC channels (so the 4-board firmware is on the board).
  But 31 channels read 0.47–0.50 V, grouped by ADC board, and C15 reads
  1.88 V. Setup step 2 expected about 1.67 V (0 kPa). Recheck with the
  regulators powered and the air off: either they were unpowered, or their
  monitor output does not use the command's −100…500 kPa scale.
- A first read-only poll on 2026-09-29, with wiring in progress, got no
  answer from any id on COM7–COM10. The ports opened and timed out normally.
- Adapting the finger tools to the full-hand mapping.
