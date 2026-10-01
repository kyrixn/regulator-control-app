# hand_win bring-up log

Stages as in `docs/superpowers/specs/2026-09-29-hand-win-system-bringup-design.md`.

## 2026-09-29 — setup

- EKU081 driver USBMSER.exe v2.2 installed. Channels A–H appear as
  `WCH USB-SERIAL Ch A` … `Ch H`; COM numbers follow the USB socket
  (A–D = COM20, COM16, COM21, COM15 at the time of the B1 run).
- Giga on COM8, read-only check: PONG, all 32 setpoints 0, 32 ADC channels
  (4-board firmware). ADC read 0.47–0.50 V on 31 channels and 1.88 V on C15,
  not the ~1.67 V expected at 0 kPa. Open: recheck with the regulators powered.
- Bus loading: every sensor board carries low-value resistors on A/B. With 10
  sensors on one channel, B–GND measured 28 Ω and A–GND 53 Ω (power off), and
  only 5 of 10 answered reliably. Fix: resistors removed from all but the last
  board of each channel, as far as needed for error-free reads.

## 2026-09-30 — B1 sensors: PASS

Upper-arm muscles (biceps brevis/longus, brachialis; V29–31, sensors 173–175)
dropped from the mapping: not part of the hand. 29 sensors expected.

`scan_sensors.py --expect 29 --seconds 60` (ids 1–247, all channels):

| Channel | Sensors | Sensor ids | Rate per sensor | Failed reads |
|---|---|---|---|---|
| A (COM20) | 7 | 50, 51, 53, 63, 64, 70, 75 | 20.0 Hz | 0 / 8386 |
| B (COM16) | 7 | 52, 55, 56, 62, 65, 77, 84 | 20.0 Hz | 0 / 8386 |
| C (COM21) | 8 | 57, 60, 66, 68, 76, 78, 81, 83 | 17.5 Hz | 0 / 8384 |
| D (COM15) | 7 | 54, 61, 67, 69, 79, 82, 86 | 20.0 Hz | 0 / 8386 |

- 29/29 found, no duplicates, no unexpected ids, E–H empty.
- 0 failed reads in 33 542 (no timeouts, no CRC errors); encoder status 0 on
  every sensor.
- Read latency: median 7.1 ms, max 12.4 ms. Sweep of 7 sensors: median 50 ms.
- Resting jitter at most 52 counts (0.0011 mm). DI4 (id 78) showed exactly 0
  counts of jitter: check in B3 that it moves.

## 2026-09-30 — B3 to B5: PASS (user report)

The user connected the airways and tested with air on and real actuation:
everything works. Per-stage notes (which sensor moved for which regulator,
stop checks, the pressure readback scale) were not recorded here.

Web UI after bring-up: each valve cell names its muscle; length bars use
−10…10 mm for forearm muscles and −4…4 mm for palmar ones
(`sensor_mapping.json` "region").
