# Pair sweep: friction dead band and settling of an antagonist pair

Date: 2026-10-01. Branch `hand_win`. Builds part C of the plan agreed on
2026-09-30 (logger and runner); homing follows once these numbers exist.

## Why

Playing with the wrist showed (user, 2026-09-30):

1. High static friction: a pressure pair (e.g. FCR 20 / ECR 30 kPa) holds the
   wrist at many non-extreme positions, so pressure is not a pose command.
2. The pair's length sum drifts over the first few antagonist sweeps and only
   then stays constant (beyond the radial-deviation part, since FCR and ECR
   both deviate radially).
3. Sup's thread sticks to its inner tube; only PT reads forearm rotation.

The homing procedure needs two numbers it cannot guess: the friction dead
band (sets trim step and tolerance) and how many warm-up sweeps the muscles
need before they settle. One protocol measures both.

## Protocol (one pair, one pretension)

- Pair (a, b), pretension P, amplitude A ≤ P, rate r (kPa/s), N cycles; other
  muscles held at given pressures.
- Inflate everything to its base pressure at 5 kPa/s, hold 5 s.
- Each cycle, with the pair's sum fixed at 2P (a = P + d, b = P − d):
  d 0 → +A, hold 2 s, → 0, hold 2 s ("zero from +"), → −A, hold 2 s, → 0,
  hold 3 s ("zero from −", the cycle's reference).
- Vent at 5 kPa/s, then switch the valves off.
- Defaults: P 30, A 15, r 1 kPa/s, N 6 → about 7.5 min per pair.

From one run:
- **Breakaway** (friction, kPa of differential): after each reversal, how far
  d moves before the pair's length moves more than 0.05 mm.
- **Gap at zero** (dead band in mm): length at "zero from +" minus at "zero
  from −" in the same cycle.
- **Settling**: the length vector at each cycle's reference; settled at the
  first cycle whose change from the previous one is under 0.1 mm for every
  logged sensor except Sup. Plus the pair's length sum per cycle.
- **Pressure readback** (optional, `--adc-hz`): the Giga's AD7606 volts logged
  next to the commanded pressure; a straight-line fit per regulator gives the
  readback scale that is still open.

## Software

- `station.py`: a `Station` holding the Giga and a `SensorHub` (polled
  back-to-back, ~20 Hz per sensor), a `Recorder` writing every reading,
  command, ADC sample and phase mark to one long-format CSV
  (`t, kind, id, value, note`), ramps and holds that check sensor health every
  tick, and abort on any problem: the Giga's stop command, everything vents.
  **Dry run by default**: sensors are read, nothing is sent. `--arm` sends.
- Hooks: `EncoderController.listeners` (every polled reading),
  `ValveController` parses the Giga's `a` (ADC) line and calls
  `adc_listeners`; `find_giga_port()` by VID 2341;
  `sensor_mapping.muscles_by_name()`.
- `experiments/pair_sweep.py`: builds the protocol from muscle names, prints
  the plan (duration, per-muscle pressure range), checks the sensors, and only
  with `--arm` runs it. Writes `runs/<stamp>_<a>-<b>_P<P>.csv` + `.json`.
- `experiments/analyze_sweep.py`: the numbers above as a table, plus PNGs:
  length against differential per cycle, and the settling curve.
- Limits: `--max-kpa` (default 50), A ≤ P, all pressures in [0, max-kpa].

## Tests (no hardware)

Protocol segments (sum constant, bounds), health checks, recorder format, ADC
line parsing, Giga port lookup, station ramps and aborts with fake devices,
and the analysis on synthetic stick-slip data.

## Not in this step

Homing (TEACH/PREVIEW/RUN), T2–T4, finger pairs, fixing Sup.
