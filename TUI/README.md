# RemoteRudder HMI — heading fusion, EKF, and instrumentation

This archive collects a multi-session engineering effort on the RemoteRudder
boat-autopilot HMI: a Raspberry Pi + PEAK PCAN node on a V-drive ski/wakeboard
boat (fixed prop, small rudder, steers only with forward way on), reading a
J1939 / NMEA 2000 marine bus at 250 kbit/s.

It's organized so you can (a) drop the code into your repo and go, and (b) keep
it as a record of how the work was done. If you're using this as a case study in
working with AI, the `SESSION_HISTORY.md` file is the narrative; this README is
the map.

## What was built

Starting from a working-but-basic autopilot HMI, the sessions added:

- **A curses TUI** with touch buttons, an 8-column CAN source table, and live
  diagnostics — usable on the boat's touchscreen.
- **J1939 NAME binding** — sources identified by their stable device identity
  (manufacturer/function/serial from the address-claim NAME), not their momentary
  bus address, so a heading source keeps working after any address change.
- **A GPS 24xd calibration tool** — a standalone on-water TUI that walks the
  Garmin basic-calibration procedure and enables the 24xd as a heading source
  when it completes.
- **COG-primary "fishing mode"** — above a speed threshold (4 mph, with
  hysteresis), the autopilot holds course *over ground* so the boat tracks
  straight in a crosswind and lines trail off the transom, rather than holding
  magnetic heading and drifting off track.
- **Yaw-rate limiting** — heading changes faster than a physically plausible
  turn (45°/s, measured from real logs) are discarded, so the controller never
  chases GPS jitter or a glitching sensor.
- **A heading EKF** — fuses the compass *derivative* (accurate even when the
  compass heading is biased) with COG and an optional gyro. 2-state normally,
  3-state once an independent gyro is available.
- **On-water instrumentation** — live web graphs (heading sources, sensor
  disagreement, rudder command, EKF state), and a filtered CAN recorder that
  logs only the ~17% of bus traffic the HMI actually uses, downloadable from the
  web page for later analysis.

Every feature is covered by tests: **544 passing** (plus 10 hardware tests that
skip without a real CAN interface).

## Layout

```
README.md                 <- you are here
SESSION_HISTORY.md        <- the narrative: what was decided and why, round by round
UPDATING_THE_REPO.md      <- how to get this into your git repo for sea trials
CHANGES.md                <- the running changelog (rounds 1-9)

code/                     <- all source modules
  app.py                  <- original web server (now with EKF + graphs + recorder)
  app_tui.py              <- curses TUI + Tornado server
  can_interface.py        <- CAN decode + read loop
  autopilot.py            <- steering control
  hmi_state.py            <- thread-safe shared state
  hmi_bridge.py           <- decode -> state, fusion (derive_fused), EKF stepping
  hmi_heading.py          <- heading quality / engage gate
  hmi_canlink.py          <- CAN backend + socketcan health
  hmi_tui.py              <- the curses UI
  heading_sources.py      <- NAME-bound source registry
  heading_fusion.py       <- yaw-rate limiter, stable-near-last fallback
  heading_ekf.py          <- 2-state and 3-state heading EKF
  j1939_name.py           <- address-claim / NAME decoding
  can_recorder.py         <- filtered CAN log recorder
  calibrate_compass_tui.py<- on-water 24xd calibration helper
  compare_ekf.py          <- EKF comparison plots
  visualize_fusion.py     <- fusion test-case plots
  templates/              <- web pages (autopilot.html, plot_data.html)

tests/                    <- 544 tests + real-capture fixtures
docs/                     <- PCAN hotplug setup
sensor_node/              <- Teensy heading-node sketch + CAN protocol
figures/                  <- generated comparison/visualization plots
round_packages/           <- the incremental delivery zips (rounds 3-9 + sensor node)
```

## Running it

Dependencies (on the Pi or a dev box):

```bash
pip install pytest pytest-cov pytest-randomly beautifulsoup4
# matplotlib only needed to regenerate the figures:
pip install matplotlib --break-system-packages
```

Run the tests:

```bash
cd code
python3 -m pytest -q         # 544 pass, 10 skip
```

Run the web server (with live graphs at /plot):

```bash
python3 app.py               # then open http://<pi>:5000/plot
```

Run the TUI:

```bash
python3 app_tui.py
```

Calibrate the 24xd on the water:

```bash
python3 calibrate_compass_tui.py --channel can0
```

## Where this is going

Next up is more sensor nodes (the `sensor_node/` Teensy design is the first),
which is exactly the case NAME binding and the source registry were built for —
a new heading source drops in by adding one profile, and the EKF gains an
independent gyro, which is when the 3-state filter earns its keep.

The sea trials come first. `UPDATING_THE_REPO.md` covers getting this into git
in a way that lets you correlate each on-water log with an exact code version.

## A note on running the tests from this archive

All logic tests pass standalone: `python3 -m pytest --ignore=tests/test_web.py`
→ 495 pass, 10 skip. The 18 web tests need the vendored front-end assets
(Plotly, Leaflet, map tiles, the service worker) under `static/`, which are not
included here to keep the archive small — they already live in your repo. Run
the full 544-test suite from your repo tree, where `static/` is present.
