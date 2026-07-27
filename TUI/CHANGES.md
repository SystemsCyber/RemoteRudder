# HMI changes

Five bugs found by running the code against the real `candump` capture and a
live server. Each was verified failing before the fix and passing after.

Run `python3 verify.py` from this directory to re-check all of them (37 assertions).

---

## 1. Engage was permanently blocked — `hmi_heading.py`

The most serious one.

```python
sig = st.get_signal("fused_heading") or st.get_signal("compass_heading")
```

This reads like a fallback but never is one. Both signals are registered in
`SystemState.__init__`, and a dataclass instance is always truthy regardless of
whether `.value` is `None`. The `or` never reached its second operand.

Consequences, both permanent until the Kalman filter lands:

- `ok_to_engage()` returned `(False, "no valid heading")` with a perfect compass
- `update_state()` raised a standing `HEADING_INVALID` fault

Fixed by adding `best_heading_signal()`, which selects on `is_valid()` rather
than object truthiness, and using it in both places.

**Watch for this pattern elsewhere.** Any `get_signal(a) or get_signal(b)` is
the same bug.

## 2. NaN and inf poisoned the heading goal — `hmi_state.py`

`float('nan') % 360.0` is `nan`, and so is `float('inf') % 360.0`. Both passed
the `try/except (TypeError, ValueError)` and were stored as the goal.

A `nan` setpoint is silent and total: every comparison against it is `False`,
and it propagates straight into the PID error term. A malformed WebSocket
payload from the phone could do this while the autopilot was engaged.

`set_goal()` now rejects non-finite values with `math.isfinite` and logs a
`WARN` event naming the source.

## 3. Pitch decoded 200° off — `can_interface.py`

PGN 65256 pitch is J1939 offset-encoded: 1/128° per bit with a −200° offset.
The decoder read it as plain unsigned, so a level boat reported ~198.7°.

Verified against 1375 real frames from the Horsetooth capture:

| | min | max | mean |
|---|---|---|---|
| raw (as decoded) | 198.41 | 201.01 | 199.78 |
| with −200 offset | −1.59 | +1.01 | −0.22 |

The *scatter* was correct, which is why sea-state attribution in
`HeadingMonitor` happened to work anyway — but the displayed number was
meaningless.

Note `roll` is still never decoded. Nothing feeds `monitor.add_roll()`, so
tilt attribution currently runs on pitch alone.

## 4. Autopilot thread outlived shutdown — `autopilot.py`

`run()` looped on `while True:` and never checked `_stop_event`, which `stop()`
had been setting all along. Worse, `stop()` closed the CSV *before* the thread
stopped writing to it.

Every exit produced:

```
ValueError: I/O operation on closed file.
can.exceptions.CanOperationError: Cannot operate on a closed bus
```

Three changes:

- the loop now waits on `_stop_event` (also makes stop latency one cycle
  instead of a full 100 ms sleep)
- `stop()` joins the thread with a timeout before closing the log
- `broadcast_status_message` logs `CanOperationError` at DEBUG rather than
  ERROR-with-traceback, since a closed bus during shutdown or an unplugged
  adapter is expected; the link fault is reported separately by `CANLink`

## 5. Replay froze all freshness — `app_tui.py`

```python
self.state.mark_source(msg.arbitration_id, msg.timestamp or time.time())
```

A replayed candump carries its original capture timestamps — August 2025. Every
source therefore read as infinitely stale the moment it arrived, blanking the
source panel and the FUSED row in offline mode.

Freshness measures *arrival*, not *capture*. Now stamps `time.time()`.

This matters specifically because offline replay is the mode the filter gets
demonstrated in.

## Also: turn-rate aliasing guard — `hmi_heading.py`

`rate_dps()` differences the window endpoints, which is only valid if heading
moved less than half a turn between them. Replaying faster than real time broke
that assumption and produced −145°/s. Now returns `None` above 90°/s rather
than reporting something false. Display-only until the Kalman rate state
replaces it.

---

# Verification

## Against the real capture

`logs/candump-2025-08-06_16442_horsetooth_firstConstants.log`, 120k frames:

- 0 decode errors
- all 8 watched PGNs reporting
- `fused_heading` and `heading_sigma` populated on 100/100 health ticks

## The four failure modes you named

| Failure | What the HMI shows |
|---|---|
| Hardware not plugged in | `LINK_DOWN` + `interface can0 not present — adapter unplugged or 'ip link set can0 up type can bitrate 250000' not run`, retry with backoff (1→2→4→8→15→30 s). No `sys.exit()`. |
| Wrong baud rate | `NO_TRAFFIC` + `no frames in Ns since open — check bitrate (set 250000) and termination`. On Linux also `BUS_ERRORS` from rising kernel `berr-counter`, which is what actually distinguishes a wrong bitrate from a quiet bus. |
| Missing messages | Per-PGN age in the sources panel; `STALE` names which one, e.g. `vessel_heading(4.3s)`. Others keep flowing. |
| Noisy heading | Circular sigma over a 6 s window, attributed: high sigma *with* tilt → `sea state, slow down or head off the swell`; high sigma with flat attitude → `suspect compass`. Blocks engage above 6°. |

## Live web/TUI sync

- phone sets goal → `goal_source: web/127.0.0.1`, TUI shows `<-web/127.0.0.1`
- `359.5 + 1 → 0.5` (wraps correctly)
- malformed payloads (`"not json"`, `{"heading_goal":"abc"}`, `null`, unknown
  command) rejected without corrupting state or killing the server
- engage refused on the noisy replay heading, with the diagnosis as the reason

## TUI

Rendered at 80x24 and confirmed. All keys work: arrows ±1, Shift-arrows and
`,`/`.` ±10, up/down ±5, `Home` snaps to current heading, `e`/`d` engage,
`s` servo, `c` clear faults, `q` quit. A 40-column terminal shows the
too-small message instead of throwing.

---

# Two things to decide

**`SIGMA_ENGAGE_MAX = 6.0` will block engagement on your real data.** Observed
sigma on the Horsetooth capture is 10–15° with near-flat attitude — consistent
with the hard-iron signature from the filter work. Either raise the threshold,
or gate engagement on the filter's post-fit residual instead of raw compass
scatter once the KF is in. The second is the better answer; the first gets you
through a demo.

**Roll is never decoded.** `HeadingMonitor` accepts it and would use it, but
nothing calls `add_roll()`. Roll couples into heading harder than pitch at
Colorado's ~66° magnetic dip, so wiring it in would sharpen the sea-state vs.
bad-compass distinction considerably.

---

# Running it

```bash
python3 app_tui.py                      # live socketcan on can0
python3 app_tui.py --offline            # replay a candump on a virtual bus
python3 app_tui.py --no-tui             # server only, logs to stderr
python3 app_tui.py --channel can1 --bitrate 250000
python3 app_tui.py --offline --speed 10 # 10x replay
```

Windows: `pip install windows-curses`, then
`--backend pcan --channel PCAN_USBBUS1`. The kernel error-counter check is
Linux-only; on Windows it degrades to the no-traffic timeout heuristic, which
still catches a wrong bitrate, just less specifically.

With the TUI up, logs go to `logs/hmi_<timestamp>.log` rather than stderr —
curses and a log handler cannot share a terminal.

`/health` returns the full state snapshot as JSON, which is useful for a
watchdog or a second display.

The original `app.py` is untouched and still runs.

---

# Web interface fixes

Two bugs found by adding `tests/test_web.py`.

## 6. `/sw.js` returned 500 — `app_tui.py` *(pre-existing, also in `app.py`)*

The reported error. The route was

```python
(r"/sw.js", StaticFileHandler, {"path": ..., "default_filename": "sw.js"})
```

`StaticFileHandler.get()` takes a `path` positional argument that must come
from a capture group in the route regex. `r"/sw.js"` captures nothing, and
`default_filename` does not fill the gap — it only applies to directory
requests. Every request raised:

```
TypeError: StaticFileHandler.get() missing 1 required positional argument: 'path'
```

`plot_data.html` registers the service worker on page load, so this fired on
every visit to `/plot`. The service worker caches the offline map tiles, so the
practical effect was that offline tiles silently stopped working.

Fixed to `r"/(sw\.js)"`.

**The same bug is in your original `app.py` at line 217.** I did not modify
`app.py`, so apply the same change there if you keep using it.

## 7. `compass.js` hardcoded port 5000 — `static/js/compass.js` *(pre-existing)*

```javascript
const socket = new WebSocket('ws://' + location.hostname + ':5000/ws');
```

Running the server on any other port left the browser connecting to nothing.
No error banner, no visible failure — just a dashboard that never updated.
Invisible without the console open.

Now derives from `location.port`, and uses `wss://` when the page is served
over HTTPS (a page on HTTPS cannot open a `ws://` socket at all).
`plot_data.html` already did this correctly with `location.host`; `compass.js`
was the outlier.

The web test fixture runs on a random port specifically so this stays caught.

---

# Round 3: COG-primary steering, manual control, touch UI

Six operator-requested changes after on-water testing with the July 2026
captures. The compass finding drives most of it.

## The compass is stuck (drives items 5, and the whole COG-primary design)

In all three July 2026 captures the magnetic compass never reads above ~264
deg, while COG spans the full 0-360 in the same runs. The boat pointed every
direction; the compass did not follow. The magnetometer is stuck or badly
calibrated, so on this hull COG is the only trustworthy heading whenever the
boat is moving. `test_cog_primary.py::test_compass_ceiling_is_visible_in_data`
documents this so a future healthy-compass capture flags that the rationale
has changed.

## 8. COG-primary fusion -- `hmi_bridge.py`

`derive_fused` was compass-first; inverted to COG-first. New behaviour:

- moving with COG          -> lock to COG (`heading_source="COG"`, `cog_lock=True`)
- COG drops out at speed   -> brief bridge on the compass, but NO lock, so the
                              autopilot holds rather than chases the bad sensor
- below 1.6 mph (COG_MIN)  -> no usable heading: fused goes None, rudder centers
                              and waits for motion

This suits the hull: V-drive ski/wakeboard boat, fixed prop, small rudder,
steers only with forward way on. `heading_source` and `cog_lock` are new state
fields, surfaced in the snapshot, TUI status panel, and web payload.

## 9. Autopilot honors COG lock -- `autopilot.py`

- `set_cog_lock()`: when engaged without a lock, hold the last rudder command
  (do not center, do not disengage); re-acquire when COG returns
- no steerage way (heading None): center the rudder once and wait
- wired `set_heading()` into the health tick -- the autopilot's
  `current_heading` was never actually being fed before, so the control loop
  ran on a heading frozen at 0
- fixed several None-poisoning crashes that surfaced once real heading flowed:
  `heading_goal = current_heading` set the goal to None on disengage when there
  was no lock, which then crashed the CSV logger's `round()`

## 10. Engage seeds goal + requires lock -- `app_tui.py`

Engaging now always seeds the goal at the current heading (hold present course,
never turn toward a stale setpoint) and refuses without a COG lock, with a
clear reason. The heading-noise interlock still applies first.

## 11. Manual steering -- `app_tui.py`, `hmi_tui.py`

`.` and `,` step the rudder motor +-2 degrees (`MANUAL_STEP_DEGREES`), only
while disengaged. Auto-enables the servo so the step actually moves the motor.
Disengaged-only makes manual and autopilot mutually exclusive by construction:
a manual input can never fight the autopilot. This is the fallback for a lost
lock or a deteriorated sensor -- keep the boat under control by hand.

## 12. Key remap -- `hmi_tui.py`

| Key | Was | Now |
|-----|-----|-----|
| Up arrow | goal +5 | snap goal to current heading (was Home + Fn) |
| Home | snap | (retired) |
| `.` / `,` | goal +-10 | manual motor step R / L |
| `[` / `]` | manual rudder | goal +-10 |
| Down arrow | goal -5 | goal -5 (unchanged) |
| arrows L/R | goal +-1 | unchanged |

Up replaces Home because the operator's keyboard needed the Fn key for Home.

## 13. 120-wide layout + touch buttons -- `hmi_tui.py`

- three-panel top row at >=118 cols: HEADING, RUDDER/DRIVE, and a new STATUS
  panel showing the COG-lock state (green COG LOCK / amber COMPASS weak / red
  NO LOCK) plus engage and servo indicators
- touch button bar: STEP L/R, SNAP, ENGAGE, DISENG, SERVO, CLR, QUIT. Buttons
  reflect state and disable themselves to match the command rules (engage greys
  out without a lock; manual steps grey out while engaged), so the touchscreen
  can never do what the keyboard would refuse
- mouse/touch enabled via curses.mousemask; taps map to button spans recorded
  each frame
- removed the extra blank lines the operator reported; the events area now
  sizes to fill the gap so the button bar sits directly above the footer
- 80-col fallback unchanged: two panels, indicators in the rudder panel

## 14. PEAK adapter hotplug -- `docs/PCAN_HOTPLUG.md`

Config so `can0` comes up at 250000 automatically on plug-in: a
systemd-networkd `.network` file (bitrate + bus-off recovery), a udev rule
matching the `peak_usb` driver, an optional stable-name link file, and
verification steps that tie back to the HMI's link-health panel.

## Tests

New: `test_cog_primary.py` (11), `test_manual_control.py` (22),
`test_touch_buttons.py` (19). Updated `test_tui.py`, `test_tui_render.py`, and
`test_integration.py` for the new key map and COG-primary fusion. Two new real
fixtures: `planing_2026.log` (13-25 mph, 0% below COG threshold) and
`docking_2026.log` (86% below). Total 438 passing, 10 hardware-gated skips.

## Item 3 (offline 500) -- not reproduced

Could not reproduce a 500 on any route, online or offline; every route returned
200. The OSM tile URL in plot_data.html fails offline, but that is a
client-side tile fetch, not a server 500. Left for a captured traceback.

---

# Round 4: address claims, source identity, two-heading diagnosis

Triggered by the stationary-south logs: the compass appeared to bounce
179 -> 30 deg while the boat sat still, and the source table showed no compass
at all.

## The diagnosis (not what it looked like)

The two stationary logs cleared the Garmin compass. Source 0xF8
(PGN 127250, vessel heading) reads a rock-steady ~170 deg -- south -- across
both captures. It does not bounce.

The apparent bounce was fused_heading switching sources. Two heading-ish
sources disagree:
  - 0xF8 (Garmin compass): steady 170 deg, correct
  - 0x1C (Garmin GPS vehicle-direction, PGN 65256): a STALE ~60 deg course,
    because a stationary GPS reports garbage course-over-ground

With COG-primary fusion and the boat stopped, the display could flip between
them. What looked like a failing compass was the fusion hopping sources -- which
is exactly the transparency the source-table work now provides.

## 15. J1939 address-claim decoding -- j1939_name.py (new)

Decodes PGN 60928 (Address Claimed) NAMEs: manufacturer, function, vehicle
system, identity. Validated against the real claim in the logs: SA 0x1C is
manufacturer 114 (Garmin), industry 4 (Marine), identity 328706. Ships a
built-in marine manufacturer lookup (Garmin, Airmar, Simrad, Raymarine,
Maretron, Furuno, Lowrance, B&G, Victron, ...).

## 16. Expanded source table -- hmi_state.py, hmi_tui.py

The source table gains the requested columns: ID, DATA (last payload), NAME,
AGE, COUNT, CLASS (vehicle system), FUNCTION, MANUFACTURER, IDENTITY. The last
four come from address claims, joined to each source by its J1939 source
address (low byte), so every PGN from a node shows that node's identity.

Rendered as a full-width table at >=136 cols; falls back to the compact
6-column view below that. STALE/NEVER show as text in both views (not just
color -- caught by a test, and better for a color-blind operator anyway).

The panel height now scales with terminal rows so it never overwrites the
faults/events lines below it.

## 17. Request-address-claim command -- app_tui.py, hmi_tui.py

New command `request_addresses`: broadcasts a J1939 Request (PGN 59904) for the
Address Claimed PGN, to the global address 0xFF. Every node re-announces its
NAME. Wired to a REQ ADDR touch button and the `a` key.

This matters because some nodes -- the compass on 0xF8 among them -- do not
send claims on their own in a short window. The request pulls their identity on
demand. Until you press it, unclaimed sources show blank identity columns
rather than wrong ones.

## 18. Source renames -- hmi_bridge.py

`vessel_heading` -> `compass_F8` and `vehicle_dir` -> `gps_vehicle_dir`, so the
two heading sources are obviously distinct in the table. This is a decode-label
change only; the CAN IDs are unchanged.

## Tests

New: `test_address_claims.py` (23) -- NAME decode against the real claim, the
request-frame structure, source-table identity joins, and the two-heading
distinction. New fixture `stationary_south.log` preserves the real address
claims and both heading sources. Total 484 passing.

## Data note

The compass (0xF8) and non-Garmin nodes only claim on request, so the identity
columns populate fully only after REQ ADDR. The 8-column table needs >=136
cols; narrower terminals get the compact view.

---

# Round 5: GPS 24xd, NAME-bound heading sources, calibration TUI

You installed a Garmin GPS 24xd (heading + attitude + GPS in one unit) and
captured a log with all devices claiming. This round wires it in by NAME.

## Device roster (from the 2026-07-25 08:45 address claims)

| SA | Device | Identity | Transmits |
|----|--------|----------|-----------|
| 0x19 | GPS 24xd (new) | Garmin, Attitude/Compass, 1602535 | heading, attitude, mag-var, position, COG/SOG |
| 0x1C | Zero-Off GPS | Garmin, 328706 | position, COG/SOG, vehicle-dir |
| 0xF8 | Wall 3-axis compass | Garmin, Heading Sensor, 1039212 | heading only |
| 0x9B | Airmar | Navigation, 1026669 | GNSS |
| 0x00 | Garmin (industrial) | 666824 | -- |

## 19. The 375-degree heading bug -- can_interface.py

The uncalibrated 24xd sends the NMEA 2000 "data not available" sentinel
(0xFFFE) for heading. The decoder scaled it to a bogus 375.5 deg, which
poisoned the fused heading and drove the HEADING_NOISE / sig 61 faults in the
08:46 TUI screenshot. Now 0xFFFE and 0xFFFF are rejected.

## 20. Multi-source heading decode -- can_interface.py

Heading (PGN 127250) now arrives from two devices: the wall compass (0xF8) and
the 24xd (0x19), on different arbitration IDs. The decoder matches by PGN, not
a fixed ID, and tags each heading with its source address so the bridge can
apply source priority.

## 21. NAME-bound heading source registry -- heading_sources.py (new)

Heading sources are bound by stable J1939 NAME identity, not address, so they
keep working after any address change (arbitration on power-up, device swap).
Each source has a priority and an enabled flag. derive_fused consumes the
registry's choice:

  priority 1: GPS 24xd   (enabled=False until calibrated)
  priority 2: Wall compass (enabled=True)
  [priority 3: Heading node, commented out until that board is on the bus]

To change which source wins or turn one off: edit DEFAULT_PROFILES in
heading_sources.py, or call registry.set_enabled/set_priority at runtime. This
is the single place source selection lives. A stale primary falls back to the
next enabled source, then to COG, then to center-and-wait.

Fusion priority is now: heading node -> registry heading source (24xd/wall) ->
COG when moving -> compass bridge -> none.

## 22. Compass calibration TUI -- calibrate_compass_tui.py (new)

A standalone on-water tool that guides the 24xd basic calibration (the
no-chartplotter procedure from its manual: two slow circles, then straight-line
alignment at >= 4 mph) and confirms success by watching the heading come alive.
It finds the 24xd by NAME, shows live heading/speed/fix, walks the phases
(WAITING -> HEADING -> CIRCLING -> ALIGNING -> DONE), and on completion offers
to enable the 24xd in the registry so the main HMI picks it up. Runs live
(--channel can0) or against a log (--replay) for a dry run.

## 23. Manual step is encoder counts, not degrees -- app_tui.py

The steering module's shaft goal is in raw encoder counts (the screenshots show
goal 1475 = center, rudder 549/348 etc). Manual steps were labeled degrees and
sized 2; changed to MANUAL_STEP_COUNTS = 10, per the operator's request for
increments of 10 counts.

## Tests

New: test_heading_sources.py (17) -- NAME binding, address-change tracking,
priority/enable selection, sentinel rejection, and the multi-source fusion
hand-off (wall compass -> 24xd when enabled -> fallback when stale). Caught a
mutable-default-state bug in the registry (shared profile objects across
instances) before it shipped. Total 501 passing.

## Calibration workflow

1. On the water, run: python3 calibrate_compass_tui.py --channel can0
2. Follow the on-screen steps (circles, then straight line at >= 4 mph).
3. When it reads DONE, press E to enable the 24xd.
4. Restart the HMI -- the 24xd is now the primary heading source, and works
   at any speed including at rest.

---

# Round 6: COG-primary fishing mode, yaw-rate limiting, stable fallback

Three requirements for fishing in wind/waves while holding a straight track.

## The COG-vs-heading distinction (why this matters)

Magnetic heading and course-over-ground are different things. In a crosswind or
current the boat crabs -- it POINTS one way (heading) but TRAVELS another (COG).
For fishing, the lines must trail straight off the transom, which means holding
a straight TRACK -- COG -- not heading. Holding magnetic heading in a crosswind
would let the boat drift sideways off its track.

## 24. COG-primary at fishing speed -- hmi_bridge.py

New threshold COG_PRIMARY_SPEED_MPH = 3.0. At and above it, COG is the primary
heading, above magnetic sources -- the boat holds its track. Below it (but above
COG_MIN 1.6), magnetic heading is primary because low-speed COG is GPS noise.
Below COG_MIN, nothing (center and wait). Three clear bands.

To change the fishing behavior: adjust COG_PRIMARY_SPEED_MPH, or to always
prefer magnetic, raise it above any cruising speed.

## 25. Yaw-rate limiting -- heading_fusion.py (new)

Every fused heading passes through a yaw-rate limiter. A candidate implying a
turn faster than YAW_RATE_MAX_DPS = 45 deg/s is discarded, and the fusion HOLDS
the last good heading rather than chasing the bad value.

Measured basis (2026-07 logs): real turns top out at ~41 deg/s (wall compass,
all logs); COG jitters to hundreds of deg/s even at planing speed (~12% of COG
samples jump >45 deg/s). So 45 deg/s clips the impossible without touching real
sharp turns.

A deliberate source switch (compass -> COG handoff) is re-baselined, not
rejected -- the limit guards jumps WITHIN a source, not intentional handoffs. A
run of consecutive rejects also re-baselines, so the filter never wedges.

## 26. Stable-near-last fallback -- heading_fusion.py

When sources disagree, choose_stable_near_last() drops impossible jumps
(yaw-rate gate), then among survivors prefers the one closest to the last
accepted heading that is also steady (low scatter). A single glitching sensor
cannot yank the controller off a good track. SourceStability tracks each
source's recent scatter over a 2 s window.

## Visualizations

visualize_fusion.py generates two figures (in outputs):
  * fusion_test_cases.png -- the fishing crosswind case (COG holds track while
    the boat crabs), yaw-rate rejections marked, and the implied-yaw-rate panel
    showing raw COG spiking to 100s of deg/s while fused stays under the limit
  * fusion_sensor_disagreement.png -- three sources with one glitching for ~15 s;
    the fused output stays on the stable sources throughout

The synthetic signals are tuned to the measured real-log noise (wall-compass
steadiness, COG spike rate) so the noise is realistic.

## Tests

New: test_heading_fusion.py (17) -- yaw-rate limiter (plausible turns accepted,
impossible jumps rejected, wraparound, re-baseline), the COG-primary transition
(magnetic below threshold, COG above), source-switch handoff, and stable
fallback. Caught and drove the fix for the source-switch-vs-glitch distinction.
Total 518 passing.

## The predicted 24xd behavior

Once calibrated, the 24xd should send a clean magnetic heading (like the wall
compass but better sited). The fusion treats it as a magnetic source: primary
below fishing speed, yielding to COG above it. Its attitude output (127257)
additionally feeds the sea-state-vs-sensor noise attribution. If its heading
proves noisy after calibration, the yaw-rate limiter and stable fallback
protect the controller regardless.

---

# Round 7: heading EKF using the compass derivative

The operator's idea: a compass with a constant bias (declination, hard-iron,
electrical) still has an accurate DERIVATIVE -- a real 10-degree yaw still moves
the compass ~10 degrees. So the compass yaw RATE is a good filter input even
when the compass heading is not.

## The idea is right, with one caveat

Confirmed. But not all compass error is constant bias:
  * declination -> constant, differentiates away cleanly (idea works perfectly)
  * hard-iron   -> becomes a heading-dependent error after the heading calc;
                   small for small yaw steps
  * soft-iron   -> SCALES the derivative (a real 10-deg turn reads 8 or 13 deg
                   depending on heading) -- this is the one that bites
  * electrical  -> transient spikes, not bias

So the compass yaw rate is a good measurement AFTER soft-iron correction (the
24xd calibration) and WITH transient spikes gated (the yaw-rate limiter from
round 6). Both already exist.

## 27. Heading EKF -- heading_ekf.py (new)

Two variants:
  HeadingEKF2  state = [heading, yaw_rate]
  HeadingEKF3  state = [heading, yaw_rate, yaw_bias]

Both fuse: compass heading (high noise -- its bias lives here), compass yaw rate
(low noise -- the operator's insight), COG (unbiased track reference, gated on
speed), and an optional independent gyro (node/24xd). Pure Python, no numpy in
the runtime path.

## 28. Which variant -- the comparison decided it

compare_ekf.py runs both against a synthetic fishing scenario with realistic
noise (compass +32-deg bias, 1.15x soft-iron scale, spikes; COG jitter). Result
(RMS error vs true track):

  priority chain: 2.53 deg
  EKF 2-state:    2.33 deg   <- best with compass derivative only
  EKF 3-state:    2.88 deg   <- WORSE without an independent gyro

The 3-state's yaw-bias is UNOBSERVABLE with only a compass derivative, so the
extra state just adds noise. But add an independent gyro (the 24xd attitude
output, or the Teensy node) and the 3-state drops to 0.77 deg RMS -- best by far,
because the gyro makes the bias observable.

Decision, encoded in the bridge: use the 2-state unless a gyro (node_yaw_rate)
is present, then the 3-state. The plot ekf_comparison.png shows all of this,
including the yaw-rate tracking (panel 3) and the bias learning (panel 4).

## 29. EKF runs alongside the priority chain -- hmi_bridge.py

The EKF is a NEW source, not a replacement. step_ekf() advances it each health
tick and publishes ekf_heading/ekf_yaw_rate/ekf_sigma. derive_fused prefers the
EKF when use_ekf is on AND its sigma is low (< 15 deg); otherwise it falls
straight through to the round-6 priority chain. So enabling the EKF cannot break
the working fusion -- if it diverges, the chain takes over.

OFF by default (use_ekf=False). Enable by setting bridge.use_ekf=True (a runtime
toggle / TUI control can be added).

## Tests

New: test_heading_ekf.py (13) -- both variants track a known turn, the yaw rate
stays accurate despite a heading bias (the core idea), the 3-state learns bias
only with a gyro, the EKF is off by default, and it runs alongside the chain
without breaking it. Total 531 passing.

## Recommendation

Run the 2-state now (compass derivative + COG). Once the 24xd is calibrated and
its attitude yaw rate is decoded, switch to the 3-state -- that is when the
operator's idea pays off most, because the gyro lets the filter learn and
subtract the compass's residual scale error online.

---

# Round 8: COG-primary threshold to 4 mph, with hysteresis

Operator note: the boat at idle often exceeds 3 mph, so a 3 mph transition
threshold flips between magnetic and COG during normal low-speed operation. The
magnetic source works fine at those speeds (proven in past use), so the handoff
should happen higher and stickier.

## 30. Threshold raised to 4 mph -- hmi_bridge.py

COG_PRIMARY_SPEED_MPH: 3.0 -> 4.0. Above idle, so trolling/idle stays on the
magnetic source and only hands off to COG once genuinely underway. Also matches
the Garmin 24xd's own 4 mph heading-alignment threshold.

## 31. Hysteresis dead-band -- hmi_bridge.py

A single threshold still chatters when the boat hovers right at it. Added
COG_PRIMARY_DROP_MPH = 3.0: COG-primary engages at 4 mph but does not drop back
to magnetic until below 3 mph. The 1 mph dead-band means a boat wobbling near
the threshold does not flip sources every time SOG crosses a line. Verified: an
idle-wobble SOG sequence that would chatter on a single threshold produces at
most one transition.

State: bridge._cog_primary_active, set at 4 mph, cleared below 3 mph.

## Tests

Added TestCogPrimaryHysteresis (5) to test_heading_fusion.py: engage at the
upper threshold, hold through the dead-band, drop below the lower threshold, no
chatter at idle, and the threshold ordering. Total 536 passing.

---

# Round 9: on-water web graphs, filtered CAN log, TUI EKF display

Three features for use on the water, where you can't upload a log to analyze.

## 32. Filtered CAN log recorder -- can_recorder.py (new)

Records ONLY the CAN frames the HMI decodes (steering, rudder, heading from any
source, COG/SOG/position, engine, autopilot status, heading node) plus address
claims (needed to rebuild the NAME registry on replay). On the real log this is
~17% of the bus -- small enough to share, and exactly what the fusion sees.
candump format, so the existing replay path reads it directly. Rotates at 20 MB.

Wired into the CAN read loop via a new raw-message listener hook
(can_interface.add_raw_listener), separate from decoded listeners so it also
captures address claims that process_message does not decode.

## 33. Web: live graphs + log download -- app.py, templates/plot_data.html

The /plot page gains:
  * EKF trace on the angles chart (compass/heading/COG/EKF/goal together)
  * a sensor-disagreement chart (COG-compass, EKF-compass, EKF-COG deltas) --
    see at a glance when sources diverge
  * a rudder/steering chart (steering goal counts, rudder angle, heading error)
  * an EKF yaw-rate + sigma chart
  * a status bar: active source (COG/COMPASS), EKF sigma, and the filtered-log
    path with a download link (/used-can-log)

app.py now runs the heading EKF on its own CAN interface (speed-weighted COG
noise, no hard threshold -- the EKF blends), folds EKF outputs + the active
source + the log path into the telemetry snapshot, and serves the log for
download. Also fixed the /sw.js route (the dot was unescaped, same bug as the
TUI server had).

## 34. TUI EKF strip -- hmi_tui.py

A one-line EKF summary between the top panels and the sources table, shown when
the EKF is producing output:

  EKF  H  98.7  rate +5.4/s  sig 0.7  [2st]  in:CG-  rej 3

Outputs (heading, yaw rate), diagnostics (sigma, state count, reject count),
and inputs present (C=compass, G=COG, Y=gyro). Color tracks sigma confidence.

## Tests

New: test_can_recorder.py (8) -- the used-frame filter (keeps decoded IDs +
heading-any-source + claims, drops the rest), candump round-trip, stats. Total
544 passing.

## Note on the EKF COG weighting

app.py's EKF uses speed-weighted COG noise (R_cog = 400/sog_mph^2, floored),
not a hard speed switch -- so it leans on the compass derivative at idle and
smoothly into COG as you speed up. This is the threshold-free behavior discussed
for the EKF path (the priority chain keeps its 4 mph hysteresis as the fallback).

---

# Round 10: sea-trial fixes + stability tests

First on-water run surfaced two bugs, both now fixed and both now guarded by
stability tests that were missing before.

## 35. Compass / vehicle-direction crossed wire -- can_interface.py, hmi_bridge.py

The GPS vehicle-direction decode (PGN 65256) returned its course under the key
"compass", which the bridge mapped into compass_heading. So the stale
stationary-GPS course (~39 deg) kept overwriting the real wall compass (~180 deg
south), and the displayed heading bounced between them (~70 deg stdev). Fixed:
vehicle direction now returns as gps_vehicle_dir and never touches the compass;
the "compass" -> compass_heading alias is removed.

Also removed the boat_speed < 1 gate on the 127250 heading decode: it meant the
compass never updated the fusion while moving (the vehicle-dir bug had been
masking this). Heading now flows at all speeds; the bridge decides when to use
it.

## 36. Heading goal single state -- autopilot.py

The goal bounced between the set value and 360 because the autopilot normalized
with `> 360` (leaving 360 as 360) while the HMI state used `% 360` (storing 0),
so the two goal copies diverged. All autopilot goal/heading wraparounds now use
`% 360.0`, matching the state exactly -- 360 always becomes 0, and the two stay
in lockstep.

## 37. Stability tests -- tests/test_stability.py (new), test_seatrial_fixes.py (new)

The class of test that was missing: assert the numbers feeding the control loop
and the display do not bounce when the input is steady. Replays a real parked-
facing-south capture and checks:
  * compass_heading and fused_heading stdev < 3 deg (measured ~0.3 deg)
  * fused holds ONE source (no flipping)
  * compass and fused agree (no wrong source leaking in)
  * the goal is one consistently-normalized value, never 360, stable across
    repeated 0/360 wraps (checked on the RAW value -- a circular metric hides
    the 0-vs-360 bounce)

Both tests were verified to FAIL when the respective bug is reintroduced, so
they actually catch the regressions rather than just passing. New fixture
parked_south.log (compact carve of the 11:23 capture, all devices claiming).
Total 559 passing.

---

# Round 11: EKF safe across CAN link transitions

Operator asked: with the app running a long time and no CAN (EKF sigma growing,
no signal), will enabling CAN later cause a problem? Investigating surfaced a
real bug and hardened the EKF around link loss.

## What sigma is (for the record)

Sigma is the EKF's own estimate of its heading uncertainty, in degrees -- the
square root of the heading variance it carries. Each predict step ADDS
uncertainty (time passes, the boat could have turned); each measurement
SUBTRACTS it (a compass/COG reading pins the heading down). With no CAN, there
are no measurements, so sigma grows without bound -- the filter honestly
reporting it is flying blind. derive_fused already ignores the EKF when
sigma > 15 deg.

## 38. EKF must not run on nothing -- hmi_bridge.py

Bug: with no CAN, step_ekf seeded the filter to heading 0.0 and published it,
so the EKF fed a fabricated NORTH heading into the control loop while the link
was down (and derive_fused accepted it, since the fresh filter's sigma was
small). Fixed three ways:

  * The EKF is not constructed until a real heading reference (compass or COG)
    exists. No reference -> no ekf_heading published.
  * Signals are read with FRESHNESS (Signal.is_valid), not just last value.
    signal_value returns the last value even when stale, so on a CAN drop the
    compass/COG looked frozen-but-present and kept driving the filter; now a
    stale reference is treated as absent.
  * If the EKF has coasted (no fresh reference) for > 3 s, it stops publishing
    ekf_heading (sigma is still shown on the TUI). So a stale estimate never
    appears confident or drives the loop.

## Recovery is clean

Verified: after 2 hours of predict-only (sigma ~7e5 deg, mathematically huge
but finite -- no NaN/inf), the first real measurement snaps the heading to the
correct value and sigma back under 3 deg within a second. So leaving the app
running without CAN and later enabling it is safe: the filter rebuilds from the
first real reference. The only visible effect is the expected one-time snap to
the true heading when data returns.

## Tests

New: test_ekf_can_recovery.py (6) -- no publish without a reference, no feeding
fused without a reference, clean convergence when CAN comes up, no NaN after a
2-hour coast, and stop-publishing + fused-fallback when CAN drops mid-run. Total
565 passing.

---

# Round 12: engage on a solid compass when parked; web heading/goal fixed

Second sea trial (parked facing south). Three problems, all fixed.

## 39. Sigma stuck ~12, could not engage -- hmi_bridge.py

Parked with a rock-solid compass (179.4 deg, 0.1s age) but no COG, the EKF sat
at sigma ~10-12 and, being under the old 15 deg gate, fed the fused heading --
yet that sigma was too high to engage. So a perfect compass could not engage the
autopilot in the parking lot. Two fixes:

  * The compass now feeds the EKF as an ABSOLUTE reference, not just its
    derivative, so sigma CONVERGES when parked (to ~0.5 deg). The weight scales
    with COG availability: R=2000 (~loose) when COG is present so COG dominates
    the track and the compass bias is ignored (preserving fishing behaviour);
    R=25 (~5 deg) when COG is absent so the compass pins the filter.
  * The EKF trust gate tightened from 15 to EKF_TRUST_SIGMA = 6 deg. Above that
    the EKF is not confident enough (e.g. compass-only mid-convergence) and
    derive_fused falls through to the priority chain, which uses the compass
    directly -- so a solid compass always yields an engageable heading.

## 40. Web heading blank + goal bouncing -- static/js/compass.js

The web page conflated heading with COG: it set `heading = data.COG` and then
computed the dial goal as `ang + heading`. Parked (no COG), the Heading readout
showed '----' and the goal bounced off the stale/absent heading (the TUI showed
GOAL 338.7 from the web while the web showed 180). Fixed: the page now reads the
real heading (heading_deg -> compass_deg -> COG, in order), so Heading shows the
compass and the goal dial computes against a stable heading.

## 41. Web-data unit tests -- tests/test_web_data.py (new)

The tests the operator asked for, to ensure the web data entries are properly
shown. Two layers:
  * the telemetry snapshot always carries the display keys (heading_deg,
    compass_deg, cog_deg, heading_goal, ...) with sane values, and a heading is
    present whenever the compass is (the parked regression);
  * compass.js reads a real heading field, not only COG (static analysis).
Verified to FAIL when the COG-only heading bug is reintroduced.

Plus parked-engage tests (sigma converges on a steady compass; fused falls
through to the compass when the EKF is not confident). Total 577 passing.

---

# Round 13: compass wheel rotates / shows heading when parked

Symptom: the data inside the compass wheel was missing and the wheel did not
turn. Expected: the wheel rotates around the boat with heading, and the green
goal line/ball sits at its position on the wheel.

## 42. heading_deg was stuck at 0 when parked -- app.py

Root cause: the web wheel rotates by heading_deg, which came from
autopilot.current_heading. But current_heading is ONLY set when a COG message
arrives (broadcast_can_message calls autopilot.set_heading(COG)). Parked with no
COG, it stays 0. The snapshot published heading_deg: 0 -- present and non-null,
so compass.js took heading = 0 and never fell back to compass_deg. The wheel
stuck at north and showed blank.

Fixed: heading_deg is now derived as the best REAL heading -- the EKF when
confident (sigma < 6), else the compass, else COG -- never a stale 0 when a real
heading exists. The wheel now rotates to the true heading (e.g. 179 south) and
the goal ball, drawn inside the rotating ring, sits at its correct mark.

Also: the EKF is stepped once per snapshot now (was twice, which doubled dt).

## 43. Test strengthened -- tests/test_web_data.py

Added test_heading_deg_is_real_not_stuck_zero_when_parked: parked with a valid
compass but autopilot.current_heading = 0, heading_deg must reflect the compass
(~179), not the stuck 0. Verified to FAIL with the old
heading_deg = current_heading. Total 578 passing.

---

# Round 14: address-claim columns in the compact source table

Operator wanted the address-claim data (who/what the device is, and its serial)
visible in the TUI without needing a 136-column terminal.

## 44. MFG / FUNCTION / IDENT in the compact table -- hmi_tui.py

The wide (136-col) table already had CLASS/FUNCTION/MFG/IDENT, but a normal
terminal shows the compact table, which had only ID/NAME/AGE/STATUS/COUNT -- so
the claim data was invisible in practice. The compact table now also shows:
  * MFG      -- manufacturer (Garmin, Airmar, ...)
  * FUNCTION -- device function (Heading Sensor, GPS, Steering, ...)
  * IDENT    -- the unique identity/serial from the NAME

These come from the decoded address claim (j1939_name), joined by source
address, so every PGN from a node shows that node's identity. Blank until the
source sends a claim. Columns truncate cleanly on narrower terminals (IDENT
drops first).

## Tests

Added TestClaimColumnsInCompactTable (2) to test_tui_render.py: the compact
table shows the MFG/FUNCTION/IDENT headers and the decoded values. Total 580
passing.

---

# Round 15: one app, one source of truth (web + TUI unified)

Symptom: goal set on the web did not update in the TUI, and the compass wheel
did not turn. Root cause was not a wraparound bug this time -- it was that
app.py (web) and app_tui.py (TUI) were being run as TWO separate processes, each
with its own Autopilot and its own heading goal. They shared only the CAN bus,
not memory, so:
  * the web's heading goal lived in app.py's autopilot; the TUI's lived in
    app_tui's state -- setting one never updated the other.
  * app.py's heading_deg came from autopilot.current_heading (COG-only), so the
    wheel could be fed a stale value.

## 45. app.py delegates to the unified app -- app.py

app_tui.py was already a complete web + TUI app (it serves /, /ws, /plot, /health
and broadcasts STATE.snapshot(), which has ONE heading_goal and heading_deg =
fused_heading). app.py is now a thin shim that launches app_tui in server-only
mode (--no-tui). So `python3 app.py` still gives the web UI, but backed by the
single source of truth. For web + TUI together: `python3 app_tui.py --ekf`.

Now the web and TUI share one goal (st.heading_goal via APP.set_goal) and one
heading (fused_heading) -- set the goal anywhere and it shows everywhere, and
the wheel rotates with the real heading.

## 46. Recorder + log download ported into app_tui -- app_tui.py

So the single app has everything app.py had: the FilteredCanRecorder now runs in
app_tui's read loop, the /used-can-log route serves it, and the broadcast
includes log_path/log_count for the /plot download bar.

## Tests

test_web_data.py rewritten to build telemetry the unified way (feed the bridge,
read STATE.snapshot()) and now asserts the goal is a single source of truth
(set it, and both the snapshot and state.heading_goal agree). Total 580 passing.

---

# Round 16: PID rewrite + TUI PID tuning panel

## 47. True accumulating integral -- autopilot.py

The integral did not accumulate: compute_rudder_command set self._integral = 0
every cycle and recomputed a windowed trapezoidal sum. So on a stationary boat
chasing an unreachable goal, the integral never wound up and the shaft never
limited out. Rewritten to a TRUE accumulator: self._integral += error*dt each
cycle, clamped to an integral limit so overshoot stays bounded when the boat
gets steerage way. The output is clamped to the shaft's physical travel, so the
integral winds up and the shaft LIMITS OUT chasing an unreachable goal -- the
expected behaviour. Integral resets on disengage. Also fixed the send block: it
compared the mean but sent the instantaneous value, and the 100-count deadband
swallowed gradual windup (now 20, and it always sends at a travel limit).

Terminology corrected in the docstring: SHAFT (intermediate power shaft, geared
via timing-belt pulleys, reported by the 'steering' node string pot, counts) is
NOT the rudder. RUDDER angle is a separate string pot on the 'rudder' node
(degrees), feedback only.

## 48. TUI PID tuning panel -- hmi_tui.py, app_tui.py, autopilot.py

New PID panel in the upper-right (when the terminal is wide enough, right region
>= 72 cols; STATUS keeps its place at 120). Shows the three gains (Kp/Ki/Kd), the
live P/I/D term contributions in shaft counts, and the raw integral, derivative,
and error. Gains are tunable from the TUI: TAB selects which gain, +/- adjust it
(steps Kp 1.0, Ki 0.1, Kd 0.5). Changing Ki recomputes the integral clamp so
authority stays consistent. Gains published to state (pid_kp/ki/kd, pid_p/i/d,
pid_integral, pid_derivative) so the panel and the web can read them.

## Tests

test_pid.py: 9 behaviour tests (integral accumulates, is not reset, more error =
more steering, shaft limits out both directions, output/integral clamped, reset
works) -- verified to FAIL when the integral-reset bug is reintroduced. Plus 4
panel tests (gains shown, term contributions shown, selected gain marked, gain
command adjusts the autopilot). Total 593 passing.

---

# Round 17: engage starts from zero error

## 49. Engage seeds the goal to the fresh current heading -- app_tui.py

Engaging now guarantees the control loop starts from zero error, so the boat
holds its present course instead of lurching toward a stale or slightly-skewed
setpoint. At engage:
  * the heading is read with FRESHNESS (not signal_value, which returns the last
    value even when stale) -- if there is no fresh heading, engage is REFUSED
    rather than seeding the goal from an inaccurate/old value;
  * the goal and the controller's current_heading are pinned to the SAME fresh
    value, and heading_error is set to 0, so the first PID cycle sees exactly
    zero error;
  * the integral is cleared so no windup carries into the fresh engagement.

Once engaged the goal stays locked (the disengaged goal-follows-heading logic is
skipped while engaged), so subsequent cycles track real drift against the fixed
course.

## Tests

test_engage_zero_error.py (4): engage starts from zero error, seeds the goal to
the current heading, clears the integral, and refuses a stale heading. Verified
to FAIL if current_heading is not pinned at engage. Total 597 passing.

---

# Round 18: fix autopilot run-loop crash (_disengaged_goal_ref)

## 50. Uninitialized _disengaged_goal_ref crashed the run thread -- autopilot.py

The disengaged goal-follows-heading logic (round 13, added to stop the green
goal line jittering on compass noise) used self._disengaged_goal_ref in the
run() loop but never initialized it in __init__. On the first disengaged tick
the thread died with:

    AttributeError: 'Autopilot' object has no attribute '_disengaged_goal_ref'

which took out the control loop (autopilot_status went STALE). Fixed by
initializing self._disengaged_goal_ref = None in __init__.

## Why the tests missed it, and the fix

Every PID test called compute_rudder_command() directly and never spun up the
real run() thread, so the uninitialized attribute -- which is only touched in
run()'s disengaged branch -- was never exercised. Added TestRunLoopDoesNotCrash
(3): the attribute is initialized, and the real run() thread survives several
ticks both disengaged and engaged (with a mock bus). Verified to FAIL (the exact
AttributeError) when the init is removed. Total 600 passing.

---

# Round 19: compass wheel actually rotates; goal follows real heading

## 51. Compass wheel rotation -- static/js/compass.js

The wheel never rotated. The cardinal letters were positioned at fixed ring
angles and then each glyph was counter-rotated by +heading to "stay upright" --
but because the whole ring was already rotated by -heading, that per-letter
+heading rotation cancelled the ring rotation for the letters, pinning N to the
top for every heading. The ticks are rotationally symmetric every 5 deg, so they
gave no visual cue, and the wheel looked static.

Fixed by drawing the cardinal letters in an UNROTATED frame with their position
computed as (bearing - heading): a heading-up card where the letter for the
current heading sits at the bow. Verified in a headless browser: heading 0->N,
90->E, 180->S, 270->W at the bow. Facing south, S is now at the bow as expected.

## 52. Disengaged goal follows real heading -- autopilot.py

Reverted the round-13 deadband that only updated the disengaged goal on a > 1
deg change. That was suppressing the green goal line's motion to hide compass
noise, but it hid real movement too. Per the operator, jitter is fine as long as
it reflects real data, so the disengaged goal now follows the current heading
directly. (Source conflation was checked separately: the fused heading correctly
holds the compass and rejects the stale stationary COG, so the displayed heading
is not a mix of sources -- the earlier "conflation" was the non-rotating wheel
disagreeing with the heading number.)

## Tests

test_web_data.py + TestCompassWheelRotation (2): the letters are positioned by
(bearing - heading) and the per-letter +heading counter-rotation is gone. Total
602 passing.

---

# Round 20: compass-face value labels on the top layer

## 53. Heading/Speed/Rudder/Goal labels missing -- static/js/compass.js

The value labels on the compass face (Heading, Speed, Rudder, Goal and their
degree values) were being drawn immediately after the boat sprite, inside the
boat's transform block, and were not reliably visible. Moved the entire label
pass to the END of draw(), after the ring, boat, rudder and crosshair, so the
labels are unambiguously the top layer and nothing can cover them. Positions
unchanged (four quadrants around the boat); added a fallback boat-height ratio
so the labels still place correctly if the sprite is not fully loaded.

Verified by serving the real page headless and confirming the blue label pixels
render over the boat, with the wheel facing south (S at the bow).

## Tests

test_web_data.py + TestCompassFaceLabels (2): the labels are drawn after the
boat and crosshair (top layer), and all four label strings are present. Total
604 passing.

---

# Round 21: graphs button wired; compass.js version stamp; stale-file diagnosis

The rendered DOM the operator sent confirmed the boat is running an OLD
compass.js: my current compass.js, tested against that exact DOM, renders the
face labels correctly (verified on the real served page: 18k blue label pixels).
So the missing numbers are a stale-file/cache issue, not a code bug.

## 54. Graphs button wired -- static/js/compass.js

The footer Graphs button (graphsBtn) had no click handler, so it did nothing.
Wired it to navigate to /plot, which serves plot_data.html with the live Plotly
graphs (RPM/speed, angles, distance, rudder) from vendored local assets. The
/plot route already exists in app_tui.

## 55. Version stamp in compass.js -- static/js/compass.js

compass.js now logs a version line on load:
  "compass.js v0.9.13 — labels top-layer, graphs wired"
so the operator can open the browser console (F12) and instantly confirm which
compass.js the boat actually loaded -- the recurring stale-file problem.

## Tests

test_web_data.py + TestGraphsButtonAndVersion (3): graphs button wired to /plot,
version stamp present, /plot route exists. Total 607 passing.
