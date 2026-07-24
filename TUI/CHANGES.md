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
