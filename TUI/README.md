# HMI test suite

386 tests, 97% coverage on the HMI modules. The full run takes about 70 seconds;
the fast subset takes 1.4.

```bash
python3 -m pytest                              # everything
python3 -m pytest -m "not integration and not slow"   # 275 tests, 1.4s
python3 -m pytest -m regression                # real-log golden checks
python3 -m pytest --hardware -m hardware       # needs the PCAN plugged in
python3 -m pytest --cov=hmi_state --cov=hmi_heading \
                  --cov=hmi_bridge --cov=hmi_canlink \
                  --cov=hmi_tui --cov-report=term-missing
```

## Layout

| File | Tests | Covers |
|---|---|---|
| `test_state.py` | 51 | Signal freshness, goal normalization, fault latching, snapshot contract, thread safety |
| `test_heading.py` | 57 | Circular statistics, noise attribution, engage interlock |
| `test_decode.py` | 33 | `process_message` against real captured frames |
| `test_canlink.py` | 36 | Bus lifecycle, platform resolution, staleness |
| `test_bus_health.py` | 15 | Kernel error counters, bitrate-mismatch signature |
| `test_error_paths.py` | 23 | Driver exceptions, the NOISY band, threshold boundaries |
| `test_regression.py` | 26 | Full-log replay, physical plausibility, COG gating |
| `test_integration.py` | 30 | Bridge mapping, live Tornado server, phone/TUI goal sync |
| `test_web.py` | 49 | Routes, HTML structure, asset resolution, JS syntax, payload contract |
| `test_tui.py` | 36 | Key handling, subprocess render |
| `test_tui_render.py` | 31 | In-process render at 10 terminal sizes |
| `test_hardware.py` | 10 | Real adapter (skipped unless `--hardware`) |

Coverage: `hmi_state` 100%, `hmi_canlink` 99%, `hmi_heading` 99%, `hmi_bridge`
99%, `hmi_tui` 94%.

## Fixtures are real captures

Frames are byte-for-byte from the logs in `logs/`, filtered to the eight
decoded arbitration IDs to keep them at 439 KB. See `fixtures/README.md` for
provenance.

Synthetic frames only prove the decoder agrees with whoever wrote the test.
These prove it agrees with the boat.

The two main slices exercise opposite regimes:

```
underway_fast.log   COG accepted 87, rejected  0    (0% rejection)
trolling_slow.log   COG accepted  3, rejected 21   (88% rejection)
```

`test_rejection_ratio_differs_between_regimes` asserts on the *difference*.
Remove the speed gate and it fails — while every other test still passes,
because each log alone would decode fine.

Finding the slow window took some doing: `combined_filtered.log` runs at
planing speed for its first ~100k frames, so slicing from the start gives a
second copy of the fast regime. It is at frame 123500. `make_fixtures.py`
scans for it rather than hardcoding an offset.

`trolling_slow.log` also contains zero `0x18FF50E0` frames — a real-world
missing-message case rather than a synthetic one.

## Golden values are bounds, not exact numbers

Pinning an exact float would make the suite fail on any legitimate improvement
to the filter. Pinning a physical range — "pitch on flat water is within 15
degrees of level", "position is near Horsetooth" — fails only when something
is genuinely wrong.

`test_golden_summary_underway` is one consolidated assertion over eleven
fields; when it fails it names which one drifted.

## Web interface tests

`test_web.py` was written after a reported 500. `/sw.js` was routed with

```python
(r"/sw.js", StaticFileHandler, {"path": ..., "default_filename": "sw.js"})
```

The regex captures no groups, but `StaticFileHandler.get()` requires a `path`
positional argument that comes from the capture. `default_filename` does not
supply it — that option only applies to directory requests. Every request
raised `TypeError` and returned 500, and since `plot_data.html` registers the
service worker on load, it fired on every visit to `/plot` and silently
disabled offline tile caching. The same bug is in the original `app.py`.

Fixed to `r"/(sw\.js)"`. Three tests catch a revert.

A second bug surfaced while writing these: `compass.js` hardcoded
`ws://host:5000/ws`, so running with `--port` anything else left the page
connecting to nothing — no error banner, just a dashboard that never updated.
Now derives from `location.port`, with `wss://` when the page is served over
HTTPS. The web fixture deliberately runs on a random port so this stays caught.

What the web tests check:

- every route returns 200 with the right content type; unknown routes 404
- every `src=`/`href=` the templates reference actually resolves (parametrized,
  so a failure names the missing file)
- HTML parses, script tags balance, no unrendered `{{ }}` template markers
- every element ID `compass.js` calls `getElementById` on exists in the HTML
  (14 of them; a renamed element gives a null reference and a dead widget with
  no error unless the console is open)
- JavaScript parses via `node --check`, including inline blocks
- the WebSocket emits the keys the browser guards on, and never bare `NaN` or
  `Infinity`, which `JSON.parse` rejects
- directory traversal is blocked and odd requests return 4xx rather than 5xx

Both fixes were mutation-tested: reverting each one turns the relevant tests
red, so they are verifying behaviour rather than passing vacuously.

## Two things the tests found

**Footer truncation.** The keybinding hint was exactly 80 characters but only
78 fit at an 80-column terminal, so `[q]uit` rendered as `[q]u`. Fixed, and
`test_shows_keybindings` now asserts the full string.

**RPM never appears during fast replay.** The engine decoder throttles on wall
clock (`GUI_TIMEOUT = 0.35 s`), but replay feeds 2,150 engine frames instantly,
so zero decode. At realistic pacing all 2,150 decode fine (637 rpm idle). Not a
code bug, but it means the RPM field looks dead in offline demos. The
`wired.feed()` harness takes `defeat_rate_limits=True` to reproduce live
behaviour, and `test_rpm_throttle_suppresses_under_instant_replay` pins the
throttle so nobody removes it and floods every connected phone at 100 Hz.

## Two documented limitations

**The sources panel truncates.** It is capped at 9 rows, leaving room for 6
entries after the border and header. With 8 watched PGNs, the last two —
currently `vessel_heading` and `autopilot_status` — are never drawn on a
24-row terminal. That matters because `vessel_heading` is the compass source,
so a compass going quiet is invisible in the panel. The FAULTS line still names
it via the STALE fault. `test_sources_panel_truncates_at_24_rows` pins this;
update it if the panel is ever made scrollable.

**`SIGMA_ENGAGE_MAX = 6.0` blocks engagement on the real Horsetooth data**,
where observed sigma runs 10–15 degrees. `test_engage_threshold_is_the_documented_value`
pins the constant so raising it to get a demo working is a conscious edit
rather than a silent one.

## Notes on the harness

**Isolation.** Every fixture is function-scoped. `SystemState` latches faults
and holds an event deque, so a shared instance would make test order
significant. Verified with `pytest-randomly` across multiple runs.

**Time.** A `fake_clock` fixture patches `time.time` in the modules that age
things out, so staleness and window-expiry tests are deterministic rather than
sleeping.

**curses.** `test_tui_render.py` attaches curses to a pty inside the test
process, which is what makes the drawing code visible to coverage (27% → 94%).
`initscr()` reads `LINES`/`COLUMNS` from the environment before consulting the
tty, so both are pinned in the fixture; without that the screen comes up at
40×9 and every content assertion hits the "terminal too small" path.

**Server tests** spawn a real Tornado process on an OS-assigned free port.
Slower than calling handlers directly, but it is the only way to catch JSON
that will not serialize, a handler that raises on malformed input, or a
snapshot key the browser depends on quietly disappearing.

## Adding tests when the Kalman filter lands

The seam already exists. `hmi_bridge` routes `fused_heading`, `heading_sigma`,
and `yaw_rate`, and `test_filter_surface_accepts_injected_values` proves it
works before the filter is written. When the filter is wired in:

1. Replace `derive_fused()` with the filter output — no other HMI changes.
2. Add golden checks on filter convergence against `trolling_slow.log`, where
   COG rejection actually matters.
3. Consider gating engagement on the filter's post-fit residual instead of raw
   compass scatter, which would let `SIGMA_ENGAGE_MAX` come back down.
