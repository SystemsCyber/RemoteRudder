# Round 21: graphs button + the stale-file fix for the missing numbers

## Key finding: your boat is running an OLD compass.js

You sent the rendered page, which was the missing piece. I ran MY current
compass.js against your exact rendered DOM, served the real page headless, and
the compass-face labels (Heading/Speed/Rudder/Goal) render correctly -- 18,000+
blue label pixels on the boat. So the code is right; the boat is loading a stale
compass.js (either the file on the Pi wasn't replaced, or the browser cached the
old one).

To make this impossible to misdiagnose again, compass.js now prints its version
to the browser console on load:

    compass.js v0.9.13 — labels top-layer, graphs wired

Open the browser console (F12 -> Console) and reload. If you do NOT see v0.9.13,
the boat is running an old file:
  1. Copy the new static/js/compass.js to the Pi's static/js/ (overwrite).
  2. Hard-refresh the browser: Ctrl-Shift-R (or clear cache).
Then the console should show v0.9.13 and the face numbers should appear.

## Graphs button now works

The footer "Graphs" button had no click handler, so it did nothing. It now
navigates to /plot, the live graphs page (RPM/speed, heading angles, distance,
rudder) we built earlier. The /plot route is served by the same app.

```
hmi/
  version.py     v0.9.13
  CHANGES.md
static/js/
  compass.js     graphs button wired + version stamp (+ the label fix)
tests/
  test_web_data.py  + graphs/version tests
```

## After installing

```bash
python3 app_tui.py --version    # v0.9.13
python3 -m pytest -q            # 607 pass, 10 skip

python3 app_tui.py --ekf
# open http://<pi>:5000, F12 console, hard-refresh (Ctrl-Shift-R)
# console should show: compass.js v0.9.13
```

Then: the compass face shows Heading/Speed/Rudder/Goal numbers, and the Graphs
button opens the live plots.
