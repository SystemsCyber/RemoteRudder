# Installing (round 3)

Copy these over your existing `HMI/` folder. Nothing is deleted; `app.py` is
untouched and still runs as a fallback.

```
app_tui.py            engage gating, manual steering, snap-goal, heading feed
hmi_bridge.py         COG-primary fusion
hmi_state.py          heading_source / cog_lock fields
autopilot.py          COG-lock hold/center, None-safety fixes
hmi_tui.py            key remap, 120-wide layout, touch buttons
can_interface.py      (unchanged this round; included for completeness)
static/js/compass.js  (unchanged this round; WebSocket port fix from round 2)
templates/*.html      (unchanged; included for reference)
docs/PCAN_HOTPLUG.md  PEAK adapter auto-config
tests/                new + updated suites, plus two 2026 fixtures
```

**Not included:** vendored libraries and assets you already have
(`plotly-*.min.js`, `socket.io.min.js`, `leaflet-1.9.4/`, `static/images/`,
`static/vendor/`, `static/tiles/`). The web asset-resolution tests need those;
they pass in your tree.

## New key map

| Key | Action |
|-----|--------|
| Left / Right | goal -+1 |
| `[` / `]` | goal -+10 |
| Down | goal -5 |
| Up | snap goal to current heading (was Home) |
| `,` / `.` | manual motor step L / R, +-2 deg (disengaged only) |
| `e` / `d` | engage / disengage autopilot |
| `s` | toggle servo |
| `c` | clear faults |
| `q` | quit |

Touchscreen: the button bar along the bottom does the same actions. Buttons
grey out when unavailable (engage without a COG lock, manual steps while
engaged).

## Two things to do by hand

**1. PEAK adapter auto-config.** Follow `docs/PCAN_HOTPLUG.md` so `can0` comes
up at 250000 on plug-in. Until then, the manual bring-up still works:

```bash
sudo ip link set can0 up type can bitrate 250000
```

**2. The `/sw.js` 500 fix from round 2 is still needed in `app.py`** (line
~217) if you keep using `app.py`. `app_tui.py` already has it.

```python
# app.py, replace:
(r"/sw.js", tornado.web.StaticFileHandler,
    {"path": ..., "default_filename": "sw.js"}),
# with:
(r"/(sw\.js)", tornado.web.StaticFileHandler, {"path": ...}),
```

## Running

```bash
pip install pytest pytest-cov pytest-randomly beautifulsoup4
python3 -m pytest                              # 438 tests, ~90s
python3 -m pytest -m "not integration and not slow"   # fast subset
python3 -m pytest tests/test_cog_primary.py    # COG-primary behavior
python3 -m pytest --hardware -m hardware       # on the boat, PCAN plugged in
```

## Behavior notes worth knowing on the water

- **The autopilot will refuse to engage without a COG lock.** That means
  forward way on and a valid COG. Below ~1.6 mph it will not engage, and if
  engaged and you slow below that, the rudder centers and waits. This is by
  design for this hull.
- **COG dropping out at speed does not disengage.** The rudder holds its last
  position and the autopilot re-acquires when COG returns.
- **Manual steering is disengaged-only.** Disengage first (`d` or the DISENG
  button), then `,` / `.` to steer by hand. The servo auto-enables on the
  first step.
- **The compass is treated as unreliable** because in your captures it never
  read above 264 deg. If you recalibrate or replace it and it becomes
  trustworthy, revisit the COG-primary weighting in `hmi_bridge.derive_fused`.
