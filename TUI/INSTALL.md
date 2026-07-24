# Installing

Copy these files over your existing `HMI/` folder. They sit alongside what is
already there; nothing is deleted.

```
app_tui.py            entry point (server + TUI)
hmi_*.py              new modules
autopilot.py          modified: stop-event handling
can_interface.py      modified: J1939 pitch offset
templates/*.html      unchanged, included for reference
static/js/compass.js  modified: WebSocket URL no longer hardcodes port 5000
static/sw.js          unchanged, included for reference
tests/                the suite
pytest.ini            pytest config
```

**Not included:** `static/js/plotly-2.35.2.min.js`, `socket.io.min.js`,
`leaflet-1.9.4/`, `static/images/`, `static/vendor/`, `static/tiles/`. Those
are vendored libraries and assets you already have — 4.5 MB of Plotly alone.
Six asset-resolution tests will fail if you run the suite anywhere those are
missing; they pass in your tree.

## Running

```bash
pip install pytest pytest-cov pytest-randomly beautifulsoup4
python3 -m pytest                                     # 386 tests, ~70s
python3 -m pytest -m "not integration and not slow"   # 275 tests, 1.4s
python3 -m pytest tests/test_web.py                   # web interface only
python3 -m pytest --hardware -m hardware              # on the boat
```

`node` is optional — the JavaScript syntax checks skip without it.

## One thing to apply yourself

The `/sw.js` 500 also exists in your original `app.py`, around line 217:

```python
# before
(r"/sw.js", tornado.web.StaticFileHandler,
    {"path": os.path.join(os.path.dirname(__file__), "static"),
     "default_filename": "sw.js"}),

# after
(r"/(sw\.js)", tornado.web.StaticFileHandler,
    {"path": os.path.join(os.path.dirname(__file__), "static")}),
```

I left `app.py` untouched so you keep a working fallback, but it has the same
bug.
