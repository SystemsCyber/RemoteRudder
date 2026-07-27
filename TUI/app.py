"""
app.py -- compatibility shim.

The web server and the TUI are now ONE application (app_tui.py) with a single
source of truth for heading, goal, and state. Running app.py and app_tui.py as
two separate processes gave each its own autopilot and its own heading goal, so
a goal set on the web never reached the TUI (and vice versa) -- they only shared
the CAN bus, not memory.

To avoid that split, app.py now launches the unified app in server-only mode
(no curses TUI), which is what `python3 app.py` used to provide: the web UI on
port 5000. For the web + TUI together, run:

    python3 app_tui.py --ekf

This shim keeps existing scripts/muscle-memory working while guaranteeing a
single source of truth.
"""

import sys

if __name__ == "__main__":
    # Delegate to the unified app in server-only mode.
    sys.argv = [sys.argv[0], "--no-tui"] + sys.argv[1:]
    import app_tui
    app_tui.main()
