"""
Single source of truth for the HMI build version.

Bump VERSION when you cut a new build. It shows in the TUI header, on the web
page, and via `python3 app_tui.py --version`, so you can always tell -- on the
boat, from a screenshot -- exactly which code is running. This is what makes a
sea-trial log correlatable to a code version.

FEATURES lists what each version introduced, so a glance tells you whether the
build in front of you has the EKF, the fishing mode, etc.
"""

VERSION = "0.9.13"

# What landed in each version, newest first. Handy when looking at a screenshot
# and asking "does this build have X?"
FEATURES = {
    "0.9.13": "graphs button wired to /plot; compass.js version stamp in console",
    "0.9.12": "compass-face value labels drawn as top layer (Heading/Speed/Rudder/Goal)",
    "0.9.11": "compass wheel actually rotates (heading-up card); disengaged goal follows real heading",
    "0.9.10": "fix crash: _disengaged_goal_ref uninitialized in autopilot run loop",
    "0.9.9": "engage starts from zero error (goal seeded to fresh current heading)",
    "0.9.8": "PID rewrite (true accumulating integral) + TUI PID tuning panel",
    "0.9.7": "single app: web + TUI share one goal/heading (app.py now delegates to app_tui)",
    "0.9.6": "address-claim columns (MFG/FUNCTION/IDENT) in the compact source table",
    "0.9.5": "compass wheel rotates (heading_deg reflects real heading, not COG-only)",
    "0.9.4": "engage on solid compass when parked; web heading/goal fixed; web-data tests",
    "0.9.3": "EKF safe across CAN drop/recover (no blind heading, clean re-converge)",
    "0.9.2": "stability tests (heading + goal do not bounce)",
    "0.9.1": "sea-trial fixes: compass/vehicle-dir split, single goal state, heading at all speeds",
    "0.9.0": "live web graphs, filtered CAN recorder, TUI EKF strip",
    "0.8.0": "COG-primary threshold 4mph + hysteresis",
    "0.7.0": "heading EKF (compass-derivative fusion), off by default",
    "0.6.0": "COG-primary fishing mode, yaw-rate limiting",
    "0.5.0": "GPS 24xd, NAME-bound heading sources, calibration TUI",
    "0.4.0": "J1939 address claims, 8-column source table, REQ ADDR",
    "0.3.0": "curses TUI, touch buttons, COG-primary groundwork",
}


def version_string() -> str:
    return f"RemoteRudder HMI v{VERSION}"


def version_banner() -> str:
    feat = FEATURES.get(VERSION, "")
    return f"RemoteRudder HMI v{VERSION} — {feat}" if feat else version_string()
