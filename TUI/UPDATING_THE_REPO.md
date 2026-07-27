# Updating your repository with these changes

You're heading into sea trials, so the priority is: get these changes into your
repo cleanly, keep a way to back out, and make it easy to correlate a code
version with what you saw on the water. Here's the workflow I'd use.

## 1. Before you touch anything: branch and baseline

```bash
cd ~/RemoteRudder          # your repo root
git status                 # make sure your tree is clean or stash first
git checkout -b heading-fusion-seatrials
```

Working on a branch means `main` stays exactly as it is now. If a sea trial
goes badly, you switch back to `main` and you're on known-good code in one
command.

## 2. Bring in the new and changed files

The rounds are overlays -- each zip contains only what changed. Unzip them in
order (round 3 through round 9) into your repo, letting later rounds overwrite
earlier ones:

```bash
for r in 3 4 5 6 7 8 9; do
  unzip -o HMI_round${r}.zip -d /tmp/rr
  cp -r /tmp/rr/HMI_round${r}/hmi/.       .          # modules + CHANGES.md
  cp -r /tmp/rr/HMI_round${r}/tests/.     tests/     2>/dev/null || true
  cp -r /tmp/rr/HMI_round${r}/templates/. templates/ 2>/dev/null || true
done
unzip -o HMI_sensor_node.zip -d /tmp/rr && cp -r /tmp/rr/sensor_node .
```

Use `-o` (overwrite) so you don't get "replace? [y/N]" prompts, and always copy
with the trailing `/.` so hidden files come too. This overwrite step is the one
that has bitten us before: if you skip it and keep a stale local file, you get
"fixture not found" or "cannot import X" errors that look like real bugs but are
just old files.

## 3. Review before committing

```bash
git status                 # see everything that changed
git diff                   # read the actual changes
python3 -m pytest -q       # 544 pass, 10 skipped (hardware) -- your baseline
```

If the test count is off, something didn't copy. The full suite passing is your
signal that the tree is consistent.

## 4. Commit in logical chunks, not one giant blob

Commit by feature so the history tells the story and you can revert one piece
without losing the rest:

```bash
git add j1939_name.py heading_sources.py calibrate_compass_tui.py
git commit -m "Add J1939 NAME binding + GPS 24xd calibration tool"

git add heading_fusion.py heading_ekf.py compare_ekf.py
git commit -m "Add yaw-rate limiting, COG-primary fishing mode, heading EKF"

git add can_recorder.py app.py templates/plot_data.html
git commit -m "Add filtered CAN recorder + live web graphs"

git add tests/
git commit -m "Add tests (544 passing)"

git add CHANGES.md docs/ sensor_node/
git commit -m "Docs, changelog, sensor node"
```

## 5. Tag the version you actually sea-trial

This is the important one for correlating code with results. Right before you go
out, tag it:

```bash
git tag -a seatrial-2026-07-26 -m "First heading-fusion sea trial"
git push origin heading-fusion-seatrials --tags
```

Now every filtered CAN log you download on the water can be matched to an exact
code version. When you send me a log, note the tag, and I know precisely what
fusion produced it.

## 6. During and after the trial

- The filtered CAN logs land in `logs/used_can-*.log`. Keep them -- they're the
  record of what happened, and they replay directly.
- If you change a tuning constant on the water (e.g. `COG_PRIMARY_SPEED_MPH`,
  `YAW_RATE_MAX_DPS`), commit that change with a message noting *why* and what
  you observed. Future-you will want to know.
- When you're back, merge to main only what proved out:
  ```bash
  git checkout main
  git merge heading-fusion-seatrials
  ```

## A note on the two entry points

Your repo has two servers:
  * `app.py` -- the original web server (now with the EKF, recorder, and live
    graphs from round 9)
  * `app_tui.py` -- the curses TUI + Tornado server I built across rounds 3-8

They share the modules (`hmi_*.py`, `heading_*.py`, `j1939_name.py`, etc.) but
are separate front ends. Decide which you're running for the trial and tag
accordingly. If you want the round-9 web graphs on `app_tui.py` too, that's a
port I can do -- just ask.

## The one recurring gotcha

If you ever see "fixture not found", "cannot import name X", or a test failing
on something that clearly exists: it's almost always a stale local file from an
incomplete copy. Re-run the overwrite copy from step 2 (`cp -r .../. .`) and it
resolves. The package zips are overlays, not full trees, so a partial unzip
leaves old files in place.
