# Test fixtures

Real CAN captures, carved down so they can live in the repo. Frames are
byte-for-byte copies from the original logs — only unwatched arbitration IDs
were dropped, which is what keeps them small.

Synthetic frames would only prove the decoder agrees with whoever wrote the
test. These prove it agrees with the boat.

| File | Frames | Source | Why it exists |
|---|---|---|---|
| `underway_fast.log` | 4,000 | `candump-2025-08-06_16442_horsetooth_firstConstants.log` | Boat underway, 3–4 mph. All eight decoded PGNs including autopilot status. The "everything works" baseline. |
| `trolling_slow.log` | 5,000 | `combined_filtered.log` @ frame 123000 | Trolling: 98% of SOG samples below the 1.6 mph COG cutoff, min 0.11 mph. Also contains **zero** autopilot status frames. |
| `smoke.log` | 400 | Horsetooth capture | Minimal fixture for tests that just need the decoder to turn over. |

## Why two speed regimes

They exercise opposite paths through the same code:

```
underway_fast.log   COG accepted 87, rejected  0   (0% rejection)
trolling_slow.log   COG accepted  3, rejected 21   (88% rejection)
```

`test_regression.py::TestCogGating::test_rejection_ratio_differs_between_regimes`
asserts on the *difference*. If the speed gate is ever removed, both ratios go
to zero and that test fails — while every other test still passes, because
each log on its own would still decode fine.

This matters for the Kalman filter work: COG becomes unreliable at trolling
speed, which is exactly when you most want the compass weighted heavily.

## The missing autopilot frames

`trolling_slow.log` has no `0x18FF50E0` frames at all. That is not a defect in
the fixture — it is what the boat was actually transmitting. It doubles as a
real-world missing-message case, which is more convincing than a synthetic one
because it reproduces the exact combination of *some* nodes going quiet while
others keep talking.

## Regenerating

If you need different slices, `tests/make_fixtures.py` carves them from the
full logs in `logs/`. Note the trolling window is at frame ~123000 in
`combined_filtered.log`; the first 100k frames are all at planing speed, so a
naive `head` gives you the wrong regime. The script finds the densest slow
window automatically.

```bash
python3 tests/make_fixtures.py
```

The full source logs are not committed (100 MB total).

## July 2026 captures (COG-primary)

| File | Frames | Source | Why it exists |
|---|---|---|---|
| `planing_2026.log` | 6,000 | `candump-2026-07-24_130223.log` @ 40k | Planing, 13.6-25.1 mph, 0% below the COG threshold. The "COG lock at speed" case. |
| `docking_2026.log` | 5,000 | `candump-2026-07-24_133327.log` @ 2k | Docking/idle, 86% of SOG below threshold (0.2-2.5 mph). The no-lock / wait-for-motion case. |

These two anchor the COG-primary tests the way underway/trolling anchor the COG
gating tests. The key property: the compass in these runs never exceeds ~264
deg while COG spans the full circle, which is the evidence that the compass is
the unreliable sensor on this hull and COG must be primary.

`make_fixtures.py` does not regenerate these two automatically -- they were
carved from specific windows of the July 2026 runs. The offsets are in the
CHANGES notes if you need to recut them.
