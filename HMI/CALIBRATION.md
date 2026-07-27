# Calibration notes — findings from the existing capture set

## What the logs actually contain

| Log | compass (127250) | COG (129026) | samples >4 m/s |
|---|---|---|---|
| candump-fullturns-michelleski | 3425 | 944 | **233** |
| candump-autopilot-north-in-southbay | 5986 | 1640 | 0 |
| candump-autopilot in south bay | 1851 | 509 | 0 |
| candump-autopilot-engaged-heading-bounce | 1499 | 410 | 0 |
| Figure 8 full turns (.trc.log) | 0 | 239 | 0 |
| Figure 8 half turns (.trc.log) | 0 | 420 | 0 |
| ZigZag 1200 rpm (.trc.log) | 0 | 110 | 0 |

**Only one log has both a compass and fast running.** The three `.trc.log`
figure-8/zigzag captures contain no PGN 127250 at all — the compass either
wasn't on the bus or wasn't being logged on that day. This is the single
biggest gap in the dataset.

## Sensor inventory (decoded, not assumed)

* **Compass**, PGN 127250, SA **248**, ~138 ms period.
  Reference field reads **magnetic** on every frame. Deviation is 0. The
  variation field reads 187.7°, which is not a valid variation and not a
  standard sentinel — treat the sender's variation as unusable and estimate
  the offset in the filter instead.
* **GPS**, PGNs 129025/129026, SA **28**, ~500 ms period.
* **PGN 65256 (vehicle direction), SA 28** duplicates COG from the same
  receiver to within 0.01°. It is *not* an independent heading source and is
  deliberately not registered as one.
* **No second GPS and no rate gyro appear in any capture.** SA 248 and SA 28
  are the only positioning/heading talkers. The registry has slots for both;
  they are currently empty.
* One position fix in the fullturns log reads lat −83.88°, lon 186.23° — a
  bad fix that passes the N2K sentinel check. Range-gate lat/lon before use.

## The compass error is heading-dependent

Correlating compass against COG during **straight, fast** segments only
(|rate| < 2°/s, SOG > 4 m/s):

```
compass 0–45°   : n=112  median offset −18.9°
compass 315–360°: n= 65  median offset  +4.2°
```

A 23° swing across 90° of heading is a hard-iron signature, not magnetic
declination (which is ~+8° and constant for Horsetooth). **A single scalar
offset state cannot represent this** — an early version of this filter used
one and the estimate wandered as the boat turned. The state vector therefore
carries a constant term plus a one-cycle harmonic:

```
compass_reading = psi − (d0 + ds·sin psi + dc·cos psi)
```

## Honest limits of the current tuning

Straight-and-fast running occurs on only **two heading sectors** in the entire
dataset (roughly 0–45° and 315–360°). Three unknowns (d0, ds, dc) fitted from
two sectors of one 474-second log is underdetermined.

A parameter sweep showed the validation score improving monotonically as the
harmonic priors were loosened, with the estimated iron amplitude running to
45°. That is the metric being gamed, not the model improving: the filter is
being scored against COG while simultaneously fitting to COG, so looser
harmonics always win by absorbing crab angle. Because no held-out log with
fast compass data exists, that sweep cannot be trusted.

The shipped values are therefore **conservative, not sweep-optimal**:

* `P0_HARM` = (8°)², `Q_HARM` = (0.005°/s)²
* `MAX_IRON_AMP` = 15°, a hard clamp on √(ds²+dc²)

Validation on the fullturns log with these settings: mean 4.3°, median 2.2°,
p95 16.1°. The looser sweep reached p95 11°, but at an implausible 45° iron
amplitude. Prefer the conservative numbers until better data exists.

## The one capture that would fix this

A **slow, steady, full 360° circle at planing speed in flat water** — ideally
two of them, in both directions. Requirements:

* SOG above ~10 mph throughout, so COG noise is small
* Yaw rate low and constant — a wide circle, not hard-over turns
* Calm water and minimal wind, so crab angle stays small
* Compass (PGN 127250) confirmed present on the bus before you start

That single run gives full heading coverage and lets `fit_iron.py` solve d0,
ds, dc directly by least squares. Seed the filter with those values, tighten
`P0_HARM`, and the low-speed performance improves too — the harmonic model is
shared across the whole speed range.

The existing "Attempted Compass Calibration ... 2 circles" capture only spans
compass 21–142°, so it does not close the circle. It agrees with the fullturns
numbers over the sectors it covers, which is a useful consistency check but
not enough to fit on.

## Second GPS and gyro

Both are handled by adding registry entries — no filter changes:

```python
kf.register(SensorSpec("cog_aux", "cog", SRC_GPS2,
                       H=[1,0,0,0,0,0], r_fn=kf._cog_variance,
                       gate=Params.GATE_COG))
kf.register(SensorSpec("gyro", "rot", SRC_GYRO,
                       H=[0,1,1,0,0,0], r_fn=lambda o,f: Params.R_GYRO,
                       gate=Params.GATE_GYRO))
```

With two GPS units the chi-squared gate does redundancy management for free:
whichever agrees with the propagated state is accepted, the outlier is gated.
Watch `gate_pct` per sensor in `health_report()` — a channel that starts
gating heavily is failing before it fails hard.

Note the gyro row is `[0,1,1,0,0,0]`, measuring rate **plus** bias, which is
what makes the bias state observable. Without a gyro, `x[BIAS]` stays at its
prior and does nothing; that is harmless but means the state is currently
carrying an unobservable element. Do not be alarmed by a flat bias trace until
a gyro is actually on the bus.
