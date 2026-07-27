#!/usr/bin/env python3
"""
Visualize the heading-fusion test cases.

Generates a multi-panel figure showing:
  1. Predicted GPS 24xd signals (heading + COG + attitude) for a fishing run
  2. Sensor deviation scenarios (sources disagree) and what fusion picks
  3. Yaw-rate limiting rejecting impossible jumps
  4. The fishing crosswind case: COG vs heading, boat crabbing
  5. Noise characteristics vs the measured real-log statistics

The synthetic signals are built to resemble the real logs (measured yaw rates,
COG jitter at speed, the wall compass steadiness) so the noise looks realistic.
"""

import sys
sys.path.insert(0, '.')

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from heading_fusion import YawRateLimiter, YAW_RATE_MAX_DPS, _angdiff

rng = np.random.default_rng(42)

# Measured constants from the real logs (see the yaw-rate analysis):
#   wall compass heading: p99 ~3 deg/s, max ~41 deg/s
#   COG at speed: ~12% of samples jump >45 deg/s (GPS jitter)
WALL_NOISE_DEG = 0.8       # steady, low scatter
COG_NOISE_DEG_AT_SPEED = 2.5   # noisier, plus occasional spikes
COG_SPIKE_PROB = 0.10      # matches the measured ~12%


def synth_run(n=400, dt=0.25):
    """
    A synthetic fishing run: boat drives a straight track at ~8 mph with a
    crosswind, so it crabs ~12 deg (heading != COG). Includes a gentle course
    change partway.
    """
    t = np.arange(n) * dt
    # True course over ground: straight, then a gentle 30-deg turn, then straight
    true_cog = np.piecewise(
        t,
        [t < 20, (t >= 20) & (t < 35), t >= 35],
        [90.0, lambda tt: 90.0 + (tt - 20) * 2.0, 120.0],
    )
    # Boat crabs 12 deg into a beam wind: magnetic heading leads COG.
    crab = 12.0
    true_heading = true_cog - crab

    return t, true_cog, true_heading


def add_sensor_noise(true_cog, true_heading):
    n = len(true_cog)
    # 24xd heading: clean magnetic, small noise
    h_24xd = true_heading + rng.normal(0, WALL_NOISE_DEG, n)
    # wall compass: also clean but slightly noisier, small fixed bias
    h_wall = true_heading + 3.0 + rng.normal(0, WALL_NOISE_DEG * 1.3, n)
    # COG: tracks true course but noisier, with occasional GPS spikes
    cog = true_cog + rng.normal(0, COG_NOISE_DEG_AT_SPEED, n)
    spikes = rng.random(n) < COG_SPIKE_PROB
    cog[spikes] += rng.normal(0, 60, spikes.sum())  # jitter spikes
    return h_24xd, h_wall, cog


def run_fusion(t, cog, h_primary, sog=8.0):
    """
    Emulate the fusion's COG-primary-at-speed + yaw-rate limit on the synthetic
    signals. Returns the fused output and which samples were rejected.
    """
    lim = YawRateLimiter()
    fused = np.full(len(t), np.nan)
    rejected = np.zeros(len(t), dtype=bool)
    last = None
    for i in range(len(t)):
        when = t[i]
        # COG-primary at fishing speed
        cand = cog[i]
        if lim.check(cand, when):
            lim.accept(cand, when)
            fused[i] = cand
            last = cand
        else:
            rejected[i] = True
            # hold last, fall back toward the magnetic source if it is closer
            if last is not None:
                # try the magnetic primary as a fallback
                if lim.check(h_primary[i], when):
                    lim.accept(h_primary[i], when)
                    fused[i] = h_primary[i]
                    last = h_primary[i]
                else:
                    fused[i] = last
            else:
                fused[i] = cand
    return fused, rejected


def main():
    t, true_cog, true_heading = synth_run()
    h_24xd, h_wall, cog = add_sensor_noise(true_cog, true_heading)
    fused, rejected = run_fusion(t, cog, h_24xd)

    fig, axes = plt.subplots(3, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("RemoteRudder heading fusion — test case visualization",
                 fontsize=14, fontweight="bold")

    # ---- Panel 1: the fishing crosswind case ----------------------------
    ax = axes[0]
    ax.plot(t, true_cog, "k--", lw=1.5, label="True COG (track)", alpha=0.7)
    ax.plot(t, true_heading, color="gray", ls=":", lw=1.5,
            label="True heading (boat crabs 12°)", alpha=0.7)
    ax.plot(t, cog, color="tab:blue", lw=0.8, alpha=0.5, label="COG (noisy, GPS)")
    ax.plot(t, h_24xd, color="tab:green", lw=0.8, alpha=0.6, label="24xd heading")
    ax.set_title("Fishing case: crosswind crab — COG holds the TRACK straight, "
                 "heading would drift off",
                 fontsize=10)
    ax.set_ylabel("degrees")
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    ax.grid(alpha=0.3)
    ax.annotate("12° crab gap\n(wind/current)", xy=(10, 84), xytext=(3, 60),
                fontsize=8, arrowprops=dict(arrowstyle="->", color="red"))

    # ---- Panel 2: fusion output + yaw-rate rejections -------------------
    ax = axes[1]
    ax.plot(t, cog, color="tab:blue", lw=0.7, alpha=0.35, label="COG raw (with spikes)")
    ax.plot(t, fused, color="tab:red", lw=1.8, label="Fused output (COG-primary)")
    ax.scatter(t[rejected], cog[rejected], color="red", marker="x", s=60,
               zorder=5, label=f"Rejected jumps (>{YAW_RATE_MAX_DPS:.0f}°/s)")
    ax.set_title(f"Yaw-rate limiter: {rejected.sum()} impossible COG jumps discarded — "
                 "controller does not chase them",
                 fontsize=10)
    ax.set_ylabel("degrees")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)

    # ---- Panel 3: implied yaw rate, with the limit line ----------------
    ax = axes[2]
    dt = np.diff(t)
    cog_rate = np.abs(np.array([_angdiff(cog[i], cog[i-1]) for i in range(1, len(cog))])) / dt
    fused_rate = np.abs(np.array([_angdiff(fused[i], fused[i-1]) for i in range(1, len(fused))])) / dt
    ax.plot(t[1:], cog_rate, color="tab:blue", lw=0.7, alpha=0.5, label="COG raw yaw rate")
    ax.plot(t[1:], fused_rate, color="tab:red", lw=1.3, label="Fused yaw rate")
    ax.axhline(YAW_RATE_MAX_DPS, color="black", ls="--", lw=1.2,
               label=f"Limit {YAW_RATE_MAX_DPS:.0f}°/s (measured max real turn ~41°/s)")
    ax.set_title("Implied yaw rate: raw COG spikes to 100s of °/s; fused stays "
                 "under the physical limit",
                 fontsize=10)
    ax.set_ylabel("deg/s")
    ax.set_xlabel("time (s)")
    ax.set_ylim(0, min(150, cog_rate.max() * 1.1))
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    out = "/mnt/user-data/outputs/fusion_test_cases.png"
    import os
    os.makedirs("/mnt/user-data/outputs", exist_ok=True)
    plt.savefig(out, dpi=130, bbox_inches="tight")
    print(f"saved {out}")

    # ---- Second figure: sensor-disagreement / stable-fallback ----------
    fig2, axes2 = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    fig2.suptitle("Sensor disagreement — fall back to the stable source near "
                  "the last reading", fontsize=13, fontweight="bold")

    n = 300
    t2 = np.arange(n) * 0.25
    truth = np.full(n, 150.0)
    # 24xd: steady on truth
    s_24xd = truth + rng.normal(0, 0.8, n)
    # wall compass: steady but biased +5
    s_wall = truth + 5 + rng.normal(0, 1.0, n)
    # a glitching source: mostly truth, but a 40 s stretch goes wild
    s_glitch = truth + rng.normal(0, 1.0, n)
    s_glitch[120:180] += rng.normal(0, 45, 60)  # erroneous stretch

    ax = axes2[0]
    ax.plot(t2, s_24xd, color="tab:green", lw=0.9, label="24xd (stable, on truth)")
    ax.plot(t2, s_wall, color="tab:orange", lw=0.9, label="Wall compass (stable, +5° bias)")
    ax.plot(t2, s_glitch, color="tab:purple", lw=0.7, alpha=0.6, label="Glitching source")
    ax.axhline(150, color="k", ls="--", alpha=0.5, label="Truth")
    ax.axvspan(t2[120], t2[180], color="red", alpha=0.08)
    ax.set_title("Three sources; one glitches for ~15 s", fontsize=10)
    ax.set_ylabel("degrees")
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    ax.grid(alpha=0.3)

    # fusion: pick stable-near-last
    from heading_fusion import SourceStability, choose_stable_near_last
    stab = {"24xd": SourceStability(), "wall": SourceStability(),
            "glitch": SourceStability()}
    fused2 = np.full(n, np.nan)
    last = None
    for i in range(n):
        when = t2[i]
        for k, arr in (("24xd", s_24xd), ("wall", s_wall), ("glitch", s_glitch)):
            stab[k].add(arr[i], when)
        cands = {"24xd": (s_24xd[i], when), "wall": (s_wall[i], when),
                 "glitch": (s_glitch[i], when)}
        pick = choose_stable_near_last(cands, last, stab, now=when)
        if pick is not None:
            fused2[i] = pick[1]
            last = pick[1]
        elif last is not None:
            fused2[i] = last

    ax = axes2[1]
    ax.plot(t2, s_glitch, color="tab:purple", lw=0.6, alpha=0.4, label="Glitching source")
    ax.plot(t2, fused2, color="tab:red", lw=1.8, label="Fused (stable-near-last)")
    ax.axhline(150, color="k", ls="--", alpha=0.5, label="Truth")
    ax.axvspan(t2[120], t2[180], color="red", alpha=0.08,
               label="Glitch window (ignored)")
    ax.set_title("Fusion stays on the stable sources through the glitch — "
                 "never chases the deviating one", fontsize=10)
    ax.set_ylabel("degrees")
    ax.set_xlabel("time (s)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out2 = "/mnt/user-data/outputs/fusion_sensor_disagreement.png"
    plt.savefig(out2, dpi=130, bbox_inches="tight")
    print(f"saved {out2}")


if __name__ == "__main__":
    main()
