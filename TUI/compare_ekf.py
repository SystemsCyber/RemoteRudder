#!/usr/bin/env python3
"""
Compare the 2-state and 3-state heading EKFs against the current priority-chain
fusion, on a synthetic fishing scenario with realistic noise.

Produces ekf_comparison.png with panels:
  1. Truth vs the raw sensors (compass heading, compass yaw rate, COG)
  2. 2-state EKF vs 3-state EKF vs priority-chain output, tracking truth
  3. Yaw-rate estimates: how the compass derivative keeps yaw smooth
  4. The 3-state's learned yaw-bias converging to the injected soft-iron scale
     error -- the payoff of the operator's idea
  5. Estimation error of each method

The scenario models the operator's insight: the compass heading is biased
(declination + hard-iron) AND scale-distorted (soft-iron), but its derivative
still reflects real yaw. The 3-state learns the scale error online.
"""

import sys
sys.path.insert(0, '.')

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from heading_ekf import HeadingEKF2, HeadingEKF3, _angdiff

rng = np.random.default_rng(7)


def scenario(n=600, dt=0.1):
    """
    A fishing run: straight track, a couple of deliberate course changes, in a
    crosswind so heading != COG.  Sensor errors modeled realistically:
      compass heading: +32 deg bias (declination+hard-iron), soft-iron scale
                       1.15 (turns read 15% too big), small noise, occasional
                       electrical spike
      compass yawrate: derivative of the above (so it carries the 1.15 scale)
      COG: true track + noise + GPS jitter spikes, only valid at speed
    """
    t = np.arange(n) * dt
    # True yaw rate profile: straight, turn right, straight, turn left, straight
    true_rate = np.zeros(n)
    true_rate[100:160] = 8.0     # 60 samples * 0.1s * 8 = 48 deg right turn
    true_rate[300:340] = -12.0   # left turn
    true_heading = 100.0 + np.cumsum(true_rate) * dt
    crab = 10.0                   # crosswind: COG leads heading
    true_cog = true_heading + crab

    # --- compass heading: bias + soft-iron scale + noise + spikes ---
    BIAS = 32.0
    SOFT_IRON_SCALE = 1.15
    # scale distorts the SHAPE of heading changes relative to a reference; model
    # as scaling the deviation from the run's mean heading.
    ref = true_heading[0]
    compass_heading = ref + (true_heading - ref) * SOFT_IRON_SCALE + BIAS
    compass_heading += rng.normal(0, 1.0, n)
    # electrical spikes
    spike_idx = rng.random(n) < 0.02
    compass_heading[spike_idx] += rng.normal(0, 25, spike_idx.sum())

    # compass yaw rate = derivative of compass heading (carries the scale)
    compass_yawrate = np.gradient(compass_heading, dt)

    # --- COG: track + noise + jitter ---
    cog = true_cog + rng.normal(0, 2.5, n)
    cog_spike = rng.random(n) < 0.10
    cog[cog_spike] += rng.normal(0, 60, cog_spike.sum())

    return dict(t=t, dt=dt, true_heading=true_heading, true_rate=true_rate,
                true_cog=true_cog, compass_heading=compass_heading,
                compass_yawrate=compass_yawrate, cog=cog,
                soft_iron_scale=SOFT_IRON_SCALE, bias=BIAS)


def clamp_rate(v, lim=45.0):
    return abs(v) <= lim


def run_ekf2(s):
    ekf = HeadingEKF2(heading0=s["cog"][0])
    # Start with the boat's own crab: initialize toward COG since that is the
    # track we steer to. Tuning: COG is the trusted absolute reference at speed
    # (unbiased track), so it gets low R; the compass ABSOLUTE heading is
    # biased, so it gets very high R (almost ignored for absolute). The compass
    # DERIVATIVE gets low R -- that is the whole point: trust its rate, not its
    # value.
    out_h = np.zeros(len(s["t"]))
    out_r = np.zeros(len(s["t"]))
    for i in range(len(s["t"])):
        ekf.predict(s["dt"])
        if clamp_rate(s["compass_yawrate"][i]):
            ekf.update_yaw_rate(s["compass_yawrate"][i], R=1.0)
        # COG heading correction (unbiased track). Gate GPS jitter with the
        # yaw-rate check so a COG spike does not yank the heading.
        if clamp_rate(_angdiff(s["cog"][i], ekf.heading) / s["dt"]):
            ekf.update_heading(s["cog"][i], R=8.0)
        out_h[i] = ekf.heading
        out_r[i] = ekf.yaw_rate
    return out_h, out_r


def run_ekf3(s):
    ekf = HeadingEKF3(heading0=s["cog"][0])
    out_h = np.zeros(len(s["t"]))
    out_r = np.zeros(len(s["t"]))
    out_b = np.zeros(len(s["t"]))
    for i in range(len(s["t"])):
        ekf.predict(s["dt"])
        if clamp_rate(s["compass_yawrate"][i]):
            ekf.update_yaw_rate_compass(s["compass_yawrate"][i], R=1.0)
        if clamp_rate(_angdiff(s["cog"][i], ekf.heading) / s["dt"]):
            ekf.update_heading(s["cog"][i], R=8.0)
        # With the 24xd's attitude yaw rate you would add update_yaw_rate_gyro()
        # here; the bias would then separate cleanly. Compass-only, the bias
        # absorbs steady scale error against the COG heading reference over the
        # turns.
        out_h[i] = ekf.heading
        out_r[i] = ekf.yaw_rate
        out_b[i] = ekf.yaw_bias
    return out_h, out_r, out_b


def run_priority_chain(s):
    """Emulate the current fusion: COG-primary at speed with yaw-rate gate."""
    from heading_fusion import YawRateLimiter
    lim = YawRateLimiter()
    out = np.zeros(len(s["t"]))
    last = None
    for i in range(len(s["t"])):
        cand = s["cog"][i]  # at fishing speed, COG-primary
        if lim.check(cand, s["t"][i]):
            lim.accept(cand, s["t"][i])
            out[i] = cand
            last = cand
        else:
            lim.reject()
            out[i] = last if last is not None else cand
    return out


def main():
    s = scenario()
    t = s["t"]
    h2, r2 = run_ekf2(s)
    h3, r3, b3 = run_ekf3(s)
    pc = run_priority_chain(s)

    # errors (vs true COG/track, since that is what we steer to when fishing)
    def err(series):
        return np.array([_angdiff(series[i], s["true_cog"][i]) for i in range(len(t))])

    fig, ax = plt.subplots(5, 1, figsize=(13, 17), sharex=True)
    fig.suptitle("Heading EKF comparison — compass-derivative fusion "
                 "(2-state vs 3-state vs priority chain)",
                 fontsize=14, fontweight="bold")

    # 1. raw sensors
    a = ax[0]
    a.plot(t, s["true_cog"], "k--", lw=1.5, label="True track (COG)", alpha=0.8)
    a.plot(t, s["compass_heading"], color="tab:orange", lw=0.7, alpha=0.6,
           label=f"Compass heading (+{s['bias']:.0f}° bias, {s['soft_iron_scale']:.2f}× scale)")
    a.plot(t, s["cog"], color="tab:blue", lw=0.7, alpha=0.4, label="COG (noisy)")
    a.set_title("Raw sensors: compass is biased AND scale-distorted; COG is noisy", fontsize=10)
    a.set_ylabel("deg")
    a.legend(loc="upper left", fontsize=8, ncol=2)
    a.grid(alpha=0.3)

    # 2. fused outputs
    a = ax[1]
    a.plot(t, s["true_cog"], "k--", lw=1.5, label="True track", alpha=0.8)
    a.plot(t, pc, color="tab:gray", lw=1.0, alpha=0.7, label="Priority chain")
    a.plot(t, h2, color="tab:green", lw=1.5, label="EKF 2-state")
    a.plot(t, h3, color="tab:red", lw=1.5, label="EKF 3-state")
    a.set_title("Fused heading: EKFs are smoother through COG jitter, hold the track", fontsize=10)
    a.set_ylabel("deg")
    a.legend(loc="upper left", fontsize=8, ncol=2)
    a.grid(alpha=0.3)

    # 3. yaw rate estimates
    a = ax[2]
    a.plot(t, s["true_rate"], "k--", lw=1.5, label="True yaw rate", alpha=0.8)
    a.plot(t, r2, color="tab:green", lw=1.0, alpha=0.8, label="EKF 2-state yaw rate")
    a.plot(t, r3, color="tab:red", lw=1.0, alpha=0.8, label="EKF 3-state yaw rate")
    a.set_title("Yaw rate: the compass derivative keeps this responsive and smooth "
                "(the operator's idea)", fontsize=10)
    a.set_ylabel("deg/s")
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    # 4. learned bias (3-state payoff)
    a = ax[3]
    # the injected scale error, expressed as an effective yaw-rate bias, is
    # (scale-1)*true_rate; show the filter's estimate tracking it during turns
    effective_bias = (s["soft_iron_scale"] - 1.0) * s["true_rate"]
    a.plot(t, effective_bias, "k--", lw=1.2, alpha=0.7,
           label="Injected scale error as yaw bias ((scale-1)·rate)")
    a.plot(t, b3, color="tab:red", lw=1.5, label="EKF 3-state learned yaw_bias")
    a.axhline(0, color="gray", lw=0.5)
    a.set_title("3-state learns the compass scale error online — separates true "
                "yaw from soft-iron distortion", fontsize=10)
    a.set_ylabel("deg/s")
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    # 5. error
    a = ax[4]
    a.plot(t, err(pc), color="tab:gray", lw=0.8, alpha=0.7, label=f"Priority chain (RMS {np.sqrt(np.mean(err(pc)**2)):.1f}°)")
    a.plot(t, err(h2), color="tab:green", lw=1.0, label=f"EKF 2-state (RMS {np.sqrt(np.mean(err(h2)**2)):.1f}°)")
    a.plot(t, err(h3), color="tab:red", lw=1.0, label=f"EKF 3-state (RMS {np.sqrt(np.mean(err(h3)**2)):.1f}°)")
    a.axhline(0, color="k", lw=0.5)
    a.set_title("Error vs true track (lower and smoother is better)", fontsize=10)
    a.set_ylabel("deg error")
    a.set_xlabel("time (s)")
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    import os
    os.makedirs("/mnt/user-data/outputs", exist_ok=True)
    out = "/mnt/user-data/outputs/ekf_comparison.png"
    plt.savefig(out, dpi=130, bbox_inches="tight")
    print(f"saved {out}")

    # print a numeric summary for the decision
    print("\nRMS error vs true track:")
    print(f"  priority chain: {np.sqrt(np.mean(err(pc)**2)):.2f} deg")
    print(f"  EKF 2-state:    {np.sqrt(np.mean(err(h2)**2)):.2f} deg")
    print(f"  EKF 3-state:    {np.sqrt(np.mean(err(h3)**2)):.2f} deg")
    print("\nYaw-rate RMS error:")
    print(f"  EKF 2-state: {np.sqrt(np.mean((r2-s['true_rate'])**2)):.2f} deg/s")
    print(f"  EKF 3-state: {np.sqrt(np.mean((r3-s['true_rate'])**2)):.2f} deg/s")


if __name__ == "__main__":
    main()
