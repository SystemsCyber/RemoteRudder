"""
Offline replay harness: run the filter against recorded candump logs.

Usage:
    python3 replay.py <logfile> [--plot out.png] [--no-cog] [--no-compass]

Produces:
  * per-sensor accept/gate statistics
  * delta (compass offset) convergence trace
  * a validation metric: during high-speed straight segments COG is
    trustworthy, so |filtered heading - COG| there is a proxy for truth error.
"""

import argparse
import math
import sys

import candecode as cd
from heading_kf import HeadingKF, Params


def angdiff_deg(a, b):
    return math.degrees(cd.wrap_pi(math.radians(a - b)))


def run(path, use_cog=True, use_compass=True, params=Params, limit=None):
    obs = cd.load_log(path, limit=limit)

    srcs = {}
    for o in obs:
        srcs.setdefault(o.kind, set()).add(o.src)
    compass_srcs = sorted(srcs.get("heading", []))
    gps_srcs = sorted(srcs.get("cog", []))
    gyro_srcs = sorted(srcs.get("rot", []))

    kf = HeadingKF(params)
    kf.register_defaults(
        compass_src=compass_srcs[0] if (compass_srcs and use_compass) else -1,
        gps_srcs=tuple(gps_srcs) if use_cog else (),
        gyro_srcs=tuple(gyro_srcs),
    )

    trace = []
    last_cog = None
    last_compass = None
    for o in obs:
        if o.kind == "cog":
            last_cog = math.degrees(o.value)
        if o.kind == "heading":
            last_compass = math.degrees(o.value)
        kf.process(o)
        if not kf.initialised:
            continue
        trace.append({
            "t": o.t,
            "psi": kf.heading_deg,
            "rate": kf.rate_dps,
            "delta": kf.delta_deg, "iron": kf.iron_amplitude_deg,
            "sigma": kf.psi_sigma_deg,
            "speed": kf.last_speed or 0.0,
            "cog": last_cog,
            "compass": last_compass,
            "aligning": kf.aligning,
        })
    return kf, trace, {"compass": compass_srcs, "cog": gps_srcs, "rot": gyro_srcs}


def validate(trace, speed_thresh=4.5, rate_thresh=3.0):
    """During fast + straight running, COG is near-truth. Score against it."""
    errs = []
    for r in trace:
        if r["cog"] is None:
            continue
        if r["speed"] >= speed_thresh and abs(r["rate"]) <= rate_thresh:
            errs.append(abs(angdiff_deg(r["psi"], r["cog"])))
    if not errs:
        return None
    errs.sort()
    return {
        "n": len(errs),
        "mean": sum(errs) / len(errs),
        "median": errs[len(errs) // 2],
        "p95": errs[int(0.95 * (len(errs) - 1))],
        "max": errs[-1],
    }


def report(path, kf, trace, srcs):
    print("=" * 74)
    print(path.split("/")[-1])
    print("=" * 74)
    print(f"sources: compass={srcs['compass']} cog={srcs['cog']} gyro={srcs['rot']}")
    print(f"samples: {len(trace)}   duration: {trace[-1]['t']:.1f}s"
          if trace else "no output")
    print()

    print(f"{'sensor':10s} {'accept':>8s} {'gated':>7s} {'invalid':>8s} "
          f"{'gate%':>7s} {'healthy':>8s}")
    for row in kf.health_report():
        print(f"{row['name']:10s} {row['accepted']:8d} {row['gated']:7d} "
              f"{row['invalid']:8d} {row['gate_pct']:6.1f}% {str(row['healthy']):>8s}")
    print()

    if trace:
        d = [r["delta"] for r in trace]
        print(f"delta (compass offset): start {d[0]:+.2f} deg  "
              f"end {d[-1]:+.2f} deg  range [{min(d):+.2f}, {max(d):+.2f}]")
        s = [r["sigma"] for r in trace]
        print(f"heading sigma: final {s[-1]:.2f} deg  max {max(s):.2f} deg")
        n_align = sum(1 for r in trace if r["aligning"])
        print(f"alignment-boost active: {n_align} samples "
              f"({100.0*n_align/len(trace):.1f}%)")

    v = validate(trace)
    print()
    if v:
        print(f"validation vs COG on fast straight segments (n={v['n']}):")
        print(f"   mean {v['mean']:.2f} deg   median {v['median']:.2f} deg   "
              f"p95 {v['p95']:.2f} deg   max {v['max']:.2f} deg")
    else:
        print("validation: no fast straight segments in this log")
    print()


def plot(trace, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in trace]
    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)

    ax[0].plot(t, [r["compass"] for r in trace], ".", ms=1,
               color="#c44", label="compass (magnetic)", alpha=0.5)
    ax[0].plot(t, [r["cog"] for r in trace], ".", ms=2,
               color="#4a4", label="COG", alpha=0.6)
    ax[0].plot(t, [r["psi"] for r in trace], "-", lw=1.1,
               color="#248", label="KF heading (true)")
    ax[0].set_ylabel("heading (deg)")
    ax[0].legend(loc="upper right", fontsize=8, ncol=3)
    ax[0].grid(alpha=0.3)

    ax[1].plot(t, [r["delta"] for r in trace], color="#84c", lw=1.2)
    ax[1].axhline(8.0, ls="--", color="#999", lw=0.8)
    ax[1].set_ylabel("delta (deg)")
    ax[1].set_title("compass-to-true offset; dashed = nominal CO declination",
                    fontsize=8, loc="left")
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, [r["rate"] for r in trace], color="#c73", lw=0.9)
    ax[2].set_ylabel("yaw rate (deg/s)")
    ax[2].grid(alpha=0.3)

    ax[3].plot(t, [r["speed"] * 2.23694 for r in trace], color="#393", lw=0.9)
    ax[3].axhline(Params.COG_MIN_SPEED * 2.23694, ls="--", color="#c44", lw=0.8)
    ax[3].set_ylabel("SOG (mph)")
    ax[3].set_xlabel("time (s)")
    ax[3].set_title("dashed = COG rejection threshold", fontsize=8, loc="left")
    ax[3].grid(alpha=0.3)

    for a in ax:
        for r0, r1 in _align_spans(trace):
            a.axvspan(r0, r1, color="#ffd", alpha=0.45, zorder=0)

    fig.tight_layout()
    fig.savefig(out, dpi=115)
    print(f"wrote {out}")


def _align_spans(trace):
    spans, start = [], None
    for r in trace:
        if r["aligning"] and start is None:
            start = r["t"]
        elif not r["aligning"] and start is not None:
            spans.append((start, r["t"]))
            start = None
    if start is not None:
        spans.append((start, trace[-1]["t"]))
    return spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--plot")
    ap.add_argument("--no-cog", action="store_true")
    ap.add_argument("--no-compass", action="store_true")
    ap.add_argument("--limit", type=int)
    a = ap.parse_args()

    kf, trace, srcs = run(a.log, use_cog=not a.no_cog,
                          use_compass=not a.no_compass, limit=a.limit)
    report(a.log, kf, trace, srcs)
    if a.plot and trace:
        plot(trace, a.plot)


if __name__ == "__main__":
    main()
