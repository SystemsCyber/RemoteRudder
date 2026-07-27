"""
Fit the compass hard-iron model from a calibration run.

Solves  compass_error(psi) = d0 + ds*sin(psi) + dc*cos(psi)
by least squares, using COG as the truth reference during segments where COG
is trustworthy: fast, straight, steady.

Intended input: a slow full-360 circle at planing speed. See CALIBRATION.md.

Usage:
    python3 fit_iron.py <log> [--min-speed 4.5] [--max-rate 2.0]
"""

import argparse
import bisect
import math

import numpy as np

import candecode as cd


def gather_pairs(path, min_speed=4.5, max_rate_dps=2.0, window=2.0):
    """Return (compass_rad, offset_rad) pairs from straight, fast segments."""
    obs = cd.load_log(path)
    H = [(o.t, o.value) for o in obs if o.kind == "heading"]
    C = [(o.t, o.value, o.extra.get("sog")) for o in obs if o.kind == "cog"]
    if not H or not C:
        return [], (len(H), len(C))

    ht = [h[0] for h in H]
    hv = [h[1] for h in H]
    pairs = []
    for t, cog, sog in C:
        if sog is None or sog < min_speed:
            continue
        i0 = bisect.bisect(ht, t - window)
        i1 = bisect.bisect(ht, t + window)
        if i1 - i0 < 8:
            continue
        seg = np.unwrap(np.array(hv[i0:i1]))
        span = ht[i1 - 1] - ht[i0]
        if span <= 0:
            continue
        rate = abs(seg[-1] - seg[0]) / span
        if rate > math.radians(max_rate_dps):
            continue
        psi_mag = seg.mean()
        pairs.append((cd.wrap_2pi(psi_mag), cd.wrap_pi(cog - psi_mag)))
    return pairs, (len(H), len(C))


def fit(pairs):
    """Least-squares fit of d0, ds, dc. Returns dict with coefficients."""
    psi = np.array([p[0] for p in pairs])
    off = np.array([p[1] for p in pairs])
    M = np.column_stack([np.ones_like(psi), np.sin(psi), np.cos(psi)])
    coef, *_ = np.linalg.lstsq(M, off, rcond=None)
    resid = off - M @ coef
    return {
        "d0": coef[0], "ds": coef[1], "dc": coef[2],
        "amplitude": math.hypot(coef[1], coef[2]),
        "resid_std": float(resid.std()),
        "n": len(pairs),
    }


def coverage(pairs, nbins=12):
    """How much of the compass circle the run actually covers."""
    hist = [0] * nbins
    for psi, _ in pairs:
        hist[int(psi / (2 * math.pi) * nbins) % nbins] += 1
    filled = sum(1 for h in hist if h >= 5)
    return hist, filled, nbins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--min-speed", type=float, default=4.5)
    ap.add_argument("--max-rate", type=float, default=2.0)
    a = ap.parse_args()

    pairs, (nh, nc) = gather_pairs(a.log, a.min_speed, a.max_rate)
    print(f"compass frames: {nh}   COG frames: {nc}")
    print(f"usable straight+fast pairs: {len(pairs)}")
    if len(pairs) < 30:
        print("\nNOT ENOUGH DATA to fit. Need a dedicated calibration circle;"
              "\nsee CALIBRATION.md for the required run profile.")
        return

    hist, filled, nbins = coverage(pairs)
    print(f"heading coverage: {filled}/{nbins} sectors populated")
    print("  " + " ".join(f"{h:4d}" for h in hist))
    if filled < nbins * 0.75:
        print("\nWARNING: partial heading coverage. The fit will be")
        print("underdetermined and should not be trusted as final.")

    r = fit(pairs)
    print()
    print(f"d0 = {math.degrees(r['d0']):+.3f} deg")
    print(f"ds = {math.degrees(r['ds']):+.3f} deg")
    print(f"dc = {math.degrees(r['dc']):+.3f} deg")
    print(f"iron amplitude = {math.degrees(r['amplitude']):.2f} deg")
    print(f"residual std   = {math.degrees(r['resid_std']):.2f} deg")
    print()
    print("Paste into Params to seed the filter:")
    print(f"    SEED_D0 = math.radians({math.degrees(r['d0']):.4f})")
    print(f"    SEED_DS = math.radians({math.degrees(r['ds']):.4f})")
    print(f"    SEED_DC = math.radians({math.degrees(r['dc']):.4f})")


if __name__ == "__main__":
    main()
