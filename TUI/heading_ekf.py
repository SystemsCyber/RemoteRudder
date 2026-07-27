"""
Heading EKF: fuse compass heading, compass yaw rate, COG, and (optional) an
external gyro into a single smooth heading estimate.

The key idea (the operator's): a compass with a constant bias still has an
accurate DERIVATIVE. Declination and hard-iron shift the heading but a real
10-degree yaw still moves the compass ~10 degrees, so the compass YAW RATE is a
good measurement even when the compass heading is not. This filter uses the
compass derivative to keep yaw responsive and smooth, while slower absolute
references (compass heading, COG) correct the heading itself.

Two caveats the design handles:
  * soft-iron makes the compass yaw rate slightly heading-dependent (a real
    turn reads compressed or stretched). The 3-state variant learns a yaw-rate
    bias online to absorb the residual; the 2-state trusts the 24xd
    calibration instead.
  * electrical transients spike the derivative. The caller gates those with the
    yaw-rate limiter (heading_fusion.YawRateLimiter) before feeding this.

Angles are handled on the circle: the heading state wraps, and all innovations
use shortest-arc differences.

Two variants, so they can be compared on real/synthetic data:
  HeadingEKF2  state = [heading, yaw_rate]
  HeadingEKF3  state = [heading, yaw_rate, yaw_bias]

Both expose the same interface:
  predict(dt)
  update_heading(meas_deg, R)          # absolute heading (compass or COG)
  update_yaw_rate(meas_dps, R)         # a yaw-rate measurement (compass deriv,
                                       #   gyro, node)
  heading  -> float (deg, 0..360)
  yaw_rate -> float (deg/s)
  sigma    -> float (deg, sqrt of heading variance)
"""

from __future__ import annotations

import math
from typing import List


def _wrap360(d: float) -> float:
    return d % 360.0


def _angdiff(a: float, b: float) -> float:
    """Shortest signed a-b in (-180, 180]."""
    d = (a - b) % 360.0
    if d > 180:
        d -= 360
    return d


class _EKFBase:
    """Shared machinery. Subclasses define n, F, and the measurement models."""

    n = 2

    def __init__(self):
        # state and covariance set by subclass
        self.x: List[float] = []
        self.P: List[List[float]] = []
        # process noise per unit time (variance). Tuned defaults; the compare
        # script sweeps these.
        self.q_heading = 0.01     # deg^2/s  -- heading itself is driven by yaw
        self.q_yawrate = 4.0      # (deg/s)^2/s -- yaw rate can change (turns)
        self.q_bias = 0.001       # (deg/s)^2/s -- bias drifts slowly (3-state)

    # -- helpers -----------------------------------------------------------

    @property
    def heading(self) -> float:
        return _wrap360(self.x[0])

    @property
    def yaw_rate(self) -> float:
        return self.x[1]

    @property
    def sigma(self) -> float:
        return math.sqrt(max(self.P[0][0], 0.0))

    # -- predict -----------------------------------------------------------

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        # State transition: heading += yaw_rate*dt; yaw_rate persists;
        # bias persists (3-state).
        self.x[0] = self.x[0] + self.x[1] * dt
        # F (Jacobian of the transition)
        F = self._F(dt)
        # P = F P F^T + Q
        self.P = _add(_matmul(_matmul(F, self.P), _transpose(F)), self._Q(dt))

    def _F(self, dt):
        raise NotImplementedError

    def _Q(self, dt):
        raise NotImplementedError

    # -- generic scalar update ---------------------------------------------

    def _update_scalar(self, H, innovation, R):
        """
        Standard EKF scalar update. H is a row (list length n), innovation is
        the measurement residual (already wrapped for angles), R is scalar
        measurement variance.
        """
        # S = H P H^T + R
        HP = [sum(H[j] * self.P[j][i] for j in range(self.n)) for i in range(self.n)]
        S = sum(HP[i] * H[i] for i in range(self.n)) + R
        if S <= 0:
            return
        # K = P H^T / S
        PHt = [sum(self.P[i][j] * H[j] for j in range(self.n)) for i in range(self.n)]
        K = [PHt[i] / S for i in range(self.n)]
        # x += K * innovation
        for i in range(self.n):
            self.x[i] += K[i] * innovation
        # P = (I - K H) P
        KH = [[K[i] * H[j] for j in range(self.n)] for i in range(self.n)]
        ImKH = [[(1.0 if i == j else 0.0) - KH[i][j] for j in range(self.n)]
                for i in range(self.n)]
        self.P = _matmul(ImKH, self.P)

    # -- measurement updates ----------------------------------------------

    def update_heading(self, meas_deg: float, R: float) -> None:
        """Absolute heading measurement (compass heading or COG)."""
        H = [1.0] + [0.0] * (self.n - 1)
        innovation = _angdiff(meas_deg, self.x[0])
        self._update_scalar(H, innovation, R)

    def update_yaw_rate(self, meas_dps: float, R: float) -> None:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# 2-state: [heading, yaw_rate]
# ---------------------------------------------------------------------------


class HeadingEKF2(_EKFBase):
    n = 2

    def __init__(self, heading0: float = 0.0):
        super().__init__()
        self.x = [heading0, 0.0]
        self.P = [[100.0, 0.0], [0.0, 25.0]]  # initial uncertainty

    def _F(self, dt):
        return [[1.0, dt], [0.0, 1.0]]

    def _Q(self, dt):
        return [[self.q_heading * dt, 0.0], [0.0, self.q_yawrate * dt]]

    def update_yaw_rate(self, meas_dps: float, R: float) -> None:
        # measures yaw_rate directly
        H = [0.0, 1.0]
        innovation = meas_dps - self.x[1]
        self._update_scalar(H, innovation, R)


# ---------------------------------------------------------------------------
# 3-state: [heading, yaw_rate, yaw_bias]
#
# yaw_bias absorbs the residual soft-iron / scale error in the compass yaw
# rate. The compass yaw-rate measurement is modeled as (yaw_rate + yaw_bias),
# so the filter learns the bias online and subtracts it. A gyro/node yaw rate
# (unbiased) measures yaw_rate alone, which is what lets the filter separate
# the two -- with only the compass, bias and rate are unobservable together, so
# the bias drifts slowly (small q_bias) and mostly captures steady scale error.
# ---------------------------------------------------------------------------


class HeadingEKF3(_EKFBase):
    n = 3

    def __init__(self, heading0: float = 0.0):
        super().__init__()
        self.x = [heading0, 0.0, 0.0]
        self.P = [[100.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 4.0]]

    def _F(self, dt):
        return [[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def _Q(self, dt):
        return [[self.q_heading * dt, 0.0, 0.0],
                [0.0, self.q_yawrate * dt, 0.0],
                [0.0, 0.0, self.q_bias * dt]]

    def update_yaw_rate_compass(self, meas_dps: float, R: float) -> None:
        """Compass-derived yaw rate: measures (yaw_rate + yaw_bias)."""
        H = [0.0, 1.0, 1.0]
        innovation = meas_dps - (self.x[1] + self.x[2])
        self._update_scalar(H, innovation, R)

    def update_yaw_rate_gyro(self, meas_dps: float, R: float) -> None:
        """Unbiased gyro/node yaw rate: measures yaw_rate alone."""
        H = [0.0, 1.0, 0.0]
        innovation = meas_dps - self.x[1]
        self._update_scalar(H, innovation, R)

    # default update_yaw_rate treats it as the compass (biased) source
    def update_yaw_rate(self, meas_dps: float, R: float) -> None:
        self.update_yaw_rate_compass(meas_dps, R)

    @property
    def yaw_bias(self) -> float:
        return self.x[2]


# ---------------------------------------------------------------------------
# tiny matrix helpers (no numpy dependency in the runtime path)
# ---------------------------------------------------------------------------


def _matmul(A, B):
    n, m, p = len(A), len(B), len(B[0])
    return [[sum(A[i][k] * B[k][j] for k in range(m)) for j in range(p)]
            for i in range(n)]


def _transpose(A):
    return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]


def _add(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
