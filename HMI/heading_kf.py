"""
Heading Kalman filter for RemoteRudder.

State (all SI, angles in radians):
    x[0] = psi    true heading
    x[1] = r      yaw rate
    x[2] = b      gyro bias
    x[3] = d0     constant compass offset (declination + hard-iron constant)
    x[4] = ds     one-cycle sine coefficient   (hard iron)
    x[5] = dc     one-cycle cosine coefficient (hard iron)

The compass error model is
    magnetic_reading = psi - (d0 + ds*sin(psi) + dc*cos(psi))
A single scalar offset is NOT sufficient: analysis of the capture set shows
the compass-to-COG offset varying by >20 degrees as a function of heading,
which is the signature of hard iron in the boat, not declination. Fitting a
constant to that makes the estimate wander as the boat turns.

Because the measurement now depends on psi non-linearly, the compass update
uses a locally linearised H (an EKF row) recomputed at each update.

Design notes
------------
* Sequential SCALAR updates. Every sensor contributes one scalar with its own
  H row, R, and validity gate. Adding a sensor = adding a registry entry, not
  touching the filter. This is deliberately the same structure intended for
  the Teensy port, so behaviour can be diffed line-for-line.

* Angle innovations are wrapped to (-pi, pi]. The covariance is never wrapped.

* delta has very small process noise: it must track slow declination/iron
  drift but must NOT chase crab angle. If delta starts moving with wind
  conditions, Q_DELTA is too large.

* COG measurement noise is speed-dependent:
      sigma_cog ~ atan(sigma_pos / (v * dt))
  so trust degrades automatically as the boat slows. Below COG_MIN_SPEED the
  measurement is rejected outright -- at walking pace COG is close to uniform
  on the circle and no variance inflation makes it safe.

* Chi-squared gating gives redundancy management for free: with two GPS units,
  whichever agrees with the propagated state is accepted and the outlier is
  gated out. Per-sensor reject counts are exposed for health monitoring.
"""

import math

from candecode import wrap_pi, wrap_2pi

NSTATES = 6
PSI, RATE, BIAS, D0, DS, DC = 0, 1, 2, 3, 4, 5


# ---------------------------------------------------------------------------
# Tuning parameters
# ---------------------------------------------------------------------------
class Params:
    # --- process noise (continuous-time PSD, per second) ---
    Q_PSI = math.radians(0.05) ** 2     # heading random walk beyond rate integ.
    Q_RATE = math.radians(12.0) ** 2    # how fast yaw rate can change
    Q_BIAS = math.radians(0.02) ** 2    # gyro bias drift
    Q_D0 = math.radians(0.03) ** 2      # constant offset drift: SLOW
    Q_HARM = math.radians(0.005) ** 2   # iron harmonic drift: SLOWER still

    # --- compass ---
    R_COMPASS = math.radians(3.0) ** 2
    GATE_COMPASS = 16.0                 # chi2 on 1 dof; 16 ~ 4 sigma

    # --- COG, speed dependent ---
    GPS_POS_SIGMA = 1.5                 # metres, per-fix horizontal noise
    GPS_DT = 0.5                        # seconds between COG fixes
    COG_MIN_SPEED = 0.7                 # m/s (~1.6 mph) below this: reject
    COG_SIGMA_MIN = math.radians(1.5)
    COG_SIGMA_MAX = math.radians(60.0)
    GATE_COG = 16.0

    # --- rate gyro ---
    R_GYRO = math.radians(0.5) ** 2
    GATE_GYRO = 25.0

    # --- initial covariance ---
    P0_PSI = math.radians(180.0) ** 2
    P0_RATE = math.radians(20.0) ** 2
    P0_BIAS = math.radians(2.0) ** 2
    P0_D0 = math.radians(30.0) ** 2
    P0_HARM = math.radians(8.0) ** 2    # iron amplitude prior; see CALIBRATION.md

    # --- engagement health gate ---
    MAX_PSI_SIGMA = math.radians(5.0)

    # Hard limit on the estimated iron amplitude. Without this the harmonic
    # terms absorb crab angle and grow without bound -- observed running to
    # 45 deg on the capture set, which is not physical for this installation.
    MAX_IRON_AMP = math.radians(15.0)

    # --- delta fast-convergence (straight-run alignment) ---
    ALIGN_MIN_SPEED = 4.5               # m/s (~10 mph)
    ALIGN_MAX_RATE = math.radians(2.0)  # rad/s, "going straight"
    ALIGN_Q_BOOST = 400.0


def sigma_cog(speed_mps, p=Params):
    """Angular noise of a COG fix at a given speed. None => reject."""
    if speed_mps is None or speed_mps < p.COG_MIN_SPEED:
        return None
    travel = max(speed_mps * p.GPS_DT, 1e-3)
    s = math.atan2(p.GPS_POS_SIGMA, travel)
    return min(max(s, p.COG_SIGMA_MIN), p.COG_SIGMA_MAX)


class SensorSpec:
    """One registered measurement channel.

    H         : measurement row, length NSTATES
    r_fn      : callable(obs, filt) -> variance, or None to reject this sample
    gate      : chi-squared rejection threshold
    timeout   : seconds without data before the channel is marked unhealthy
    """

    def __init__(self, name, kind, src, H, r_fn, gate, timeout=2.0, h_fn=None):
        self.name = name
        self.kind = kind
        self.src = src
        self.H = list(H) if H is not None else None
        self.h_fn = h_fn
        self.r_fn = r_fn
        self.gate = gate
        self.timeout = timeout
        self.last_t = None
        self.n_accept = 0
        self.n_reject_gate = 0
        self.n_reject_valid = 0

    def healthy(self, now):
        return self.last_t is not None and (now - self.last_t) <= self.timeout


class HeadingKF:
    def __init__(self, params=Params):
        self.p = params
        self.x = [0.0] * NSTATES
        self.P = [[0.0] * NSTATES for _ in range(NSTATES)]
        self.P[PSI][PSI] = params.P0_PSI
        self.P[RATE][RATE] = params.P0_RATE
        self.P[BIAS][BIAS] = params.P0_BIAS
        self.P[D0][D0] = params.P0_D0
        self.P[DS][DS] = params.P0_HARM
        self.P[DC][DC] = params.P0_HARM

        self.t = None
        self.initialised = False
        self.sensors = {}
        self.last_speed = None
        self.aligning = False

    # -- registry ----------------------------------------------------------
    def register(self, spec):
        self.sensors[(spec.kind, spec.src)] = spec
        return spec

    def register_defaults(self, compass_src=248, gps_srcs=(28,), gyro_srcs=()):
        p = self.p
        # Compass measures MAGNETIC heading:
        #   z = psi - (d0 + ds*sin psi + dc*cos psi)
        # H is state-dependent, so it is built per-update by _compass_H().
        self.register(SensorSpec(
            "compass", "heading", compass_src,
            H=None,                      # dynamic, see _compass_H
            r_fn=lambda o, f: p.R_COMPASS,
            gate=p.GATE_COMPASS, timeout=1.0,
            h_fn="compass",
        ))
        for src in gps_srcs:
            self.register(SensorSpec(
                f"cog{src}", "cog", src,
                H=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                r_fn=self._cog_variance,
                gate=p.GATE_COG, timeout=2.0,
            ))
        for src in gyro_srcs:
            self.register(SensorSpec(
                f"gyro{src}", "rot", src,
                H=[0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
                r_fn=lambda o, f: p.R_GYRO,
                gate=p.GATE_GYRO, timeout=1.0,
            ))

    def _cog_variance(self, obs, _filt):
        s = sigma_cog(obs.extra.get("sog", self.last_speed), self.p)
        return None if s is None else s * s

    # -- prediction --------------------------------------------------------
    def predict(self, dt):
        if dt <= 0:
            return
        p = self.p
        # psi += r*dt  (r is the bias-corrected rate held in state)
        self.x[PSI] = wrap_pi(self.x[PSI] + self.x[RATE] * dt)

        # F = I except d(psi)/d(r) = dt
        # P = F P F^T + Q, expanded for a sparse F to keep the Teensy port cheap
        P = self.P
        for i in range(NSTATES):
            P[PSI][i] += dt * P[RATE][i]
        for i in range(NSTATES):
            P[i][PSI] += dt * P[i][RATE]

        boost = p.ALIGN_Q_BOOST if self.aligning else 1.0
        P[PSI][PSI] += p.Q_PSI * dt
        P[RATE][RATE] += p.Q_RATE * dt
        P[BIAS][BIAS] += p.Q_BIAS * dt
        P[D0][D0] += p.Q_D0 * boost * dt
        P[DS][DS] += p.Q_HARM * boost * dt
        P[DC][DC] += p.Q_HARM * boost * dt
        self._symmetrise()

    # -- compass error model ----------------------------------------------
    def compass_error(self, psi=None):
        """Total compass offset at a given true heading: d0 + ds*sin + dc*cos."""
        psi = self.x[PSI] if psi is None else psi
        return (self.x[D0]
                + self.x[DS] * math.sin(psi)
                + self.x[DC] * math.cos(psi))

    def _compass_H(self):
        """Linearised measurement row for z = psi - error(psi)."""
        psi = self.x[PSI]
        s, c = math.sin(psi), math.cos(psi)
        # d(z)/d(psi) = 1 - (ds*cos psi - dc*sin psi)
        dpsi = 1.0 - (self.x[DS] * c - self.x[DC] * s)
        return [dpsi, 0.0, 0.0, -1.0, -s, -c]

    def _compass_predicted(self):
        return wrap_pi(self.x[PSI] - self.compass_error())

    # -- scalar update -----------------------------------------------------
    def update_scalar(self, H, z, R, gate, angular=True, hx=None):
        """Returns (accepted, normalised_innovation_squared).

        hx overrides the predicted measurement for non-linear channels;
        H must then be the linearisation about the current state.
        """
        Hx = sum(H[i] * self.x[i] for i in range(NSTATES)) if hx is None else hx
        y = wrap_pi(z - Hx) if angular else (z - Hx)

        PH = [sum(self.P[i][j] * H[j] for j in range(NSTATES))
              for i in range(NSTATES)]
        S = R + sum(H[i] * PH[i] for i in range(NSTATES))
        if S <= 0.0:
            return False, float("inf")

        nis = (y * y) / S
        if nis > gate:
            return False, nis

        K = [PH[i] / S for i in range(NSTATES)]
        for i in range(NSTATES):
            self.x[i] += K[i] * y
        for i in range(NSTATES):
            for j in range(NSTATES):
                self.P[i][j] -= K[i] * PH[j]

        self.x[PSI] = wrap_pi(self.x[PSI])
        self.x[D0] = wrap_pi(self.x[D0])
        self._clamp_iron()
        self._symmetrise()
        return True, nis

    def _clamp_iron(self):
        """Keep the harmonic terms inside a physically plausible envelope."""
        amp = math.hypot(self.x[DS], self.x[DC])
        if amp > self.p.MAX_IRON_AMP:
            k = self.p.MAX_IRON_AMP / amp
            self.x[DS] *= k
            self.x[DC] *= k

    def _symmetrise(self):
        P = self.P
        for i in range(NSTATES):
            for j in range(i + 1, NSTATES):
                m = 0.5 * (P[i][j] + P[j][i])
                P[i][j] = P[j][i] = m
            if P[i][i] < 1e-12:
                P[i][i] = 1e-12

    # -- driving -----------------------------------------------------------
    def process(self, obs):
        """Feed one Observation. Returns the SensorSpec used, or None."""
        if obs.kind == "sog":
            self.last_speed = obs.value
            return None

        spec = self.sensors.get((obs.kind, obs.src))
        if spec is None:
            return None

        if self.t is None:
            self.t = obs.t
        dt = obs.t - self.t
        if dt > 0:
            self._update_align_flag()
            self.predict(dt)
            self.t = obs.t

        # Cold start: seed heading from the first usable angular measurement
        if not self.initialised and obs.kind in ("heading", "cog"):
            self._seed(obs, spec)
            spec.last_t = obs.t
            return spec

        R = spec.r_fn(obs, self)
        if R is None:
            spec.n_reject_valid += 1
            return spec

        angular = obs.kind in ("heading", "cog", "attitude")
        if spec.h_fn == "compass":
            H, hx = self._compass_H(), self._compass_predicted()
        else:
            H, hx = spec.H, None
        ok, _nis = self.update_scalar(spec.H if H is None else H,
                                      obs.value, R, spec.gate, angular, hx)
        if ok:
            spec.n_accept += 1
            spec.last_t = obs.t
        else:
            spec.n_reject_gate += 1
        return spec

    def _seed(self, obs, spec):
        """Initialise psi from the first measurement, keeping delta unknown."""
        if obs.kind == "cog":
            if self._cog_variance(obs, self) is None:
                return
            self.x[PSI] = wrap_pi(obs.value)
        else:
            # compass: psi = magnetic + error, error still at prior
            self.x[PSI] = wrap_pi(obs.value + self.compass_error(obs.value))
        self.P[PSI][PSI] = math.radians(20.0) ** 2
        self.initialised = True

    def _update_align_flag(self):
        """Boost delta process noise during fast, straight running."""
        p = self.p
        self.aligning = (
            self.last_speed is not None
            and self.last_speed >= p.ALIGN_MIN_SPEED
            and abs(self.x[RATE]) <= p.ALIGN_MAX_RATE
        )

    # -- outputs -----------------------------------------------------------
    @property
    def heading_deg(self):
        return math.degrees(wrap_2pi(self.x[PSI]))

    @property
    def rate_dps(self):
        return math.degrees(self.x[RATE])

    @property
    def delta_deg(self):
        """Total compass offset at the current heading."""
        return math.degrees(wrap_pi(self.compass_error()))

    @property
    def iron_amplitude_deg(self):
        return math.degrees(math.hypot(self.x[DS], self.x[DC]))

    @property
    def psi_sigma_deg(self):
        return math.degrees(math.sqrt(max(self.P[PSI][PSI], 0.0)))

    def ok_to_engage(self):
        """Health gate for the autopilot. Subsumes a plain RX timeout."""
        if not self.initialised:
            return False, "filter not initialised"
        if math.sqrt(self.P[PSI][PSI]) > self.p.MAX_PSI_SIGMA:
            return False, f"heading sigma {self.psi_sigma_deg:.1f} deg too high"
        healthy = [s for s in self.sensors.values() if s.healthy(self.t or 0.0)]
        if not healthy:
            return False, "no healthy heading sensor"
        return True, "ok"

    def health_report(self):
        rows = []
        for spec in self.sensors.values():
            total = spec.n_accept + spec.n_reject_gate + spec.n_reject_valid
            rows.append({
                "name": spec.name,
                "accepted": spec.n_accept,
                "gated": spec.n_reject_gate,
                "invalid": spec.n_reject_valid,
                "gate_pct": 100.0 * spec.n_reject_gate / total if total else 0.0,
                "healthy": spec.healthy(self.t or 0.0),
            })
        return rows
