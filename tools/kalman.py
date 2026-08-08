"""Attitude estimation with a quaternion error-state Kalman filter (MEKF).

State: attitude error dθ (3) + gyro bias b (3). Gyro in rad/s, accel in m/s².

    f = Kalman()
    f.init_from_accel(ax, ay, az)
    f.update(gx, gy, gz, ax, ay, az, dt)
    f.q                                    # [w, x, y, z], body -> world

Accel is a valid gravity reference only while |a| ~ g. On this buoy we have experienced
|a| ranges up to ~3000 mg.

    R = R0·(DT_REF/dt)·(1 + λ_a·((|a|-g)/g)² + λ_ω·(|ω|/ω0)²)

R inflates during slams and fast rolling, leaving attitude to the gyro; in calm
stretches it drops back and accel anchors the slow drift. This matters because the
observed noise floor sits at 0.08-0.20 Hz, where the gyro is roughly 35x better
than the angle error needed to explain that floor.

RATE INVARIANCE. The DT_REF/dt factor is what keeps the filter's bandwidth
independent of the logging rate. Process noise is Q = σ_g²·dt, so a fixed R would
give a steady-state gain K ~ √(σ_g²·dt/R) and a time constant τ = dt/K = √(dt·R/σ_g²)
— proportional to √dt. Measured on the real filter, the crossover moved from
1.78 Hz at 50 Hz logging to 2.51 Hz at 100 Hz, a factor 1.416 against √2 = 1.414.
Two sessions logged at different rates were therefore being compared through two
filters of different bandwidth, which is not a property of the sea state.

Scaling R with 1/dt cancels it exactly: τ = √(dt·(R0·DT_REF/dt)/σ_g²) = √(DT_REF·R0)/σ_g,
with no dt left. R is then really a noise DENSITY rather than a per-sample
variance, which is also the physically honest reading: each logged row is a window
average, and a shorter window averages away less noise.

Defaults tuned against the Skjærhalden sessions (noise floor, mg/√Hz). The sweeps
were run at 50 Hz logging, which is why DT_REF is 20 ms — at that rate the factor
is exactly 1 and the numbers below still apply unchanged:
    R0    1e-7 -> 8.36,  1e-6 -> 6.34,  1e-5 -> 4.30,  1e-4 -> 4.85,  1e-3 -> 6.24
          Optimum near 1e-5. Below it accel is trusted blindly and wave
          acceleration tilts the attitude; above it the gyro drifts.
    λ_a   0 -> 4.299,  25 -> 4.315,  100 -> 4.535
          No effect: slams are too brief to land in 0.08-0.20 Hz. Default 0.
    λ_ω   0 -> 6.543,  2 -> 4.299,  10 -> 4.653
          The whole gain (-34%). Gravity is least reliable when rotation is fast,
          since attitude then changes within one sample.

Sign conventions (getting these wrong is the classic trap):
    q rotates body -> world:    v_world = R(q)·v_body
    error state is body-frame:  R_true  = R(q)·exp([dθ]x)
    accel measures specific force: f_body = R(q)^T·[0,0,g]   (az ~ +1 g upright)

LINEAR-ACCELERATION STATE (optional, tau_a is not None). Off by default, so the
six-state filter above is what `Kalman()` still is, bit for bit.

The six-state filter has no model for wave acceleration: it normalises the accel to
a direction and throws the magnitude away, then declares the measurement noisy when
|a| strays from g (the λ_a term). NXP's 12-state filter — the one the SFY buoy runs —
instead carries linear acceleration as three more states and SUBTRACTS its estimate
before the attitude is corrected:

    gErrSeMi = Accel + aSeMi - gSeGyMi          (Adafruit_AHRS_NXPFusion.cpp)

Turning tau_a on ports exactly that idea onto this filter, giving a nine-state
error [dθ, b, a] with a in BODY frame and units of g:

    measurement   z = a_meas / g               (NOT normalised - the magnitude is
                                                the whole point)
    prediction    h = R(q)^T·ẑ + a
    H             [ [R^T ẑ]x | 0 | I ]

a is a decaying first-order process, a <- ca·a, which is what keeps it from
absorbing a steady tilt: only accel that changes faster than tau_a is explained
away as motion, anything slower still corrects the attitude.

RATE INVARIANCE, one deliberate departure from NXP. NXP hard-codes a per-SAMPLE
decay (FCA = 0.5) so its time constant follows the sample rate. That would break
the property this module exists to keep, so tau_a is a TIME and ca = exp(-dt/tau_a)
is derived per step. NXP's FCA at rate fs corresponds to tau_a = 1/(fs·ln 2), i.e.
14.4 ms at 100 Hz — set it there to mimic NXP on a capture of that rate.

Q_a = σ_a²·dt with that decay gives a stationary variance σ_a²·tau_a/2, independent
of dt — the same reasoning as Q_g = σ_g²·dt above.
"""

import math

import numpy as np

from rotation import quat_from_accel, quat_exp, quat_matrix, quat_mul, skew

GRAVITY = 9.80665            # kGravity [m/s²]
SIGMA_G = 0.005              # gyro noise density [rad/s/√Hz] (~0.3 °/s/√Hz)
SIGMA_B = 1.0e-5             # bias random walk [rad/s²/√Hz]
R0 = 1.0e-5                  # base variance of the accel DIRECTION, at DT_REF
DT_REF = 0.020               # rate the tuning was swept at [s] - see RATE INVARIANCE
LAMBDA_A = 0.0               # weight on |a| deviation from 1 g - measured as no-op
LAMBDA_W = 2.0               # weight on |ω| - where the gain is
W0 = 1.0                     # normalisation for |ω| [rad/s]
GATE = 0.0                   # chi² threshold for rejecting a measurement (0 = off)
P0_ANGLE = 5.0               # initial attitude uncertainty [deg]
P0_BIAS = 1.0                # initial bias uncertainty [deg/s]
TAU_A = None                 # linear-accel time constant [s]; None = state off
SIGMA_A = 0.1                # linear-accel driving noise density [g/√Hz]
P0_A = 0.03                  # initial linear-accel uncertainty [g]


class Kalman:
    """Error-state Kalman filter: attitude error dθ (body frame), gyro bias b, and
    - when tau_a is set - body-frame linear acceleration a. See the module docstring."""

    def __init__(self, sigma_g=SIGMA_G, sigma_b=SIGMA_B, r0=R0,
                 lambda_a=LAMBDA_A, lambda_w=LAMBDA_W, w0=W0, gate=GATE,
                 gravity=GRAVITY, dt_ref=DT_REF,
                 tau_a=TAU_A, sigma_a=SIGMA_A, p0_a=P0_A):
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self.b = np.zeros(3)
        # The a-state is what makes this 9 states rather than 6. Keeping n as an
        # attribute rather than branching everywhere lets predict/correct stay one
        # code path - the six-state filter is just the n = 6 case of it.
        self.tau_a = tau_a
        self.n = 9 if tau_a is not None else 6
        self.a = np.zeros(3)             # body-frame linear accel [g], unused if n == 6
        self.P = np.zeros((self.n, self.n))
        p0a = P0_ANGLE * math.pi / 180.0
        p0b = P0_BIAS * math.pi / 180.0
        self.P[:3, :3] = np.eye(3) * (p0a * p0a)
        self.P[3:6, 3:6] = np.eye(3) * (p0b * p0b)
        if self.n == 9:
            self.P[6:, 6:] = np.eye(3) * (p0_a * p0_a)
        self.sg, self.sb, self.r0 = sigma_g, sigma_b, r0
        self.sa = sigma_a
        self.la, self.lw, self.w0 = lambda_a, lambda_w, max(w0, 1e-9)
        self.gate = gate
        self.gravity = gravity
        self.dt_ref = dt_ref
        self.n_gated = 0
        # |ω| and dt from the last predict(). The adaptive R needs both: how fast
        # we are rotating right now, and how long the sample averaged over. Neither
        # is visible to correct() on its own. Seeded so a correct() before any
        # predict() still gives a sane R rather than dividing by zero.
        self._w_norm = 0.0
        self._dt = dt_ref

    @property
    def q(self):
        """Attitude as [w, x, y, z], body -> world."""
        return list(self._q)

    def init_from_accel(self, ax, ay, az):
        """Start near the true attitude rather than at identity, so the settling
        transient stays out of the record. Divided by ω⁴ downstream, such a
        transient would surface as a false low-frequency peak."""
        self._q = np.array(quat_from_accel(ax, ay, az), dtype=np.float64)

    def predict(self, gx, gy, gz, dt):
        """Time update: propagate q and P with the bias-corrected rate.

        The nominal quaternion is propagated exactly, q <- q (x) exp(w·dt), while
        the covariance is carried on the error state:

            dθ/dt = -[w]x·dθ - db
            db/dt =            w_b

        That split is the point of an error-state formulation. The quaternion is
        nonlinear and norm-constrained, but the error is small and lives in a
        3-dimensional tangent space where the ordinary linear Kalman equations
        hold - hence six states rather than seven, and no norm bookkeeping in P.

        The minus sign on db is what makes bias observable: a bias error produces
        an angle error that grows with time, which is what distinguishes it from a
        plain angle error."""
        if dt <= 0.0:
            return self.q
        w = np.array([gx, gy, gz], dtype=np.float64)
        self._w_norm = float(np.linalg.norm(w))
        self._dt = dt
        wc = w - self.b
        self._q = quat_mul(self._q, quat_exp(wc * dt))
        self._q /= np.linalg.norm(self._q)
        F = np.eye(self.n)
        F[:3, :3] -= skew(wc) * dt
        F[:3, 3:6] = -np.eye(3) * dt
        if self.n == 9:
            # Linear accel decays towards zero with time constant tau_a. Derived
            # from dt rather than fixed per sample - see RATE INVARIANCE.
            ca = math.exp(-dt / self.tau_a)
            self.a *= ca
            F[6:, 6:] = np.eye(3) * ca
        self.P = F @ self.P @ F.T
        self.P[:3, :3] += np.eye(3) * (self.sg * self.sg * dt)
        self.P[3:6, 3:6] += np.eye(3) * (self.sb * self.sb * dt)
        if self.n == 9:
            self.P[6:, 6:] += np.eye(3) * (self.sa * self.sa * dt)
        return self.q

    def r_accel(self, acc_norm):
        """Adaptive measurement variance - see the R formula in the module docstring.

        The dt_ref/dt factor makes the filter bandwidth independent of the logging
        rate; without it the crossover would scale as 1/√dt."""
        dev = (acc_norm - self.gravity) / self.gravity
        wn = self._w_norm / self.w0
        rate = self.dt_ref / self._dt if self._dt > 0.0 else 1.0
        return self.r0 * rate * (1.0 + self.la * dev * dev + self.lw * wn * wn)

    def correct(self, ax, ay, az):
        """Measurement update against the accel.

        Six-state: against the accel DIRECTION. Using the direction keeps the three
        axes properly coupled, and yaw drops out as unobservable on its own: H has
        rank 2, so only the two directions accel actually sees are updated.

        Nine-state: against the accel SCALED BY g, with the linear-accel estimate
        subtracted. H then has rank 3 - but the third direction updates a, not the
        attitude, so yaw stays unobservable exactly as before.

        Separate from predict() so the measurement can be skipped without halting
        the time update. The adaptive R does this gradually; gate > 0 does it
        abruptly, rejecting innovations that are improbable given S."""
        a = np.array([ax, ay, az], dtype=np.float64)
        an = float(np.linalg.norm(a))
        if an <= 1e-9:
            return self.q
        g_body = quat_matrix(self._q).T @ np.array([0.0, 0.0, 1.0])

        H = np.zeros((3, self.n))
        H[:, :3] = skew(g_body)             # h_true ~ h_hat + [h_hat]x·dθ
        if self.n == 9:
            # Scale by g, do NOT normalise: dividing by |a| would throw away exactly
            # the magnitude the a-state exists to explain. H's a-block is the
            # identity because a enters the prediction additively.
            z = a / self.gravity
            h = g_body + self.a
            H[:, 6:] = np.eye(3)
        else:
            z = a / an
            h = g_body
        y = z - h
        Rm = np.eye(3) * self.r_accel(an)

        S = H @ self.P @ H.T + Rm
        if self.gate > 0.0:
            try:
                if float(y @ np.linalg.solve(S, y)) > self.gate:
                    self.n_gated += 1
                    return self.q          # improbable innovation -> reject
            except np.linalg.LinAlgError:
                return self.q
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return self.q
        dx = K @ y
        self._q = quat_mul(self._q, quat_exp(dx[:3]))
        self._q /= np.linalg.norm(self._q)
        self.b = self.b + dx[3:6]
        if self.n == 9:
            self.a = self.a + dx[6:]
        IKH = np.eye(self.n) - K @ H
        # Joseph form: keeps P symmetric and positive definite even when K is far
        # from optimal, which it is every time R has just inflated.
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T
        return self.q

    def update(self, gx, gy, gz, ax, ay, az, dt):
        """One full step. Order is binding: the correction must act on the
        predicted state, and the adaptive R reads the |ω| predict() just set."""
        if dt <= 0.0:
            return self.q
        self.predict(gx, gy, gz, dt)
        return self.correct(ax, ay, az)
