import numpy as np
import warnings

class QuinticTrajectory:
    """
    Parameters
    ----------
    t0, tf : float
        Start and end times.
    p0, v0, a0 : float
        Initial position, velocity, acceleration.
    pf, vf, af : float
        Final position, velocity, acceleration.
    """

    def __init__(self, t0: float, tf: float,
                 p0: float, v0: float, a0: float,
                 pf: float, vf: float, af: float):
        if tf <= t0:
            raise ValueError("tf must be greater than t0")

        # store absolute times
        self.t0 = float(t0)
        self.tf = float(tf)
        self.dt = self.tf - self.t0

        if p0 == pf:
            v0, vf, a0, af = 0.0, 0.0, 0.0, 0.0

        dt = self.dt
        T = np.array([
            [1,    0,      0,        0,          0,           0       ],  # p(0)  = p0
            [0,    1,      0,        0,          0,           0       ],  # v(0)  = v0
            [0,    0,      2,        0,          0,           0       ],  # a(0)  = a0
            [1,   dt,   dt**2,    dt**3,      dt**4,       dt**5      ],  # p(dt) = pf
            [0,    1,   2*dt,   3*dt**2,    4*dt**3,     5*dt**4      ],  # v(dt) = vf
            [0,    0,      2,     6*dt,    12*dt**2,    20*dt**3      ],  # a(dt) = af
        ], dtype=float)

        b = np.array([p0, v0, a0, pf, vf, af], dtype=float)
        self._coeffs = np.linalg.solve(T, b)

    def _to_tau(self, t: np.ndarray) -> np.ndarray:
        """Convert absolute time t to local time τ = t - t0, optionally clamped."""
        t = np.asarray(t)
        tau = t - self.t0
        # optional clamp to segment: uncomment if you want strict [0, dt]
        # tau = np.clip(tau, 0.0, self.dt)
        return tau

    def _poly(self, t: np.ndarray) -> np.ndarray:
        """Return position for array-like absolute t."""
        tau = self._to_tau(t)
        a0, a1, a2, a3, a4, a5 = self._coeffs
        return a0 + a1 * tau + a2 * tau ** 2 + a3 * tau ** 3 + a4 * tau ** 4 + a5 * tau ** 5

    def _dpoly(self, t: np.ndarray, order: int) -> np.ndarray:
        """Return the x order derivative (1=vel, 2=acc, 3=jerk) at absolute t."""
        tau = self._to_tau(t)
        a0, a1, a2, a3, a4, a5 = self._coeffs
        if order == 1:
            return a1 + 2 * a2 * tau + 3 * a3 * tau ** 2 + 4 * a4 * tau ** 3 + 5 * a5 * tau ** 4
        elif order == 2:
            return 2 * a2 + 6 * a3 * tau + 12 * a4 * tau ** 2 + 20 * a5 * tau ** 3
        elif order == 3:
            return 6 * a3 + 24 * a4 * tau + 60 * a5 * tau ** 2
        else:
            raise ValueError("Supported orders: 1 (velocity), 2 (acceleration), 3 (jerk)")

    def position(self, t):
        """Position at time(s) t (scalar or array)."""
        return self._poly(np.asarray(t))

    def velocity(self, t):
        """Velocity at time(s) t."""
        return self._dpoly(np.asarray(t), 1)

    def acceleration(self, t):
        """Acceleration at time(s) t."""
        return self._dpoly(np.asarray(t), 2)

    def jerk(self, t):
        """Jerk at time(s) t."""
        return self._dpoly(np.asarray(t), 3)

    def sample(self, dt: float = 0.01):
        """Return uniformly-spaced samples (t, pos, vel, acc)."""
        t = np.arange(self.t0, self.tf + 1e-9, dt)
        return t, self.position(t), self.velocity(t), self.acceleration(t)

    def coeffs(self):
        """Return polynomial coefficients a0…a5."""
        return self._coeffs.copy()
