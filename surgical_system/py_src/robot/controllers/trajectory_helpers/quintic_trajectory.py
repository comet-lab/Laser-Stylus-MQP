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
        self.t0, self.tf = t0, tf
        
        if p0 == pf:
            # print("p0:", p0, "pf:", pf)
            # If initial and final positions are equal, set velocities and accelerations to zero
            # warnings.warn("Initial and final positions are equal. Setting in zero velocity and acceleration.")
            v0, vf, a0, af = 0.0, 0.0, 0.0, 0.0

        T = np.array([
            [1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
            [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
            [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
            [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
            [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
            [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]
        ], dtype=float)

        b = np.array([p0, v0, a0, pf, vf, af], dtype=float)
        self._coeffs = np.linalg.solve(T, b)  

    def _poly(self, t: np.ndarray) -> np.ndarray:
        """Return position for array-like t."""
        a0, a1, a2, a3, a4, a5 = self._coeffs
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

    def _dpoly(self, t: np.ndarray, order: int) -> np.ndarray:
        """Return the x order derivative (1=vel, 2=acc, 3=jerk)."""
        a0, a1, a2, a3, a4, a5 = self._coeffs
        if order == 1:
            return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
        elif order == 2:
            return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
        elif order == 3:
            return 6 * a3 + 24 * a4 * t + 60 * a5 * t ** 2
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
        """Return uniformly-spaced samples (t, pos, vel, acc).

        Parameters
        ----------
        dt : float, optional
        """
        t = np.arange(self.t0, self.tf + 1e-9, dt)
        return t, self.position(t), self.velocity(t), self.acceleration(t)

    def coeffs(self):
        """Return polynomial coefficients a0…a5."""
        return self._coeffs.copy()