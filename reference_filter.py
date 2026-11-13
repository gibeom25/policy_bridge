"""Jerk-limited reference filter for tracking a live setpoint.

Ported from the control math in ``~/teleop-franka/gello_software/gello/robots/
franka_fr3.py`` (``_control_loop``, critically-damped second-order filter).
That file's docstring explains why this exists: pylibfranka's active-control
API (``start_joint_position_control`` -> ``readOnce``/``writeOnce``) sends
whatever command you give it straight to the robot with zero shaping -- unlike
libfranka's blocking ``Robot::control()`` path, it does not run
``limitRate()``/``lowpassFilter()`` itself. Without this filter, a noisy or
discontinuous target (a human hand on gello_software's leader, or here a
policy's raw action) produces a jerk spike that trips
``joint_motion_generator_acceleration_discontinuity`` at best, and drives
sustained vibration into the joints at worst.

This is deliberately just the math, with no pylibfranka dependency, so it can
be unit-tested without hardware (see ``tests/test_reference_filter.py``).
"""

from __future__ import annotations

import numpy as np


class JerkLimitedReferenceFilter:
    """Critically-damped 2nd-order filter, saturated in jerk/accel/velocity.

    Call :meth:`reset` once with the robot's actual starting position, then
    call :meth:`step` once per control tick with the current desired target.
    The returned command changes smoothly even if ``target`` jumps or is
    noisy -- the filter is the safety boundary, not the caller.

    Args:
        n_dims: dimensionality (7 for joints, 6 for a Cartesian twist, ...).
        dt: control period in seconds (1e-3 for the FCI's 1 kHz loop).
        v_max, a_max, j_max: velocity / acceleration / jerk saturation
            limits. Scalar (applied to all dims) or per-dim array. Defaults
            match the values validated live on this FR3
            (max_joint_velocity=1.0 rad/s, max_joint_acceleration=4.0 rad/s^2,
            max_joint_jerk=3000, comfortably under libfranka's
            ``kMaxJointJerk`` of 5000) -- don't raise these without
            re-validating on hardware first.
        wn: natural frequency of the reference filter (rad/s). Higher tracks
            the target more aggressively; lower is smoother. kp = wn^2,
            kd = 2*wn (critical damping -> no overshoot by construction).
    """

    def __init__(
        self,
        n_dims: int,
        dt: float = 1e-3,
        v_max: float | np.ndarray = 1.0,
        a_max: float | np.ndarray = 4.0,
        j_max: float | np.ndarray = 3000.0,
        wn: float = 10.0,
    ):
        self.n_dims = n_dims
        self.dt = float(dt)
        self.v_max = np.broadcast_to(np.asarray(v_max, dtype=float), (n_dims,)).copy()
        self.a_max = np.broadcast_to(np.asarray(a_max, dtype=float), (n_dims,)).copy()
        self.j_max = np.broadcast_to(np.asarray(j_max, dtype=float), (n_dims,)).copy()
        self.kp = float(wn) ** 2
        self.kd = 2.0 * float(wn)

        self.q_cmd = np.zeros(n_dims)
        self.qd_cmd = np.zeros(n_dims)
        self._acc_prev = np.zeros(n_dims)

    def reset(self, q0: np.ndarray) -> None:
        """Latch the filter to an actual starting position (no ramp-in jump)."""
        self.q_cmd = np.asarray(q0, dtype=float).copy()
        self.qd_cmd = np.zeros(self.n_dims)
        self._acc_prev = np.zeros(self.n_dims)

    def step(self, target: np.ndarray, dt: float | None = None) -> np.ndarray:
        """Advance the filter toward ``target`` by ``dt`` seconds (default: the nominal period).

        Pass the REAL elapsed control period (libfranka's ``duration``). When a 1 kHz cycle is missed the robot
        expects the command to have advanced by 2 ms; advancing only 1 ms halves the commanded velocity for one
        tick, which the robot sees as a huge acceleration step -> ``joint_motion_generator_acceleration_discontinuity``.
        """
        dt = self.dt if dt is None else float(dt)
        target = np.asarray(target, dtype=float)
        err = target - self.q_cmd
        # Velocity limit expressed as an ACCELERATION bound that shrinks to 0 as |qd| -> v_max, at a rate the jerk
        # limit can follow (a = sqrt(2 * j_max * remaining_dv)). Before this, the hard velocity clamp below cut the
        # acceleration from a_max to 0 in ONE tick whenever a fast policy step saturated v_max -> jerk ~2200 rad/s^3
        # (> j_max) -> joint_motion_generator_acceleration_discontinuity.
        # Discrete form: ramping a down to 0 in steps of j_max*dt adds a^2/(2 j) + a*dt/2 of velocity.
        h = 0.5 * self.j_max * dt
        a_hi = np.minimum(self.a_max, -h + np.sqrt(h * h + 2.0 * self.j_max * np.maximum(self.v_max - self.qd_cmd, 0.0)))
        a_lo = -np.minimum(self.a_max, -h + np.sqrt(h * h + 2.0 * self.j_max * np.maximum(self.v_max + self.qd_cmd, 0.0)))
        acc_target = np.clip(self.kp * err - self.kd * self.qd_cmd, a_lo, a_hi)
        dacc_max = self.j_max * dt
        acc = np.clip(acc_target, self._acc_prev - dacc_max, self._acc_prev + dacc_max)

        # Hard clamp is only a last-resort guard (5 % above v_max): the soft bound above keeps |qd| at v_max with a
        # sub-percent overshoot, so this clamp (which bypasses the jerk limit) never engages in normal operation.
        v_hard = 1.05 * self.v_max
        qd_new = np.clip(self.qd_cmd + acc * dt, -v_hard, v_hard)
        # Feed back the acceleration that actually survived the velocity
        # clamp (not the pre-clamp value) so acc_prev doesn't drift away from
        # reality and spike the moment the clamp releases.
        self._acc_prev = (qd_new - self.qd_cmd) / dt
        self.qd_cmd = qd_new
        self.q_cmd = self.q_cmd + self.qd_cmd * dt
        return self.q_cmd.copy()
