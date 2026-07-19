"""Helpers for saving and reconstructing simulation logs."""

import numpy as np
from pydrake.all import PiecewisePolynomial


def save_log(log, dir_prefix):
    """Save a Drake log's sample times and data to ``.npy`` files."""
    t = log.sample_times()
    x = log.data()
    np.save(dir_prefix + "_t.npy", t)
    np.save(dir_prefix + "_data.npy", x)


def load_log(fdir):
    """Load an ``.npy`` array from disk."""
    return np.load(fdir)


def reconstruct_log_to_trajectory(log):
    """Rebuild position/velocity trajectories from a live Drake log."""
    t = log.sample_times()
    x = log.data()
    return reconstruct_logdata_to_trajectory(t, x, n_downsample=10)


def reconstruct_logdata_to_trajectory(t, x, n_downsample=100):
    """Rebuild position/velocity trajectories from saved log arrays."""
    q0 = x[:83, 0]
    q1 = x[:83, 1]

    N = np.shape(t)[0]
    q_all_traj = PiecewisePolynomial.FirstOrderHold(
        [t[0], t[1]], np.vstack([q0, q1]).T
    )

    for idx in range(n_downsample, N):
        if idx % n_downsample == 0:
            q = x[:59 + 12 + 12, idx]
            t_now = t[idx]
            q_all_traj.AppendFirstOrderSegment(t_now, q)

    dq_all_traj = q_all_traj.MakeDerivative()
    return q_all_traj, dq_all_traj
