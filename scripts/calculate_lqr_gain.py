#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import scipy
import control
from dtk.bicycle import benchmark_state_space_vs_speed, benchmark_matrices


def compute_whipple_lqr_gain(velocity):
    _, A, B = benchmark_state_space_vs_speed(*benchmark_matrices(), velocity)
    Q = np.diag([1e5, 1e3, 1e3, 1e2])
    R = np.eye(2)


    gains = [control.lqr(Ai, Bi, Q, R)[0] for Ai, Bi in zip(A, B)]
    return gains


if __name__ == '__main__':
    import sys

    v_low = 0 # m/s
    if len(sys.argv) > 1:
        v_high = int(sys.argv[1])
    else:
        v_high = 1 # m/s

    velocities = [v_low, v_high]
    gains = compute_whipple_lqr_gain(velocities)

    for v, K in zip(velocities, gains):
        print('computed LQR controller feedback gain for v = {}'.format(v)j)
        print(K)
        print()
