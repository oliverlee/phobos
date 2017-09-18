#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import scipy
from dtk.bicycle import benchmark_state_space_vs_speed, benchmark_matrices


def calculate_weave_frequency(t, x):
    """fit roll rate to equation of the form:
    x = c1 + exp(d*t)*[c2*cos(w*t + c4) + c3*sin(w*t + c4)]
    """
    t = np.copy(t)
    t = np.reshape(t, (-1,))
    x = np.reshape(x, (-1,))
    t0 = t[0]
    t -= t0
    print('t range', np.min(t), np.max(t))
    print('x range', np.min(x), np.max(x))
    ## fit equation
    #f = lambda c1, c2, c3, c4, d, w: lambda t: c1 + np.exp(d*t)*[c2*np.cos(w*t + c4) + c3*np.sin(w*t + c4)]
    def f(c1, c2, c3, c4, d, w):
        def g(t):
            return np.squeeze(
                    c1 +
                    np.exp(d*t)*[c2*np.cos(w*t + c4) + c3*np.sin(w*t + c4)])
        return g


    # error function
    # params, p = (c1, c2, c3, c4, d, w)
    e = lambda p: f(*p)(t) - x

    # assumes this function oscillates about a mean
    dt = t[-1] - t[0]
    mean = np.mean(x)
    y = x - mean
    z = np.where(np.diff(np.sign(y)))[0]
    print('z', z)
    print('dt', dt)
    try:
        amp0 = np.max(np.abs(y[z[0]:z[1]])) # first hump
        amp1 = np.max(np.abs(y[z[-2]:z[-1]])) # last hump
    except IndexError:
        amp0 = np.max(np.abs(y[:10]))
        amp1 = np.max(np.abs(y[-10:]))
    decay = np.log(amp1/amp0)/dt
    freq = 2*np.pi*len(z)/dt/2
    delay = (z[1] - z[0] - z[0])/(z[1] - z[0]) * np.pi

    p0 = (mean, # c1
          amp0 - np.abs(mean), # c2
          amp0 - np.abs(mean), # c3
          delay, # c4
          decay, # d
          freq) # w
    print(p0)
    p = scipy.optimize.leastsq(e, p0)[0]

    g = lambda t: f(*p)(t + t0)
    return p, g


def simulate(v, x0, dt, n, u=None):
    _, A, B = benchmark_state_space_vs_speed(*benchmark_matrices(), [v])
    A = np.squeeze(A)
    B = np.squeeze(B)

    M = np.zeros((6, 6))
    M[:4, :4] = A
    M[:4, 4:] = B
    M *= dt

    Md = scipy.linalg.expm(M)
    Md_zero = Md[4:, :4]
    Md_eye = Md[4:, 4:]
    if not np.array_equal(Md_zero, np.zeros(Md_zero.shape)):
        print('WARNING: Failure in system discretization')
        print(Md_zero)
        print('should equal 0')
    if not np.array_equal(Md_eye, np.eye(2)):
        print('WARNING: Failure in system discretization')
        print(Md_eye)
        print('should equal I')
    Ad = Md[:4, :4]
    Bd = Md[:4, 4:]

    if u is None:
        u = np.zeros((2, n))
    x = np.zeros((4, n))
    for i in range(n):
        x[:, i:i+1] = np.dot(Ad, x0) + np.dot(Bd, u[:, i:i+1])
        x0 = x[:, i:i+1]
    return x


if __name__ == '__main__':
    import sys
    import matplotlib.pyplot as plt
    from plot_sim import plot_states

    sim_time = 3 # time for our Whipple simulation or upper limit [s]
    start_index = 50 # number of records to skip to allow some time for state
                     # estimate to converge

    if len(sys.argv) > 1:
        from load_sim import (load_messages, get_records_from_messages,
                              get_time_vector)
        messages = load_messages(sys.argv[1])
        records = get_records_from_messages(messages)
        t = get_time_vector(records[start_index:])
        t -= t[0] # redefine zero time for simulator data

        m = messages[0]
        v = m.model.v
        dt = m.model.dt
        if t[-1] < sim_time:
            sim_time = t[-1]
        n = int(sim_time/dt)

        states = records[start_index:].state[:, 1:]
        fig2, ax2 = plot_states(t[:n], states[:n, :], to_degrees=True)
        fig2.suptitle('Phobos simulator')
        x0 = np.array(messages[start_index].state.x[1:]).reshape((4, 1))
    else:
        v = 5
        dt = 0.005
        n = int(sim_time/dt)
        x0 = np.array(
            [ 0.0031247 ,  0.09299604, -0.03369007, -0.05003717]
            ).reshape((4, 1))

    x = simulate(v, x0, dt, n)
    t = np.array(range(n + 1)) * dt
    fig, ax = plot_states(t, np.hstack((x0, x)).T, to_degrees=True)
    fig.suptitle('Whipple simulation')

    plt.show()
