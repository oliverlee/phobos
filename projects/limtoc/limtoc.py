#!/usr/bin/env python
# -*- coding: utf-8 -*-
import gzip

import numpy as np
import scipy
import scipy.signal

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns


def load_log(filename):
    """Load data from logs generated from limtoc. Returns
    a numpy record array.
    """
    datatype = None
    values = []

    if filename.endswith('.gz'):
        openf = lambda x: gzip.open(x, 'rt')
    else:
        openf = open

    with openf(filename) as f:
        for line in f:
            if not line.strip():
                continue
            if datatype is None:
                if line.strip().endswith('fields are:'):
                    # ignore lines until we get here
                    # next line describes data
                    datatype = []
                elif 'k = ' in line:
                    print(line.strip())
                continue
            if not datatype:
                field_names = line.split(',')
                datatype = np.dtype(
                    [('time', '<f8')] + [(name.strip(), '<f8') for name in field_names])
            else:
                vals = [float(f) for f in line.split(',')]
                assert len(vals) == len(datatype) - 1
                vals.insert(0, vals[0]) # copy clock field value to time
                values.append(tuple(vals))

    data = np.rec.array(values, dtype=datatype)
    t = data.time
    dt = np.diff(t)
    overflow_counts = 2**32 * np.cumsum(dt < 0)
    data.time[1:] += overflow_counts
    data.time -= data.time[0]
    data.time /= 168e6 # real time clock frequency
    return data


def plot_log(record, show_plot=True):
    colors = sns.color_palette('Paired', 10)
    fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

    ax[0].plot(record.time, record.steer_angle, color=colors[1], label='steer angle')
    ax[0].plot(record.time, 0*record.time, color='black', linewidth=1, zorder=1)
    ax[0].legend()
    ax[0].set_xlabel('time [s]')
    ax[0].set_ylabel('[rad]')

    ax[1].plot(record.time, record.kistler_torque, color=colors[3],
               alpha=0.8, label='sensor torque')
    ax[1].plot(record.time, record.motor_torque, color=colors[5],
               alpha=0.8, label='motor torque')
    ax[1].plot(record.time, record.feedback_torque, color=colors[7],
               alpha=0.8, label='feedback torque command')
    ax[1].legend()
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('torque [N-m]')
    ax[1].plot(record.time, 0*record.time, color='black', linewidth=1, zorder=1)
    if show_plot:
        plt.show()
    return fig, ax


def display_log_plot(filename):
    vals = load_log(filename)
    fig, ax = plot_log(vals, show_plot=False)
    try:
        title = filename
        mpld3
    except NameError:
        fig.suptitle(title)
    else:
        ax[0].set_title(title)
    plt.show()


def plot_fft(record):
    N = len(record)
    dt = np.diff(record.time).mean()
    xf = scipy.fftpack.fftfreq(N, dt)
    yf = scipy.fftpack.fft(record.steer_angle)

    # plot only frequency components less than 10 Hz
    fftslice = slice(0, np.where(xf > 10)[0][0])

    fig, ax = plt.subplots()
    ax.plot(xf[fftslice], 2/N * np.abs(yf[fftslice]))
    plt.show()
    return fig, ax


# motor torque = -k * x
def calculate_stiffness(record):
    k_commanded = np.linalg.lstsq(np.reshape(record.steer_angle, (-1, 1)),
                                  -np.reshape(record.feedback_torque, (-1, 1)))[0][0][0]
    k_measured = np.linalg.lstsq(np.reshape(record.steer_angle, (-1, 1)),
                                 -np.reshape(record.motor_torque, (-1, 1)))[0][0][0]
    return k_commanded, k_measured


def calculate_fit(record):
    # use equation of the form:
    #   x = (a + bt)*exp(c*t)*sin(2*pi*e*t + d)
    fit = []

    n = 'exponential'
    f = lambda a, b, c, d, e: lambda t: a*np.exp(c*t)*np.sin(2*np.pi*e*t + d)
    p0 = (0, 0, -0.1, 0, 1.5)
    fit.append([n, f, p0])

    n = 'linear'
    f = lambda a, b, c, d, e: lambda t: (a + b*t)*np.sin(2*np.pi*e*t + d)
    p0 = (0, -2, 0, 0, 1.5)
    fit.append([n, f, p0])

    g = lambda record, f: lambda p: f(*p)(record.time) - record.steer_angle

    for i in range(len(fit)):
        n, f, p0 = fit[i]
        p = scipy.optimize.leastsq(g(record, f), p0)[0]
        fit[i][2] = p
    return fit


def plot_fit(record, fit_params, show_plot=True):
    colors = sns.color_palette('Paired', 10)
    fig, ax = plt.subplots(len(fit_params), 1, figsize=(12, 8), sharex=True)

    for i in range(len(fit_params)):
        n, f, p = fit_params[i]
        ax[i].plot(record.time, record.steer_angle, color=colors[1],
                   label='steer angle')
        ax[i].plot(record.time, f(*p)(record.time), color=colors[5],
                   linestyle='--',
                   label='{} decay fit, {}'.format(n, p))
        ax[i].plot(record.time, 0*record.time, color='black', linewidth=1, zorder=1)
        ax[i].legend()
        ax[i].set_ylabel('[rad]')
    ax[-1].set_xlabel('time [s]')
    plt.show()
    return fig, ax
