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
    """Load data from logs generated from reldresal. Returns
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
                    [('time', '<f8')] + [(name.strip(), '<f8')
                     for name in field_names])
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
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

    steer_angle = record.steer_angle
    rescaled_voltage = record.steer_angle_voltage*20/2**12 - 10 # 12-bit ADC to ±12V
    KOLLM_DEG_PER_VOLT = 4.5
    steer_angle_kollm = rescaled_voltage*KOLLM_DEG_PER_VOLT*np.pi/180
    offset = steer_angle_kollm[0] - steer_angle[0]
    steer_angle_kollm_oc = steer_angle_kollm - offset

    A = np.vstack([steer_angle_kollm, np.ones(len(steer_angle))]).T
    m, c = np.linalg.lstsq(A, steer_angle)[0]
    steer_angle_kollm_lq = m*steer_angle_kollm + c

    ax[0].plot(record.time, steer_angle,
               color=colors[1], label='steer angle encoder')
    ax[0].plot(record.time, steer_angle_kollm,
               color=colors[0], label='steer angle drive (converted)')
    ax[0].plot(record.time, steer_angle_kollm_oc,
               color=colors[9],
               label='steer angle drive (converted, offset corrected')
    ax[0].plot(record.time, steer_angle_kollm_lq,
               color=colors[8],
               label='steer angle drive (converted, least squares fit)')
    ax[0].plot(record.time, 0*record.time,
               color='black', linewidth=1, zorder=1)
    ax[0].legend()
    ax[0].set_xlabel('time [s]')
    ax[0].set_ylabel('[rad]')

    ax[1].plot(record.time, record.kistler_torque, color=colors[3],
               alpha=0.8, label='sensor torque')
    ax[1].legend()
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('torque [N-m]')
    ax[1].plot(record.time, 0*record.time, color='black', linewidth=1, zorder=1)
    if show_plot:
        plt.show()
    return fig, ax


def plot_integrated_torque(record, m1, show_plot=True):
    colors = sns.color_palette('Paired', 10)
    fig, ax = plt.subplots(1, 1, figsize=(11, 6))

    steer_angle = record.steer_angle
    rescaled_voltage = record.steer_angle_voltage*20/2**12 - 10 # 12-bit ADC to ±12V
    KOLLM_DEG_PER_VOLT = 4.5
    steer_angle_kollm = rescaled_voltage*KOLLM_DEG_PER_VOLT*np.pi/180
    offset = steer_angle_kollm[0] - steer_angle[0]
    steer_angle_kollm_oc = steer_angle_kollm - offset

    a = -record.kistler_torque/m1
    v = np.zeros(record.time.shape)
    x = np.zeros(record.time.shape)
    dt = np.diff(record.time)
    for i in range(1, len(record.time)):
        v[i] = a[i]*dt[i - 1] + v[i - 1]
        x[i] = v[i]*dt[i - 1] + x[i - 1]
    steer_angle_kollm_int = x

    ax.plot(record.time, steer_angle,
            color=colors[1], label='steer angle encoder')
    ax.plot(record.time, steer_angle_kollm_oc,
            color=colors[9],
            label='steer angle drive (converted, offset corrected')
    ax.plot(record.time, steer_angle_kollm_int,
            color=colors[5],
            label='steer angle estimate (torque integrated)')
    ax.plot(record.time, 0*record.time,
            color='black', linewidth=1, zorder=1)
    ax.legend()
    ax.set_xlabel('time [s]')
    ax.set_ylabel('[rad]')

    if show_plot:
        plt.show()
    return fig, ax


def plot_differentiated_position(record, m1, show_plot=True):
    colors = sns.color_palette('Paired', 10)
    fig, ax = plt.subplots(1, 1, figsize=(11, 6))

    steer_angle = record.steer_angle

    steer_accel_sg = scipy.signal.savgol_filter(
            steer_angle, 111, 5,
            deriv=2,
            delta=np.mean(np.diff(record.time)[:-1]),
            mode='nearest')

    ax.plot(record.time, record.kistler_torque, color=colors[3],
            alpha=0.8, label='sensor torque')
    ax.plot(record.time, -steer_accel_sg*m1, color=colors[5],
            alpha=0.8, label='sensor torque (encoder differentiated)')
    ax.legend()
    ax.set_xlabel('time [s]')
    ax.set_ylabel('torque [N-m]')
    ax.plot(record.time, 0*record.time, color='black', linewidth=1, zorder=1)

    if show_plot:
        plt.show()
    return fig, ax
