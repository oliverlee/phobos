#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This script estimates the inertia of the handlebars and upper part of the
steering column (everything above the torque sensor) about the steer axis.
Although we expect this value to be small, it still influences the dynamics of
the system and influences the values we read from the torque sensor.

For more information (and plots) refer to:
https://github.com/oliverlee/phobos/issues/90
"""
import sys
from scipy.signal import savgol_filter as sg
from load_sim import load_messages, get_records_from_messages, get_time_vector

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


KOLLMORGEN_MAX_TORQUE = 10.78 # N-m
ENCODER_COUNT_PER_REV = 152000


def estimate_inertia(records):
    """We assume no stiffness or damping for the equation of motion is:
    T = I*alpha
    where T is the sum of torques applied
          I is the moment of inertia
          alpha is the angular acceleration
    """
    t = get_time_vector(records)
    torque = (((records.sensors.kollmorgen_actual_torque).astype(float) -
               2**11)/2**11 * KOLLMORGEN_MAX_TORQUE) # N-m
    angle = np.unwrap(((records.sensors.steer_encoder_count).astype(float) *
                       2*np.pi / ENCODER_COUNT_PER_REV)) # radians
    dt = np.diff(t).mean()
    angle_d = sg(angle, 11, 3, deriv=1, delta=dt, mode='nearest')
    angle_dd = sg(angle, 11, 3, deriv=2, delta=dt, mode='nearest')

    color = sns.color_palette('Paired', 10)
    fig, ax = plt.subplots()
    ax.plot(t, angle, label='angle', color=color[1])
    ax.plot(t, angle_d, label='angular rate', color=color[3])
    ax.plot(t, angle_dd, label='angular accel', color=color[5])
    ax.plot(t, torque, label='torque', color=color[7])
    ax.legend()
    plt.show()

    ret = np.linalg.lstsq(np.reshape(angle_dd, (-1, 1)),
                          np.reshape(torque, (-1, 1)))
    return np.squeeze(ret[0])


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: {} <log_file>\n'.format(__file__))
        print('    <log_file>\tFile containing serialized and framed ' +
              'protobuf messages')
        sys.exit(1)

    messages = load_messages(sys.argv[1])
    # ignore first sample as it transmitted before the simulation loop
    records = get_records_from_messages(messages)[1:400]
    inertia = estimate_inertia(records)
    print('Calculated handlebar inertia is: {} kg-m^2'.format(inertia))

    sys.exit(0)
