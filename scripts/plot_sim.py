#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from load_sim import load_messages, get_records_from_messages


if __name__ == '__main__':
    messages = load_messages(sys.argv[1])
    records = get_records_from_messages(messages)

    # plot in degrees
    states = records.state[:, 1:] * 180/np.pi
    state_labels = ['roll_angle', 'steer_angle', 'roll_rate', 'steer_rate']
    sensor_labels = ['kistler measured torque', 'kollmorgen_actual_torque',
                     'steer_encoder_count', 'rear_wheel_encoder_count']

    colors = sns.color_palette('Paired', 10)

    fig, ax = plt.subplots()
    for i, label in enumerate(state_labels):
        ax.plot(states[:, i], label=label, color=colors[1 + 2*i])
    ax.legend()

    encoder_count = records.sensors.steer_encoder_count
    encoder_angle = encoder_count / 152000 * 360
    encoder_angle[np.where(encoder_angle > 180)[0]] -= 360

    fig, ax = plt.subplots()
    ax.plot(states[:, 1], label=state_labels[1], color=colors[3])
    ax.plot(encoder_angle, label='encoder angle', color=colors[2])
    ax.legend()

    plt.show()

