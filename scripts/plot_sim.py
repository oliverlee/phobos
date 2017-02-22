#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from load_sim import load_messages, get_records_from_messages


CH_CFG_ST_FREQUENCY = 10000

def get_time_vector(records):
    return records.timestamp/10000


if __name__ == '__main__':
    messages = load_messages(sys.argv[1])
    # ignore first sample as it transmitted before the simulation loop
    records = get_records_from_messages(messages)[1:]
    t = get_time_vector(records)

    # plot in degrees
    states = records.state[:, 1:] * 180/np.pi
    state_labels = ['roll angle', 'steer angle', 'roll rate', 'steer rate']
    sensor_labels = ['kistler measured torque', 'kollmorgen actual torque',
                     'steer encoder count', 'rear wheel encoder count']

    state_color = np.roll(sns.color_palette('Paired', 10), 2, axis=0)

    fig, ax = plt.subplots()
    for i, label in enumerate(state_labels):
        ax.plot(t, states[:, i], label=label, color=state_color[1 + 2*i])
    ax.legend()

    encoder_count = records.sensors.steer_encoder_count
    encoder_angle = encoder_count / 152000 * 360
    encoder_angle[np.where(encoder_angle > 180)[0]] -= 360

    # calculate a simple numerical derivative of encoder_angle
    # timestamps units are system ticks, at 10 kHz
    dt = np.insert(np.diff(t), 0, 0)
    encoder_rate = np.insert(np.diff(encoder_angle), 0, 0) / dt

    fig, ax = plt.subplots()

    index = state_labels.index('steer angle')
    ax.plot(t, states[:, index], label=state_labels[index],
            color=state_color[1 + 2*index])
    ax.plot(t, encoder_angle, label='encoder angle', color=state_color[2*index])

    index = state_labels.index('steer rate')
    ax.plot(t, states[:, index], label=state_labels[index],
            color=state_color[1 + 2*index])
    ax.plot(t, encoder_rate, label='encoder rate', color=state_color[2*index])

    ax.legend()

    fig, ax = plt.subplots()
    ax.plot(t, records.actuators.kollmorgen_command_torque,
            label='kollmorgen command torque', color=state_color[9])
    ax.plot(t, records.sensors.kollmorgen_actual_torque,
            label=sensor_labels[1], color=state_color[8])
    ax.plot(t, records.sensors.kistler_measured_torque,
            label=sensor_labels[0], color=state_color[5])
    ax.legend()
    plt.show()

