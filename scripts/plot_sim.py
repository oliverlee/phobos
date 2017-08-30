#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy as np
import scipy
import matplotlib.pyplot as plt
import seaborn as sns

from load_sim import load_messages, get_records_from_messages, get_time_vector
from phobos.constants import sa


STATE_LABELS = ['roll angle', 'steer angle', 'roll rate', 'steer rate']
SENSOR_LABELS = ['kistler measured torque', 'kollmorgen actual torque',
                 'steer encoder count', 'rear wheel encoder count']

STATE_COLOR = np.roll(sns.color_palette('Paired', 10), 2, axis=0)

### adc/dac values are 12-bit
##def bits_to_si(data, max_si_unit, zero_offset=None):
##    half_full_range = 2**11
##    if zero_offset is None:
##        zero_offset = half_full_range
##    return (data.astype(float) - zero_offset)/half_full_range * max_si_unit


def convert_adcsample(adc_samples, adc_zero_value, magnitude):
    shifted_value = adc_samples.astype(float) - adc_zero_value
    return shifted_value*magnitude/sa.ADC_HALF_RANGE


def get_kollmorgen_applied_torque(adc_samples):
    return convert_adcsample(adc_samples,
                             sa.KOLLMORGEN_ADC_ZERO_OFFSET,
                             sa.MAX_KOLLMORGEN_TORQUE)


def get_kollmorgen_command_torque(adc_samples):
    return convert_adcsample(adc_samples,
                             sa.KOLLMORGEN_DAC_ZERO_OFFSET,
                             sa.MAX_KOLLMORGEN_TORQUE)


def get_kistler_sensor_torque(adc_samples):
    torque = np.zeros(adc_samples.shape)
    # inequalities are reversed as we need to negate adc_samples
    # max torque value constants are negative
    ineg = np.where(adc_samples > sa.KISTLER_ADC_ZERO_OFFSET_NEGATIVE)
    ipos = np.where(adc_samples < sa.KISTLER_ADC_ZERO_OFFSET_POSITIVE)
    torque[ineg] = convert_adcsample(
            adc_samples[ineg],
            sa.KISTLER_ADC_ZERO_OFFSET_NEGATIVE,
            sa.MAX_KISTLER_TORQUE_NEGATIVE)
    torque[ipos] = convert_adcsample(
            adc_samples[ipos],
            sa.KISTLER_ADC_ZERO_OFFSET_POSITIVE,
            sa.MAX_KISTLER_TORQUE_POSITIVE)
    return torque


def handlebar_inertia_torque(records, A):
    A_delta_dd = A[4, :]
    state = records.state
    steer_accel = np.dot(A_delta_dd, state.T)
    inertia_torque = sa.UPPER_ASSEMBLY_INERTIA_PHYSICAL * steer_accel
    return inertia_torque, steer_accel


def plot_states(t, states, second_yaxis=False, to_degrees=True):
    fig, ax1 = plt.subplots()
    lines = []
    if second_yaxis:
        ax2 = ax1.twinx()
    for i, label in enumerate(STATE_LABELS):
        linewidth = 0.9 * 2.5
        color = STATE_COLOR[1 + 2*i]

        if second_yaxis and label.endswith('rate'):
            ax = ax2
            linewidth = 0.6 * 2.5
        else:
            ax = ax1
        if second_yaxis:
            color_order = [3, 7, 2, 6]
            color = STATE_COLOR[color_order[i]]
            ax.yaxis.grid(False)

        if to_degrees:
            a = 180/np.pi
        else:
            a = 1
        l = ax.plot(t, a*states[:, i], label=label, color=color,
                    linewidth=linewidth)
        lines.extend(l)

    if second_yaxis:
        if to_degrees:
            ax1.set_ylabel('deg')
            ax2.set_ylabel('deg/s')
        else:
            ax1.set_ylabel('rad')
            ax2.set_ylabel('rad/s')
        ax1.set_xlabel('time [s]')
        ax1.legend(lines, [l.get_label() for l in lines])
        return fig, (ax1, ax2)

    if to_degrees:
        ax1.set_ylabel('deg, deg/s')
    else:
        ax1.set_ylabel('rad, rad/s')
    ax1.set_xlabel('time [s]')
    ax1.legend()
    return fig, ax1


def plot_entries(t, entries, n, m):
    fig, axes = plt.subplots(n, m, sharex=True)
    axesr = np.ravel(axes, order='F') # column order
    color = sns.husl_palette(len(axesr))
    for i, ax in enumerate(axesr):
        ax.plot(t, entries[:, i], color=color[i])
        ax.set_xlabel('time [s]')
    return fig, ax


def plot_kalman_gain_entries(t, kalman):
    ec = kalman[0].error_covariance
    y = len(ec) # number of entries
    # since error covariance is symmetric, size is (n, n) where
    # n(n + 1)/2 = y => n**2 + n - 2y = 0
    # take the first real, non-negative value
    n = 0
    for r in np.roots([1, 1, -2*y]):
        try:
            if r > 0:
                n = int(r) # state size
                break
        except TypeError:
            # r is complex
            pass
    msg = "unable to determine size of state vector from Kalman object"
    assert n != 0, msg
    m = len(kalman[0].kalman_gain)//n # input size
    fig, ax = plot_entries(t, kalman.kalman_gain, n, m)
    fig.suptitle('Kalman gain entries')
    return fig, ax


if __name__ == '__main__':
    messages = load_messages(sys.argv[1])
    # ignore first sample as it transmitted before the simulation loop
    records = get_records_from_messages(messages)[1:]
    t = get_time_vector(records)

    # current logs all use a constant speed so model is only written in the
    # first message
    m = messages[0]
    Ad_data = m.model.A.m
    Ad_size = int(np.sqrt(len(Ad_data)))
    Ad = np.array(Ad_data).reshape(2*(Ad_size,), order='F')
    A = scipy.linalg.logm(Ad)/m.model.dt
    inertia_torque, steer_accel = handlebar_inertia_torque(records, A)

    states = records.state[:, 1:]
    fig1, ax1 = plot_states(t, states, second_yaxis=False, to_degrees=False)

    encoder_count = records.sensors.steer_encoder_count
    encoder_angle = encoder_count / 152000 * 2*np.pi
    encoder_angle[np.where(encoder_angle > np.pi)[0]] -= 2*np.pi

    # calculate a simple numerical derivative of encoder_angle
    # timestamps units are system ticks, at 10 kHz
    dt = np.insert(np.diff(t), 0, 0)
    encoder_rate = np.insert(np.diff(encoder_angle), 0, 0) / dt

    fig2, ax2 = plt.subplots()
    index = STATE_LABELS.index('steer angle')
    ax2.plot(t, states[:, index], label=STATE_LABELS[index],
             color=STATE_COLOR[1 + 2*index])
    ax2.plot(t, encoder_angle, label='encoder angle',
             color=STATE_COLOR[2*index])

    index = STATE_LABELS.index('steer rate')
    ax2.plot(t, states[:, index], label=STATE_LABELS[index],
             color=STATE_COLOR[1 + 2*index])
    ax2.plot(t, encoder_rate, label='encoder rate (from angle diff)',
             color=STATE_COLOR[2*index])
    ax2.plot(t, steer_accel, label='estimated steer accel',
             color=STATE_COLOR[1])
    ax2.set_ylabel('rad, rad/s, rad/s/s')
    ax2.set_xlabel('time [s]')
    ax2.legend()

    fig3, ax3 = plt.subplots()
    ax3.plot(t,
             get_kollmorgen_applied_torque(
                 records.sensors.kollmorgen_actual_torque),
             label=SENSOR_LABELS[1],
             color=STATE_COLOR[9])
    ax3.plot(t,
             get_kistler_sensor_torque(records.sensors.kistler_measured_torque),
             label=SENSOR_LABELS[0],
             color=STATE_COLOR[5])
    ax3.plot(t,
             records.input[:, 1],
             label='steer torque',
             color=STATE_COLOR[3])
    ax3.plot(t,
             inertia_torque,
             label='handlebar inertia torque',
             color=STATE_COLOR[1])
    # FIXME The field sensors.kollmorgen_command_velocity contains the
    # kollmorgen drive reference. This is either interpreted as a torque or
    # velocity command depending on the drive settings and must be set correctly
    # depending on the project.
    # We currently assume torque mode is being used.
    ax3.plot(t,
             get_kollmorgen_command_torque(
                 records.actuators.kollmorgen_command_velocity),
             label='kollmorgen command torque',
             color=STATE_COLOR[7])
    ax3.set_ylabel('torque [N-m]')
    ax3.set_xlabel('time [s]')
    ax3.legend()

    fig4, ax4 = plot_kalman_gain_entries(t, records.kalman)

    plt.show()

