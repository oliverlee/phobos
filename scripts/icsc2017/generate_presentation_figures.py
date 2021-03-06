#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import seaborn as sns
sns.set_style('whitegrid', {'legend.frameon':True})
sns.set_context('talk')

file_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(file_dir, os.pardir)))
from load_sim import load_messages, get_records_from_messages, get_time_vector
from simulate_whipple_benchmark import simulate, calculate_weave_frequency


if __name__ == '__main__':
    filename = sys.argv[1]
    messages = load_messages(filename)

    # ignore first sample as it is transmitted before the simulation loop
    records = get_records_from_messages(messages[1:])
    t = get_time_vector(records)
    t -= t[0]

    state = records.state[:, 1:] # skip yaw angle

    colors = np.roll(sns.color_palette('Paired', 10), 2, axis=0)
    labels = [
            r'$\phi$, roll angle',
            r'$\delta$, steer angle',
            r'$\dot{\phi}$, roll rate',
            r'$\dot{\delta}$, steer rate',
    ]
    ylabels = [
        r'[$\degree$]',
        r'[$\degree$]',
        r'[$\degree$/s]',
        r'[$\degree$/s]',
    ]

    figsize = (14, 7.5)
    dpi = 1000
    linewidth = 2
    labelpad = 20
    stride = 10
    legend_loc = 'lower right'
    legend_alpha = 0.5
    legend_fc = 'white'
    legend_ec = 'white'

    try:
        endtime = float(sys.argv[2]) # new end time
    except IndexError:
        endtime = -1

    if endtime <= 0:
        endtime_index = -1
    else:
        endtime_index = np.squeeze(np.argwhere(t >= endtime))[0]
    t = t[:endtime_index]
    state = state[:endtime_index]

    # full timeseries plot
    fig1, ax1 = plt.subplots(4, 1, sharex=True, figsize=figsize)
    for i, ax in enumerate(ax1):
        ax.plot(t[::stride], 180/np.pi * state[::stride, i],
                label=labels[i],
                color=colors[2*i + 1], linewidth=linewidth)
        ax.set_ylabel(ylabels[i], rotation='horizontal', labelpad=labelpad)
        ax.legend(loc=legend_loc, framealpha=legend_alpha,
                  facecolor=legend_fc, edgecolor=legend_ec)
        ax.yaxis.set_major_locator(ticker.MaxNLocator(5))
    ax1[-1].set_xlabel('time [s]')
    ax1[0].set_xlim(0, max(t))

    if len(sys.argv) <= 3:
        plt.show()
        sys.exit()

    start = float(sys.argv[3]) # reduced time start
    stop = float(sys.argv[4]) # reduced time end

    start_index = np.squeeze(np.argwhere(t >= start))[0]
    stop_index = np.squeeze(np.argwhere(t >= stop))[0]

    # reduced timeseries plot
    fig2, ax2 = plt.subplots(4, 1, sharex=True, figsize=figsize)
    for i, ax in enumerate(ax2):
        ax.plot(t[start_index:stop_index],
                180/np.pi * state[start_index:stop_index, i],
                label=labels[i],
                color=colors[2*i + 1], linewidth=linewidth)
        ax.set_ylabel(ylabels[i], rotation='horizontal', labelpad=labelpad)
        ax.legend(loc=legend_loc, framealpha=legend_alpha,
                  facecolor=legend_fc, edgecolor=legend_ec)
        ax.yaxis.set_major_locator(ticker.MaxNLocator(5))
    ax2[-1].set_xlabel('time [s]')
    ax2[0].set_xlim(start, stop)

    # reduced timeseries + simulation plot
    v = messages[0].model.v
    if messages[0].model.HasField('dt'):
        dt = messages[0].model.dt
    else:
        dt = 0.001 # phoviz not writing first message
    n = int((stop - start)/dt)
    x0 = np.array(state[start_index, :]).reshape((4, 1))
    x = simulate(v, x0, dt, n)
    t_y = np.array(range(n + 1))*dt + start
    y = np.hstack((x0, x)).T
    fig3, ax3 = plt.subplots(4, 1, sharex=True, figsize=figsize)
    for i, ax in enumerate(ax3):
        ax.plot(t[start_index:stop_index],
                180/np.pi * state[start_index:stop_index, i],
                label='{}, simulator'.format(labels[i]),
                color=colors[2*i + 1], linewidth=linewidth)
        ax.plot(t_y, 180/np.pi * y[:, i],
                label='{}, model'.format(labels[i]),
                color=colors[2*i], linewidth=linewidth)
        ax.set_ylabel(ylabels[i], rotation='horizontal', labelpad=labelpad)
        ax.legend(loc=legend_loc, framealpha=legend_alpha,
                  facecolor=legend_fc, edgecolor=legend_ec)
        ax.yaxis.set_major_locator(ticker.MaxNLocator(5))
    ax3[-1].set_xlabel('time [s]')
    ax3[0].set_xlim(start, stop)

    plt.show()

    output_type = 'pdf'
    for i, fig in enumerate([fig1, fig2, fig3], 1):
        fig.tight_layout()
        j = filename.find('.pb.cobs')
        fig.savefig('{}_plot{}.{}'.format(filename[:j], i, output_type),
                    format=output_type, dpi=dpi)
    plt.show()
