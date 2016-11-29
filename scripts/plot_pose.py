#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import struct
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pylab as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
import seaborn as sns

from phobos import pose
from phobos import cobs
from phobos import load


def get_time_vector(data):
    ts = data['timestamp']
    ts_shift = np.roll(ts, -1).astype('int')
    overflow = np.roll(np.array(ts - ts_shift > 0).astype('int'), 1)
    overflow[0] = 0
    nan_index = np.where(np.isnan(data['x']))[0]
    try:
        overflow[nan_index + 1] = 0
    except IndexError:
        overflow[nan_index[:-1]] = 0
    ts_offset = np.cumsum(overflow) * 256
    ts_offset[nan_index] = -255 # nan values have time set to 0
    return ts + ts_offset


def plot_pose(data, filename=None):
    t = get_time_vector(data)

    rows = 5
    cols = 2
    fields = data.dtype.names
    fig, axes = plt.subplots(rows, cols, sharex=True)
    axes = axes.ravel()
    colors = sns.color_palette('husl', len(fields))

    for name, n in zip(data.dtype.names, range(len(axes))):
        if name == 'timestamp':
            x = data['x']
            y = data['y']
            husl20 = sns.color_palette('husl',20)
            trajectory = np.array([x, y]).T.reshape(-1, 1, 2)
            segments = np.concatenate([trajectory[:-1], trajectory[1:]],
                                      axis=1)
            lc = LineCollection(segments, cmap=ListedColormap(husl20))
            lc.set_array(t)
            ax = plt.subplot2grid((rows, cols), (0, 0), rowspan=2)
            ax.add_collection(lc)
            minima = np.min(trajectory, axis=0)[0]
            maxima = np.max(trajectory, axis=0)[0]
            ax.set_xlim(minima[0], maxima[0])
            ax.set_ylim(minima[1], maxima[1])
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.plot(trajectory[-1, 0, 0], trajectory[-1, 0, 1],
                    color=husl20[0],
                    label='rear contact point trajectory')
            ax.invert_yaxis()
            ax.xaxis.tick_top()
            ax.xaxis.set_label_position('top')
            ax.set_aspect('equal', 'datalim')
            ax.legend()
            axes[2] = ax
            axes[0].axis('off')
        else:
            if name == 'x':
                ax = axes[1]
            elif name == 'y':
                ax = axes[3]
                ax.invert_yaxis()
            else:
                ax = axes[n + 2]
            x = data[name]
            if name in ('pitch', 'yaw', 'roll', 'steer', 'rear_wheel'):
                x = x * 180 / np.pi
                labelname = name + ' [°]'
            elif name in ('x', 'y'):
                labelname = name + ' [m]'
            elif name in ('v'):
                labelname = name + ' [m/s]'
            ax.plot(t, x, label=labelname, color=colors[n])
            ax.legend()

    axes[-1].set_xlabel('time [ms]')
    axes[-2].set_xlabel('time [ms]')

    title = 'bicycle pose'
    if filename is not None:
        title += ' (file \'{}\')'.format(filename)
    fig.suptitle(title, size=mpl.rcParams['font.size'] + 2)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: {} <pose_log_file>\n\nPlot pose data'.format(__file__))
        print('    <pose_log_file>\tFile containing samples in ' +
              'the given packed struct binary format:\n')
        print(desc)
        sys.exit(1)

    # set helvetica as plot font
    mpl.rcParams['font.family'] = 'Helvetica'
    mpl.rcParams['font.weight'] = 'light'
    mpl.rcParams['axes.labelweight'] = 'light'

    #samples = convert.load_sample_log(sys.argv[1])
    filename = os.path.realpath(sys.argv[1])
    gitsha1, pose_data, num_errors = load.pose_logfile(filename)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(pose_data), num_errors))

    plot_pose(pose_data, filename)
    plt.show(block=False)

    sys.exit(0)
