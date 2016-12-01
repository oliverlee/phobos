#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pylab as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
import seaborn as sns

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

def plot_pose_vispy(data):
    t = get_time_vector(data)
    rows = 5
    cols = 2
    fields = data.dtype.names
    colors = np.array(sns.color_palette('husl', len(fields)))

    # plot all lines using my hacky fig and plot widget
    from phobos.vispy.fig import Fig
    fig = Fig(show=False, sharex=True)

    print('plotting the following fields:')
    print_name = lambda r, c, name: print('[{}, {}] {}'.format(r, c, name))

    # plot trajectory
    p = fig[0, 0]
    chunks = 100
    trajectory_colors = np.array(sns.color_palette('husl', chunks))
    for x, y, color in zip(np.array_split(data['x'], chunks),
                           np.array_split(data['y'], chunks),
                           trajectory_colors):
        p.plot((x, y), color=color, linked=False)
    print_name(0, 0, 'trajectory')

    for n, (name, color) in enumerate(zip(fields, colors), 1):
        if n == 1:
            name = 'y'
            color = colors[1]
        elif n == 2:
            name = 'x'
            color = colors[0]
        row = n // cols
        col = n % cols
        p = fig[row, col]
        if name == 'timestamp':
            # plot histogram of sample time instead
            dt = (data[name] - np.roll(data[name], 1))[1:]
            p.histogram(dt, t=t[:-1], color=color)
            name = 'timestamp histogram'
        else:
            p.plot((t, data[name]), color=color)
        p.xaxis.visible = True
        print_name(row, col, name)

    if sys.flags.interactive == 0:
        fig.show(run=True)
    return fig


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

    filename = os.path.realpath(sys.argv[1])
    gitsha1, pose_data, num_errors = load.pose_logfile(filename)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(pose_data), num_errors))

    if len(sys.argv) > 2 and sys.argv[2]:
        plot_vispy(pose_data)
    else:
        plot_pose_vispy(pose_data, filename)
        plt.show()

    sys.exit(0)
