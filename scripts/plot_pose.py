#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
import scipy
import scipy.signal
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
    print_field = lambda r, c, name: print('[{}, {}] {}'.format(r, c, name))

    # plot trajectory
    p = fig[0, 0]
    chunks = 100
    trajectory_colors = np.array(sns.color_palette('husl', chunks))
    for x, y, color in zip(np.array_split(data['x'], chunks),
                           np.array_split(data['y'], chunks),
                           trajectory_colors):
        p.plot((x, y), color=color, linked=False)
    print_field(0, 0, 'trajectory')

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
        print_field(row, col, name)

    if sys.flags.interactive == 0:
        fig.show(run=True)
    return fig


class TrajectoryDisplay(object):
    def __init__(self, ax, t, x, y):
        self.ax = ax
        self.t = t
        self.x = x
        self.y = y

    def ax_update(self, ax):
        tmin, tmax = ax.get_xlim()
        imin = np.argmax(self.t >= tmin)
        imax = self.t.shape[0] - np.argmax(self.t[::-1] < tmax)
        x = self.x[imin:imax]
        y = self.y[imin:imax]
        self.ax.set_xlim(np.min(x), np.max(x))
        self.ax.set_ylim(np.min(y), np.max(y))

class TimeseriesDisplay(object):
    def __init__(self, lines, data, t, title_func):
        self.lines = lines
        self.data = data
        self.t = t
        self.title_func = title_func

    def decimate(self, time_range):
        # let's try to keep the data length at 1000 points
        decimate_target_length = 1000
        tmin, tmax = time_range
        indices = np.where((self.t >= tmin) & (self.t <= tmax))[0]
        decimation_factor = int(len(indices) / decimate_target_length)
        f = lambda x: x
        if decimation_factor > 0:
            f = lambda x: scipy.signal.decimate(x, decimation_factor,
                                                zero_phase=True)

        return f(self.t), (f(d) for d in self.data), decimation_factor

    def ax_update(self, ax):
        time_range = ax.get_xlim()
        t, data, dfactor = self.decimate(time_range)
        for line, d in zip(self.lines, data):
            line.set_data(t, d)
        self.title_func(ax.figure, dfactor)

def plot_pose(data, filename=None):
    t = get_time_vector(data)
    rows = 5
    cols = 2
    names = data.dtype.names
    colors = sns.color_palette('husl', len(names))
    fig, axes = plt.subplots(rows, cols, sharex=True)
    axes = axes.ravel()

    base_title = 'bicycle pose'
    if filename is not None:
        base_title += ' (file \'{}\')'.format(filename)
    title_size = mpl.rcParams['font.size'] + 2

    def set_title_func(f, x):
        title = base_title
        if x > 0:
            title = '{}\ndecimation factor {}'.format(base_title, x)
        f.suptitle(title, size=title_size)

    angle_names = ('pitch', 'yaw', 'roll', 'steer', 'rear_wheel')
    data = tuple(data[name] if name not in angle_names else
                 data[name]*180/np.pi for name in names)

    full_time_range = (t.min(), t.max()) # start with full range of data
    ts_display = TimeseriesDisplay(None, data, t, set_title_func)
    td, datad, dfactor = ts_display.decimate(full_time_range)
    set_title_func(fig, dfactor)

    lines = []
    for n, dd in enumerate(datad):
        ax = axes[n]
        name = names[n]

        if name in angle_names:
            #d *= 180 / np.pi # convert radians to degrees
            labelname = name + ' [°]'
        elif name in ('x', 'y'):
            labelname = name + ' [m]'
        elif name == 'v':
            labelname = name + ' [m/s]'
        elif name == 'timestamp':
            labelname = name + ' [ms]'

        lines.append(ax.plot(td, dd, label=labelname, color=colors[n])[0])
        ax.legend()
        ax.set_autoscale_on(False)
        ax.callbacks.connect('xlim_changed', ts_display.ax_update)

    ts_display.lines = lines

    #for name, n in zip(data.dtype.names, range(len(axes))):
    #    if name == 'timestamp':
    #        x = data['x']
    #        y = data['y']
    #        husl20 = sns.color_palette('husl',20)
    #        trajectory = np.array([x, y]).T.reshape(-1, 1, 2)
    #        segments = np.concatenate([trajectory[:-1], trajectory[1:]],
    #                                  axis=1)
    #        lc = LineCollection(segments, cmap=ListedColormap(husl20))
    #        lc.set_array(t)
    #        ax = plt.subplot2grid((rows, cols), (0, 0), rowspan=2)
    #        ax.add_collection(lc)
    #        minima = np.min(trajectory, axis=0)[0]
    #        maxima = np.max(trajectory, axis=0)[0]
    #        ax.set_xlim(minima[0], maxima[0])
    #        ax.set_ylim(minima[1], maxima[1])
    #        ax.set_xlabel('x')
    #        ax.set_ylabel('y')
    #        ax.plot(trajectory[-1, 0, 0], trajectory[-1, 0, 1],
    #                color=husl20[0],
    #                label='rear contact point trajectory')
    #        ax.invert_yaxis()
    #        ax.xaxis.tick_top()
    #        ax.xaxis.set_label_position('top')
    #        ax.set_aspect('equal', 'datalim')
    #        ax.legend()
    #        axes[2] = ax
    #        axes[0].axis('off')
    #    else:
    #        if name == 'x':
    #            ax = axes[1]
    #        elif name == 'y':
    #            ax = axes[3]
    #            ax.invert_yaxis()
    #        else:
    #            ax = axes[n + 2]
    #        x = data[name]
    #        if name in ('pitch', 'yaw', 'roll', 'steer', 'rear_wheel'):
    #            x = x * 180 / np.pi
    #            labelname = name + ' [°]'
    #        elif name in ('x', 'y'):
    #            labelname = name + ' [m]'
    #        elif name in ('v'):
    #            labelname = name + ' [m/s]'
    #        ax.plot(t, x, label=labelname, color=colors[n])
    #        ax.legend()

    axes[-1].set_xlabel('time [ms]')
    axes[-2].set_xlabel('time [ms]')

    return fig, ts_display


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
        plot_pose_vispy(pose_data)
    else:
        # keep a reference to the TimeseriesDisplay object so that it doesn't
        # get garbage collected
        fig, ts = plot_pose(pose_data, filename)
        plt.show()

    sys.exit(0)
