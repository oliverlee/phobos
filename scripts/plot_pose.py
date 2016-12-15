#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
import scipy
import scipy.signal
import matplotlib as mpl
import matplotlib.pylab as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import seaborn as sns

from phobos import load
from phobos import pose
from phobos.display import DecimatingDisplay


def plot_pose_vispy(data):
    t = pose.get_time_vector(data)
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


def _plot_trajectory(ax, x, y, t=None, t_label=None,
                     xpos='bottom', ypos='left',
                     xinvert=False, yinvert=False,
                     **kwargs):
    # clear axes if it already contains objects
    try:
        # remove colorbar if added before
        ax.figure.delaxes(ax._colorbar)
        ax.set_axes_locator(ax._locator)
        del ax._colorbar
        del ax._locator
    except AttributeError:
        pass
    ax.clear()

    # calculate trajectory and change color over indices
    trajectory = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([trajectory[:-1], trajectory[1:]], axis=1)
    lc = mpl.collections.LineCollection(segments, **kwargs)
    if t is not None:
        lc.set_array(t)

    # plot the trajectory
    ax.add_collection(lc)

    # plot a single point to use the legend
    color = None
    if 'cmap' in kwargs:
        color = kwargs['cmap'](0)
    ax.plot(trajectory[-1, 0, 0], trajectory[-1, 0, 1],
            label='rear contact point trajectory',
            color=color)
    # plot start and end points in the legend
    colors = sns.color_palette('deep', 3)
    marker_start = ax.plot(x[0], y[0], color=colors[1], linestyle='None',
                           marker='o', markersize=6, label='start')[0]
    marker_end = ax.plot(x[-1], y[-1], color=colors[2], linestyle='None',
                         marker='o', markersize=6, label='end')[0]
    ax.legend()

    # flip around axes if requested
    ax.xaxis.set_ticks_position(xpos)
    ax.xaxis.set_label_position(xpos)
    ax.yaxis.set_ticks_position(ypos)
    ax.yaxis.set_label_position(ypos)
    if xinvert:
        ax.invert_xaxis()
    if yinvert:
        ax.invert_yaxis()

    if t is not None:
        make_axes_locatable(ax)
        ax._locator = ax.get_axes_locator()

        # get a new locator so we can restore this axis if necessary
        divider = make_axes_locatable(ax)
        cax = divider.append_axes('right', size=0.05, pad=0.05)
        cbar = plt.colorbar(lc, cax=cax)

        if t_label is not None:
            cbar.set_label(t_label)
        ax._colorbar = cax
    return lc, (marker_start, marker_end)

    # set plot limits and set x and y aspect to be equal
    ax.set_aspect('equal', 'datalim')
    minima = np.min(trajectory, axis=0)[0]
    maxima = np.max(trajectory, axis=0)[0]
    ax.set_xlim(minima[0], maxima[0])
    ax.set_ylim(minima[1], maxima[1])


def _plot_histogram(ax, data, logscale=True, **kwargs):
    ax.clear()
    if 'alpha' not in kwargs:
        kwargs['alpha'] = 0.9
    if 'linewidth' not in kwargs:
        kwargs['linewidth'] = 0
    ax.hist(data, **kwargs)
    if logscale:
        ax.set_yscale('log')
    if 'label' in kwargs:
        ax.legend()


def plot_pose(data, filename=None, gitsha1=None):
    t = pose.get_time_vector(data)
    ts = data['timestamp']
    dt = (ts - np.roll(ts, 1))[1:] # length is now 1 shorter than data
    rows = round((len(data.dtype.names) + 4) / 2)
    cols = 2
    names = data.dtype.names
    colors = sns.color_palette('husl', len(names))
    fig, axes = plt.subplots(rows, cols, sharex=True)
    axes = axes.ravel()

    base_title = 'bicycle pose'
    if filename is not None:
        base_title += ' (file \'{}\''.format(filename)
        if gitsha1 is not None:
            base_title += ', {}'.format(gitsha1)
        base_title += ')'

    title_size = mpl.rcParams['font.size'] + 2

    def set_title_func(f, x):
        title = base_title
        if x > 0:
            title = '{}\ndecimation factor {}'.format(base_title, x)
        f.suptitle(title, size=title_size)

    # convert angle data from radians to degrees
    angle_names = ('pitch', 'yaw', 'roll', 'steer', 'rear_wheel')
    data_np = []
    for name in names:
        if name in angle_names:
            data_np.append(data[name] * 180/np.pi)
        #elif name == 'timestamp':
        #    continue
        else:
            data_np.append(data[name])
        print('appending', name, 'to data')

    # plot objects to be added later
    dd_display = DecimatingDisplay(data_np, t, dt, set_title_func,
                                   None, None, None, None)

    full_time_range = (t.min(), t.max()) # start with full range of data
    td, datad, dfactor, indices = dd_display.decimate(full_time_range)
    set_title_func(fig, dfactor)

    lines = []
    for n, dd in enumerate(datad):
        axes_index = n + 4
        name = names[n]

        if name in angle_names:
            labelname = name + ' [°]'
        elif name in ('x', 'y'):
            labelname = name + ' [m]'
        elif name == 'v':
            labelname = name + ' [m/s]'
        elif 'time' in name:
            labelname = name + ' [us]'
        else:
            labelname = name
        print('creating line for', name)

        ax = axes[axes_index]
        lines.append(ax.plot(td, dd, label=labelname, color=colors[n])[0])
        ax.legend()
        ax.set_autoscale_on(False)
        ax.callbacks.connect('xlim_changed', dd_display.ax_update)
        if name == 'y':
            ax.invert_yaxis()

    # create dt plot
    ax = axes[3]
    lines.append(ax.plot(t[:-1], dt[indices[:-1]],
                 label='dt [us]', color=colors[-1])[0])
    ax.legend()
    ax.set_autoscale_on(False)
    ax.callbacks.connect('xlim_changed', dd_display.ax_update)

    axes[-1].set_xlabel('time [us]')
    axes[-2].set_xlabel('time [us]')

    # display trajectory plot
    ax = plt.subplot2grid((rows, cols), (0, 0), rowspan=2, sharey=axes[5])
    husl100 = mpl.colors.ListedColormap(sns.color_palette('husl', 100))
    lc, markers = _plot_trajectory(ax, data['x'][::100], data['y'][::100],
                                   t[::100], xpos='top', yinvert=True,
                                   cmap=husl100)
    axes[0] = ax # overwrite original axes

    # display histogram of sample time
    ax = plt.subplot2grid((rows, cols), (0, 1))
    histf = lambda dt: _plot_histogram(
            ax, dt, logscale=True, color=colors[-1],
            label='loop time [us]\nmax time = {}'.format(dt.max()))
    histf(dt)
    axes[1] = ax

    dd_display.lines = lines
    dd_display.lc = lc
    dd_display.markers = markers
    dd_display.histf = histf
    fig._timeseries_display = dd_display
    return fig


if __name__ == '__main__':
    _, dtype, desc = pose.parse_format(load.flimnap_file)
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
    gitsha1, pose_data, num_errors = load.pose_logfile(filename, dtype)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(pose_data), num_errors))

    if len(sys.argv) > 2 and sys.argv[2]:
        plot_pose_vispy(pose_data)
    else:
        # if there is 1 decode error, assume it is the first packet
        if num_errors == 1:
            fig = plot_pose(pose_data[1:], filename, gitsha1)
        else:
            fig = plot_pose(pose_data, filename, gitsha1)
        plt.show()

    sys.exit(0)
