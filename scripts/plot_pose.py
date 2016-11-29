#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pylab as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
import seaborn as sns

from phobos import pose
from phobos import cobs


script_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.join(script_dir, os.path.pardir, 'projects')
flimnap_file = os.path.join(project_dir, 'flimnap', 'main.cc')

flatgrey = '#95a5a6'


def process_file(filename, dtype):
    with open(filename, 'rb') as f:
        gitsha1 = None
        num_errors = 0
        packet = b''
        pose_data = b''

        byte = f.read(1)
        while byte:
            if byte == b'\x00': # COBS packet delimiter
                try:
                    data = cobs.decode(packet)
                except cobs.DecodeError as e:
                    # further testing necessary to determine cause of:
                    # 'not enough input bytes for length code'
                    # print(e)
                    num_errors += 1
                    # add a packet with all floats as nan
                    pose_data += b'\xff' * dtype.itemsize
                else:
                    if len(data) != dtype.itemsize:
                        if len(data) == 7: # this is sent as the first packet
                            gitsha1 = data.decode('ascii')
                        else:
                            print('invalid packet size: {0} not {1}'.format(
                                len(data), dtype.itemsize))
                    else:
                        pose_data += data
                finally:
                    packet = b''
            else:
                packet += byte
            byte = f.read(1)
    return gitsha1, np.frombuffer(pose_data, dtype), num_errors


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
    title = 'bicycle pose'
    if filename is not None:
        title += ' (file \'{}\')'.format(filename)
    fig.suptitle(title, size=mpl.rcParams['font.size'] + 2)


if __name__ == '__main__':
    _, dtype, desc = pose.parse_format(flimnap_file)
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
    gitsha1, pose_data, num_errors = process_file(filename, dtype)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(pose_data), num_errors))

    plot_pose(pose_data, filename)
    plt.show(block=False)

    sys.exit(0)
