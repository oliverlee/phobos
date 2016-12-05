#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
import scipy
import matplotlib.pylab as plt

from phobos import load
from phobos import pose
from phobos import util
from plot_pose import plot_pose

def print_velocity_stats(data, filename='', gitsha1=''):
    t = pose.get_time_vector(data)
    v = data['v']
    v_target = 2.5
    v_margin = 1.0
    print('determining positive velocity subset about {} m/s...'.format(
        v_target))
    indices = util.find_continguous_regions((v > 0) &
                                            (v >= (v_target - v_margin)) &
                                            (v <= (v_target + v_margin)))
    min_contiguous_samples = 10
    diff = indices[:, 1] - indices[:, 0]
    large_regions = np.where(diff > min_contiguous_samples)[0]
    t_subset = t[indices[large_regions, :]]
    dt = t_subset[:, 1] - t_subset[:, 0]

    region_index = dt.argmax()
    index = indices[large_regions, :][region_index, :]
    tmin, tmax = t[index]
    print('subset selected with from index {} ({} ms) to {} ({} ms)'.format(
        index[0], tmin, index[1], tmax))
    print('time span of {} ms'.format(tmax - tmin))

    data_subset = data[index[0]:index[1]]
    t_subset = t[index[0]:index[1]]
    v_subset = data_subset['v']

    print('-' * 50)
    stats = scipy.stats.describe(v_subset)
    for field in stats._fields:
        print('{} {}'.format(field, getattr(stats, field)))
    print('-' * 50)

    print('plotting subset (time {} ms redefined to 0)...'.format(tmin))
    fig = plot_pose(data_subset, filename, gitsha1)
    plt.show()
    return stats


if __name__ == '__main__':
    _, dtype, desc = pose.parse_format(load.flimnap_file)
    if len(sys.argv) < 2:
        print('Usage: {} <pose_log_file>\n\n'.format(__file__) +
              'Print velocity stats for a given pose log.\n\n' +
              'Arguments:\n')
        print('    <pose_log_file>\tFile containing samples in ' +
              'the given packed struct binary format:\n')
        print(desc)
        sys.exit(1)

    filename = os.path.realpath(sys.argv[1])
    gitsha1, data, num_errors = load.pose_logfile(filename, dtype)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(data), num_errors))

    print_velocity_stats(data, filename, gitsha1)

    sys.exit(0)
