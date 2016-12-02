#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np

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


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: {} <pose_log_file>\n\nLoad pose data'.format(__file__))
        print('    <pose_log_file>\tFile containing samples in ' +
              'the given packed struct binary format:\n')
        print(desc)
        sys.exit(1)

    #samples = convert.load_sample_log(sys.argv[1])
    filename = os.path.realpath(sys.argv[1])
    gitsha1, data, num_errors = load.pose_logfile(filename)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(data), num_errors))

    sys.exit(0)
