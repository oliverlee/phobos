#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np

from phobos import load
from phobos import pose
from phobos import pb

def get_simulation_types():
    option_file = '../projects/proto/simulation.options'
    max_repeated = pb.make_max_repeated_dict(option_file)

    proto_files = ['../projects/proto/pose.proto',
                   '../projects/proto/simulation.proto']
    proto = pb.import_modules(proto_files)[-1]
    dtype = pb.get_np_dtype(proto.SimulationMessage.DESCRIPTOR, max_repeated)
    return proto, dtype


def load_messages(filename):
    proto, _ = get_simulation_types()

    def deserialize_callback(packet):
        return pb.decode_delimited(proto.SimulationMessage(), packet)

    return load.cobs_framed_log(filename, deserialize_callback, True)


def get_records_from_messages(messages):
    _, dtype = get_simulation_types()
    records = np.recarray((len(messages),), dtype)
    for rec, msg in zip(records, messages):
        pb.set_record_from_message(rec, msg)
    return records


"""
if __name__ == '__main__':
    _, dtype, desc = pose.parse_format()
    if len(sys.argv) < 2:
        print('Usage: {} <pose_log_file>\n\nLoad pose data'.format(__file__))
        print('    <pose_log_file>\tFile containing samples in ' +
              'the given packed struct binary format:\n')
        print(desc)
        sys.exit(1)

    filename = os.path.realpath(sys.argv[1])
    gitsha1, data, num_errors = load.pose_log(filename, dtype)
    print('firmware version {0}'.format(gitsha1))
    print('read {0} total packets, {1} decode errors'.format(
        len(data), num_errors))

    sys.exit(0)

"""

if __name__ == '__main__':
    messages = load_messages(sys.argv[1])
    print('got {} message(s)'.format(len(messages)))
    records = get_records_from_messages(messages)
