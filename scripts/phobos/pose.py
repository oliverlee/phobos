# Old-style pose type for flimnap is defined in scripts/phobos/pose_def.cc

import os
import re
import struct
import numpy as np

script_dir = os.path.dirname(os.path.realpath(__file__))
pose_def_file = os.path.join(script_dir, 'pose_def.cc')


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
    discont = 2**16
    ts_offset = np.cumsum(overflow) * discont
    ts_offset[nan_index] = -(discont - 1) # nan values have time set to 0
    return ts + ts_offset


def parse_format(source_file=pose_def_file):
    with open(source_file, 'r') as f:
        start_pose_definition = False
        struct_format = '<'
        np_dtype = []
        pose_format_text = ''
        for line in f:
            if not line:
                raise EOFError('Pose not fully defined in file {0}'.format(
                    source_file))

            if not start_pose_definition:
                if 'struct __attribute__((__packed__)) pose_t {' in line:
                    start_pose_definition = True
                    pose_format_text += line
            else:
                match = re.search('}; /\* ([0-9]+) bytes \*/', line)
                if match: # end of pose definition
                    pose_size = int(match.group(1))
                    pose_format_text += line
                    struct_size = struct.Struct(struct_format).size
                    assert struct_size == pose_size, \
                            '{0}, {1}'.format(pose_size, struct_size)
                    assert np.dtype(np_dtype).itemsize == struct_size
                    return (struct_format, np.dtype(np_dtype), pose_format_text)
                else:
                    words = line.split()
                    if words[1][-1] != ';':
                        # line is commented
                        assert words[0].startswith(('//', '/*', '*/', '*')), \
                                'Unexpected format for pose definition: ' + line
                        continue

                    t = words[0]
                    if t == 'float':
                        struct_format += 'f'
                        np_dtype.append((words[1][:-1], 'f4'))
                    elif t == 'uint8_t':
                        struct_format += 'B'
                        np_dtype.append((words[1][:-1], 'u1'))
                    elif t == 'uint16_t':
                        struct_format += 'H'
                        np_dtype.append((words[1][:-1], 'u2'))
                    elif t == 'uint32_t':
                        struct_format += 'I'
                        np_dtype.append((words[1][:-1], 'u4'))
                    else:
                        msg = 'Conversion for type {0} not handled'.format(t)
                        raise ValueError(msg)
                    pose_format_text += line


if __name__ == '__main__':
    a, b, c = parse_format(pose_def_file)
    print(a)
    print(b)
    print(c)
