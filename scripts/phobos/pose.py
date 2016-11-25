#pose definition for flimnap
#
# example:
# struct __attribute__((__packed__)) pose_t {
#     float x; /* m */
#     float y; /* m */
#     float pitch; /* rad */
#     float yaw; /* rad */
#     float roll; /* rad */
#     float steer; /* rad */
#     float rear_wheel; /* rad */
#     float v; /* m/s */
#     uint8_t timestamp;  /* Converted from system ticks to milliseconds
#                          * and contains only the least significant bits */
# }; /* 33 bytes */

import os
import re
import struct


def parse_format(source_file):
    with open(source_file, 'r') as f:
        start_pose_definition = False
        pose_format = '<'
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
                    struct_size = struct.Struct(pose_format).size
                    assert struct_size == pose_size, \
                            '{0}, {1}'.format(pose_size, struct_size)
                    return (pose_format, pose_format_text)
                else:
                    words = line.split()
                    if words[1][-1] != ';':
                        # line is commented
                        assert words[0].startswith(('//', '/*', '*/', '*')), \
                                'Unexpected format for pose definition: ' + line
                        continue

                    t = words[0]
                    if t == 'float':
                        pose_format += 'f'
                    elif t == 'uint8_t':
                        pose_format += 'B'
                    else:
                        msg = 'Conversion for type {0} not handled'.format(t)
                        raise ValueError(msg)
                    pose_format_text += line


if __name__ == '__main__':
    local_dir = os.path.dirname(os.path.realpath(__file__))
    project_dir = os.path.join(local_dir, os.path.pardir, os.path.pardir, 'projects')
    flimnap_file = os.path.join(project_dir, 'flimnap', 'main.cc')
    print(parse_format(flimnap_file))
