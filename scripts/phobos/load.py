import os
import numpy as np
from phobos import cobs
from phobos import pose


class DeserializeError(Exception):
    pass


def pose_log(filename, dtype=None):
    if dtype is None:
        _, dtype, _ = pose.parse_format(pose.pose_def_file)

    gitsha1 = None
    packets = cobs_framed_log(filename)
    data = bytearray()
    for p in packets:
        if len(p) == 7:
            gitsha1 = p.decode('ascii')
        elif len(p) != dtype.itemsize:
            # replace malformed packet with a "zero" packet
            data.extend(bytearray([0xff] * dtype.itemsize))
        else:
            data.extend(p)

    return gitsha1, np.frombuffer(data, dtype), 0


def cobs_framed_log(filename, packet_callback=None):
    bytedata = None
    packets = []
    datums = []

    with open(filename, 'rb') as f:
        bytedata = f.read()
    mv = memoryview(bytedata)

    packet_start = 0;
    packet_end = 0;
    num_errors = 0;
    for i, byte in enumerate(bytedata):
        if byte == 0:
            packet_end = i
            try:
                unstuffed_packet = cobs.decode(mv[packet_start:packet_end])
            except cobs.DecodeError as e:
                print(e)
                num_errors += 1
                # TODO handle unstuff errors, add error callback?
            else:
                # TODO may need to change how a packet is deserialized for
                # messages spanning multiple packets
                if packet_callback is not None:
                    datum = packet_callback(unstuffed_packet)
                    if datum:
                        datums.append(datum)
                    else:
                        msg = 'Unable to deserialize packet: {}'.format(
                                unstuffed_packet)
                        raise DeserializeError(msg)
                else:
                    packets.append(unstuffed_packet)
            finally:
                packet_start = i + 1
                packet_end = i + 1

    if num_errors:
        print("{} error(s) when unstuffing file {}".format(num_errors,
                                                           filename))
    if datums:
        return datums
    return packets
