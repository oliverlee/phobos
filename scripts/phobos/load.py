import gzip
import os
import numpy as np
from phobos import cobs
from phobos import pose
from phobos import pb


DeserializeMissingDataError = pb.MissingDataError
DeserializeExtraDataError = pb.ExtraDataError

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


def cobs_framed_log(filename, packet_decode_callback=None,
                    multipacket_message=False):

    if multipacket_message and packet_decode_callback is None:
        msg = ('packet_decode_callback must be defined if '
               'multipacket_message is True.')
        raise ValueError(msg)

    bytedata = None
    packets = []
    datums = []

    if filename.endswith('.gz'):
        openf = lambda x: gzip.open(x, 'rb')
    else:
        openf = lambda x: open(x, 'rb')
    with openf(filename) as f:
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
                if packet_decode_callback is not None:
                    if multipacket_message:
                        packets.append(unstuffed_packet)
                        datum = __decode_multipacket(packet_decode_callback,
                                                     packets)
                        # no real way to tell if decoding fails as we
                        # assume it's normally due to not enough data
                        # or too much data (which could happen if we
                        # miss the beginning of a message)
                        if datum:
                            datums.append(datum)
                    else:
                        datum = packet_decode_callback(unstuffed_packet)
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


def __decode_multipacket(callback, packet_list):
    if not packet_list:
        return

    data = bytes().join(packet_list)
    try:
        message = callback(data)
    except DeserializeMissingDataError:
        # wait for next packet
        return None
    except DeserializeExtraDataError:
        # throw away the data
        print('Extra data found when attempting to deserialize. '
              'Packets dropped')
        packet_list.clear()
        return None
    else:
        packet_list.clear()
        return message
