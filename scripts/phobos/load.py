from phobos import cobs
import numpy as np

def pose_logfile(filename, dtype):
    bytedata = None
    gitsha1 = None
    num_errors = 0
    data = bytearray()

    with open(filename, 'rb') as f:
        bytedata = f.read()

    packet_start = 0;
    packet_end = 0;
    mv = memoryview(bytedata)
    for i, byte in enumerate(bytedata):
        #if byte == b'\x00': # COBS packet delimiter
        if byte == 0: # b'\x00': # COBS packet delimiter
                      # Python 3.3+ returns an int when iterating over a
                      # 'bytes' object??
            packet_end = i
            try:
                pose = cobs.decode(mv[packet_start:packet_end], True)
            except cobs.DecodeError as e:
                # further testing necessary to determine cause of:
                # 'not enough input bytes for length code'
                # print(e)
                num_errors += 1
                # add a packet with all floats as nan
                data.extend(bytearray([0xff] * dtype.itemsize))
            else:
                if len(pose) != dtype.itemsize:
                    if len(pose) == 7: # this is sent as the first packet
                        gitsha1 = pose.decode('ascii')
                    else:
                        print('invalid packet size: {0} not {1}'.format(
                            len(pose), dtype.itemsize))
                else:
                    data.extend(pose)
            finally:
                packet_start = i + 1
                packet_end = i + 1

    return gitsha1, np.frombuffer(data, dtype), num_errors
