import numpy as np
import scipy.signal


def _get_signal(log, label):
    if label == 'u':
        return  log.records.input[:, 1], 'steer torque'
    elif label == 'u1':
        return log.kollmorgen_command_torque, 'command motor torque'
    elif label == 'u2':
        return log.kollmorgen_applied_torque, 'applied motor torque'
    elif label == 'r':
        return log.states[:, 1], 'reference steer angle'
    elif label == 'y':
        return log.measured_steer_angle, 'measured steer angle'
    else:
        raise ValueError('unknown signal label')

def rudin_shapiro(N):
    """
    Return first N terms of Rudin-Shapiro sequence
    https://en.wikipedia.org/wiki/Rudin-Shapiro_sequence
    Confirmed correct output to N = 10000:
    https://oeis.org/A020985/b020985.txt
    """
    def hamming(x):
        """
        Hamming weight of a binary sequence
        http://stackoverflow.com/a/407758/125507
        """
        return bin(x).count('1')

    out = np.empty(N, dtype=int)
    for n in range(N):
        b = hamming(n << 1 & n)
        a = (-1)**b
        out[n] = a

    return out

def power_spectrum(x, y=None, window_size=None, noverlap=None):
    if window_size is None:
        window_size = len(x)
        noverlap = None
    else:
        assert window_size > 0 and window_size <= len(x)

    if y is None:
        y = x

    x = scipy.signal.detrend(x)
    y = scipy.signal.detrend(y)

    def _spectrum(index):
        X = np.fft.rfft(x[index], norm='ortho')
        Y = np.fft.rfft(y[index], norm='ortho')

        return Y*np.conj(X)

    if noverlap is not None:
        nwindows = int((len(x) - window_size)//noverlap)
    else:
        nwindows = 1

    Syx = np.zeros(np.fft.rfftfreq(window_size).shape, dtype=np.complex)
    for i in range(nwindows):
        if i == 0:
            index = slice(-window_size, None)
        else:
            index = slice(int(-window_size - i*noverlap), int(-i*noverlap))
        Syx += _spectrum(index)

    return Syx/nwindows

def frequency_response(x, y,
                       data=None, skipn=None,
                       window_size=None, noverlap=None):
    if data is not None:
        x = _get_signal(data, x)[0]
        y = _get_signal(data, y)[0]

    if skipn is not None:
        x = x[skipn:]
        y = y[skipn:]

    if window_size is not None:
        assert window_size > 0 and window_size <= len(x)

    Sxx = power_spectrum(x, x, window_size, noverlap)
    Syx = power_spectrum(x, y, window_size, noverlap)
    return Syx/Sxx

def coherence(x, y,
              data=None, skipn=None,
              window_size=None, noverlap=None):
    if data is not None:
        x = _get_signal(data, x)[0]
        y = _get_signal(data, y)[0]

    if skipn is not None:
        x = x[skipn:]
        y = y[skipn:]

    if window_size is not None:
        assert window_size <= len(x)

    Sxx = power_spectrum(x, x, window_size, noverlap)
    Syx = power_spectrum(x, y, window_size, noverlap)
    Syy = power_spectrum(y, y, window_size, noverlap)
    return np.power(np.abs(Syx), 2)/(Sxx*Syy)
