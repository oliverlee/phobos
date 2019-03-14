import numpy as np
import scipy.signal


import io
import contextlib
import matlab.engine
_engine = matlab.engine.start_matlab() # global matlab engine


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


class MatlabWorkspaceObject(object):
    def __init__(self, name, value=None, command=None, engine=_engine):
        self.name = name
        self.engine = engine

        errstr = 'can\'t set a MatlabWorkspaceObject with value and command'
        assert not (value is not None and command is not None), errstr

        if value is not None:
            engine.workspace[name] = value
        if command is not None:
            _matlab_eval('{} = {}'.format(name, command), engine=engine)

    def __str__(self):
        return self.name

    def __repr__(self):
        f = io.StringIO()
        with contextlib.redirect_stdout(f):
            _matlab_eval(str(self), verbose=True, engine=self.engine)
        output = f.getvalue()
        f.close()
        return output

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.delete()

    def delete(self):
        return _matlab_eval('clear {}'.format(self), engine=self.engine)

def _matlab_eval(command, verbose=False, engine=_engine):
    if verbose:
        engine_io = io.StringIO()
    else:
        engine_io = None

    engine.eval(command, nargout=0, stdout=engine_io)

    if verbose:
        print(engine_io.getvalue())
        engine_io.close()

def help(command):
    _matlab_eval('help {}'.format(command), verbose=True)

def iddata(u, y, dt, matlab_name, engine=_engine):
    _u = matlab.double(u.tolist())
    _u.reshape((len(u), 1))
    _y = matlab.double(y.tolist())
    _y.reshape((len(y), 1))

    mwo_u = MatlabWorkspaceObject('py_iddata_u', value=_u, engine=engine)
    mwo_y = MatlabWorkspaceObject('py_iddata_y', value=_y, engine=engine)
    with mwo_u, mwo_y:
        mwo_data = MatlabWorkspaceObject(matlab_name,
                command='iddata({}, {}, {})'.format(mwo_y, mwo_u, dt),
                engine=engine)

    return mwo_data

def armax(mwo_iddata, na, nb, nc, nk=0, nrmse=False):
    engine = mwo_iddata.engine

    mwo_model = MatlabWorkspaceObject('py_armax_model',
            command='armax({}, [{} {} {} {}])'.format(
                mwo_iddata, na, nb, nc, nk),
            engine=engine)

    with mwo_model:
        b = np.array(engine.eval('{}.B'.format(mwo_model))).reshape((-1,))
        a = np.array(engine.eval('{}.A'.format(mwo_model))).reshape((-1,))

        if nrmse:
            nrmse_value = engine.eval('{}.Report.Fit.FitPercent'.format(
                mwo_model))
            return b, a, nrmse_value
    return b, a

def bj(mwo_iddata, nb, nc, nd, nf, nk=0, nrmse=False):
    engine = mwo_iddata.engine

    mwo_model = MatlabWorkspaceObject('py_bj_model',
            command='bj({}, [{} {} {} {} {}])'.format(
                mwo_iddata, nb, nc, nd, nf, nk),
            engine=engine)

    with mwo_model:
        b = np.array(engine.eval('{}.B'.format(mwo_model))).reshape((-1,))
        a = np.array(engine.eval('{}.F'.format(mwo_model))).reshape((-1,))

        if nrmse:
            nrmse_value = engine.eval('{}.Report.Fit.FitPercent'.format(
                mwo_model))
            return b, a, nrmse_value
    return b, a
