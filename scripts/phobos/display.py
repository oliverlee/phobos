import numpy as np
import scipy
import scipy.signal

# Class used for updating plots in callbacks.

class DecimatingDisplay(object):
    def __init__(self, data, t, dt, title_func, lines, lc, markers, histf):
        # assume lines and data have the same order
        # and first two data elements are x, y
        self.lines = lines
        self.data = data
        self.t = t
        self.dt = dt
        self.title_func = title_func
        self.lc = lc # line collection for trajectory
        self.markers = markers
        self.histf = histf

    def decimate(self, time_range):
        decimate_target_length = 5000
        tmin, tmax = time_range
        indices = np.where((self.t >= tmin) & (self.t <= tmax))[0]
        decimation_factor = int(len(indices) / decimate_target_length)
        f = lambda x: x[indices]
        if decimation_factor > 0:
            f = lambda x: scipy.signal.decimate(x[indices],
                                                decimation_factor,
                                                zero_phase=True)

        return f(self.t), (f(d) for d in self.data), decimation_factor, indices

    def ax_update(self, ax):
        time_range = ax.get_xlim()
        t, data, dfactor, indices = self.decimate(time_range)
        self.title_func(ax.figure, dfactor)

        def replot(line, t, d):
            line.set_data(t, d)
            ymin = d.min()
            ymax = d.max()
            dy = ymax - ymin
            margin = max(dy * 0.1, 0.001)
            line.axes.set_ylim(ymin - margin, ymax + margin)

        x = None
        y = None
        xlim = None
        for line, d in zip(self.lines, data):
            if line is not None:
                replot(line, t, d)
            # save decimated x and y samples
            if x is None:
                x = d
                xlim = line.axes.get_ylim()
            elif y is None:
                y = d

        # last line is 'dt' which doesn't have a corresponding data element
        line = self.lines[-1]
        dt = self.dt[indices[:-1]]
        replot(line, t[:-1], dt)

        if self.lc is not None:
            trajectory = np.array([x, y]).T.reshape(-1, 1, 2)
            segments = np.concatenate([trajectory[:-1], trajectory[1:]],
                                      axis=1)
            self.lc.set_segments(segments)
            self.lc.set_array(t)
            self.lc.axes.set_xlim(xlim)
            self.markers[0].set_data(x[0], y[0])
            self.markers[1].set_data(x[-1], y[-1])

        if self.histf is not None:
            dt = self.dt[indices[:-1]]
            self.histf(dt)
