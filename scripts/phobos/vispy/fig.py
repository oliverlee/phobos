# -*- coding: utf-8 -*-
# Copyright (c) 2015, Vispy Development Team.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.

import numpy as np
from vispy.scene import SceneCanvas, Histogram
from vispy.geometry import Rect
from phobos.vispy.plotwidget import PlotWidget


def _do_the_histogramming(data, bins, orientation):
    X, Y = (0, 1) if orientation == 'h' else (1, 0)
    data, bin_edges = np.histogram(data, bins)
    # construct our vertices
    rr = np.zeros((3 * len(bin_edges) - 2, 3), np.float32)
    rr[:, X] = np.repeat(bin_edges, 3)[1:-1]
    rr[1::3, Y] = data
    rr[2::3, Y] = data
    bin_edges.astype(np.float32)
    # and now our tris
    tris = np.zeros((2 * len(bin_edges) - 2, 3), np.uint32)
    offsets = 3 * np.arange(len(bin_edges) - 1,
                            dtype=np.uint32)[:, np.newaxis]
    tri_1 = np.array([0, 2, 1])
    tri_2 = np.array([2, 0, 3])
    tris[::2] = tri_1 + offsets
    tris[1::2] = tri_2 + offsets
    return (rr, tris)


class Fig(SceneCanvas):
    """Create a figure window

    Parameters
    ----------
    bgcolor : instance of Color
        Color to use for the background.
    size : tuple
        Size of the figure window in pixels.
    show : bool
        If True, show the window.
    **kwargs : dict
        Keywoard arguments to pass to `SceneCanvas` base class.

    Notes
    -----
    You can create a Figure, PlotWidget, and diagonal line plot like this::

        >>> from vispy.plot import Fig
        >>> fig = Fig()
        >>> ax = fig[0, 0]  # this creates a PlotWidget
        >>> ax.plot([[0, 1], [0, 1]])

    See the gallery for many other examples.

    See Also
    --------
    PlotWidget : the axis widget for plotting
    SceneCanvas : the super class
    """
    def __init__(self, bgcolor='w', size=(800, 600), show=True, sharex=False,
                 **kwargs):
        self._plot_widgets = []
        self._grid = None  # initialize before the freeze occurs
        self._sharex = sharex
        super(Fig, self).__init__(bgcolor=bgcolor, keys='interactive',
                                  show=show, size=size, **kwargs)
        self._grid = self.central_widget.add_grid()
        self._grid._default_class = PlotWidget

        if self._sharex:
            self.events.mouse_move.connect(self._linked_plots_event,
                                           position='last')
            self.events.mouse_wheel.connect(self._linked_plots_event,
                                            position='last')

    @property
    def plot_widgets(self):
        """List of the associated PlotWidget instances"""
        return tuple(self._plot_widgets)

    def __getitem__(self, idxs):
        """Get an axis"""
        pw = self._grid.__getitem__(idxs)
        self._plot_widgets += [pw]
        return pw

    def _get_new_x_axis(self):
        d = {}
        for pw in (pw for pw in self.plot_widgets if pw.linked):
            rect = pw.view.camera.rect
            xaxis = (rect.left, rect.right)
            if xaxis in d:
                d[xaxis] += 1
            else:
                d[xaxis] = 1
        for key, value in d.items():
            if value == 1:
                return key
        return None

    def _set_all_x_axes(self, xaxis):
        if xaxis is not None:
            left, right = xaxis
            for pw in self.plot_widgets:
                if pw.linked:
                    rect = Rect(pw.view.camera.rect)
                    rect.left = left
                    rect.right = right
                    pw.view.camera.rect = rect
                else:
                    for c in pw.view.scene.children:
                        if isinstance(c, Histogram) and pw.data is not None:
                            t, data, bins, orientation = pw.data
                            subset = np.where((t >= left) &
                                              (t <= left + right))
                            (rr, tris) = _do_the_histogramming(data[subset],
                                                               bins,
                                                               orientation)
                            c.set_data(rr, tris, color=c.color)

    def _linked_plots_event(self, event):
        if event.handled:
            xaxis = self._get_new_x_axis()
            self._set_all_x_axes(xaxis)
