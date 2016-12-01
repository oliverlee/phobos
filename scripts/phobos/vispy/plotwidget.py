# -*- coding: utf-8 -*-
# Copyright (c) 2015, Vispy Development Team.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.

from vispy import scene


class PlotWidget(scene.Widget):
    """Widget to facilitate plotting

    Parameters
    ----------
    *args : arguments
        Arguments passed to the `ViewBox` super class.
    **kwargs : keywoard arguments
        Keyword arguments passed to the `ViewBox` super class.

    Notes
    -----
    This class is typically instantiated implicitly by a `Figure`
    instance, e.g., by doing ``fig[0, 0]``.

    See Also
    --------
    """
    def __init__(self, *args, **kwargs):
        self._fg = kwargs.pop('fg_color', 'k')
        self.grid = None
        self.camera = None
        self.title = None
        self.yaxis = None
        self.xaxis = None
        self._configured = False
        self.visuals = []
        self.section_y_x = None
        self.linked = False
        self.data = None

        super(PlotWidget, self).__init__(*args, **kwargs)
        self.grid = self.add_grid(spacing=0, margin=10)

        self.title = scene.Label("", font_size=16, color="#ff0000")

    def _configure_2d(self, fg_color=None):
        if self._configured:
            return

        if fg_color is None:
            fg = self._fg
        else:
            fg = fg_color

        #     +-------+-------+
        #     | yaxis |  view |
        #     |-------+-------+
        #     |       | xaxis |
        #     +-------+-------+

        # row 0
        # yaxis - column 0
        # view - column 1
        self.yaxis = scene.AxisWidget(orientation='left',
                                      text_color=fg,
                                      axis_color=fg, tick_color=fg)

        yaxis_widget = self.grid.add_widget(self.yaxis, row=0, col=0)
        yaxis_widget.width_max = 40

        self.view = self.grid.add_view(row=0, col=1,
                                       border_color='grey', bgcolor="#efefef")
        self.view.camera = 'panzoom'
        self.camera = self.view.camera

        # row 1
        # xaxis - column 1
        self.xaxis = scene.AxisWidget(orientation='bottom', text_color=fg,
                                      axis_color=fg, tick_color=fg)
        xaxis_widget = self.grid.add_widget(self.xaxis, row=1, col=1)
        xaxis_widget.height_max = 40

        self._configured = True
        self.xaxis.link_view(self.view)
        self.yaxis.link_view(self.view)


    def plot(self, data, color='k', symbol=None, line_kind='-', width=1.,
             marker_size=10., edge_color='k', face_color='b', edge_width=1.,
             linked=True):
        """Plot a series of data using lines and markers

        Parameters
        ----------
        data : array | two arrays
            Arguments can be passed as ``(Y,)``, ``(X, Y)`` or
            ``np.array((X, Y))``.
        color : instance of Color
            Color of the line.
        symbol : str
            Marker symbol to use.
        line_kind : str
            Kind of line to draw. For now, only solid lines (``'-'``)
            are supported.
        width : float
            Line width.
        marker_size : float
            Marker size. If `size == 0` markers will not be shown.
        edge_color : instance of Color
            Color of the marker edge.
        face_color : instance of Color
            Color of the marker face.
        edge_width : float
            Edge width of the marker.
        title : str | None
            The title string to be displayed above the plot
        xlabel : str | None
            The label to display along the bottom axis
        ylabel : str | None
            The label to display along the left axis.

        Returns
        -------
        line : instance of LinePlot
            The line plot.

        See also
        --------
        marker_types, LinePlot
        """
        self._configure_2d()
        line = scene.LinePlot(data, connect='strip', color=color,
                              symbol=symbol, line_kind=line_kind,
                              width=width, marker_size=marker_size,
                              edge_color=edge_color,
                              face_color=face_color,
                              edge_width=edge_width)
        self.view.add(line)
        self.view.camera.set_range()
        self.visuals.append(line)
        self.linked = linked
        return line

    def histogram(self, data, t=None, bins=10, color='w', orientation='h'):
        """Calculate and show a histogram of data

        Parameters
        ----------
        data : array-like
            Data to histogram. Currently only 1D data is supported.
        bins : int | array-like
            Number of bins, or bin edges.
        color : instance of Color
            Color of the histogram.
        orientation : {'h', 'v'}
            Orientation of the histogram.

        Returns
        -------
        hist : instance of Polygon
            The histogram polygon.
        """
        self._configure_2d()
        hist = scene.Histogram(data, bins, color, orientation)
        self.view.add(hist)
        self.view.camera.set_range()
        self.linked = False

        if t is not None:
            if t.shape != data.shape:
                raise ValueError
            self.data = (t, data, bins, orientation)
        return hist
