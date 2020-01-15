#!/usr/bin/python3
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtCore
import collections
import numpy as np

class EasyPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure()

        super().__init__(self.fig)
        self.setParent(parent)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(60)

        # Add subplots
        self.axes = {}

        # Add plots
        self.plots = {}

        self.nb_point = 360

        # Declare plot data
        self.data = {}

    def add_subplot(self, pos, title, min, max, polar):
        self.axes[pos] = self.fig.add_subplot(pos, polar=polar)
        self.axes[pos].set_title(title)
        self.axes[pos].set_ylim(min, max)

    def add_plot(self, pos, label):
        self.plots[(pos, label)] = self.axes[pos].plot([], [], label=label, marker='o', markersize=1, linestyle='None')[0]
        self.data[(pos, label)] = [np.linspace(0,2*np.pi,self.nb_point),
                                   np.zeros(self.nb_point)]
        self.axes[pos].legend()

        return self.data[(pos, label)]

    def update_figure(self):
        for key in self.plots:
            self.plots[key].set_data(self.data[key])

        for key in self.axes:
            self.axes[key].relim()
            self.axes[key].autoscale_view(True,True,False)

        self.draw()
