#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import (QWidget, QPushButton, QDoubleSpinBox, QVBoxLayout, QLineEdit, QHBoxLayout, QFormLayout, QGroupBox, QApplication, QSizePolicy)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.animation as animation
import numpy as np
import yaml
from PyQt5 import QtCore
from binserial import BinSerial
import threading
import time

class MyMplCanvas(FigureCanvas):
	"""Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
	def __init__(self, parent=None, width=5, height=4, dpi=100):
		fig = Figure(figsize=(width, height), dpi=dpi)
		self.axes = fig.add_subplot(111)

		self.compute_initial_figure()

		FigureCanvas.__init__(self, fig)
		self.setParent(parent)

		FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
		FigureCanvas.updateGeometry(self)

	def compute_initial_figure(self):
		pass


class BeautifulPlot(MyMplCanvas):
	def __init__(self, *args, **kwargs):
		MyMplCanvas.__init__(self, *args, **kwargs)
		self.compute_initial_figure()
		timer = QtCore.QTimer(self)
		timer.timeout.connect(self.update_figure)
		timer.start(60)

		self.d = [[],[]]
		self.axes.set_title('Motor Response')
		self.axes.legend()

	def compute_initial_figure(self):
		self.p, = self.axes.plot([], [], 'b', label="Encoder position")

	def update_figure(self):
		#self.axes.cla()
		#self.axes.plot(self.d[0],self.d[1], 'r')
		self.p.set_data(self.d)
		self.axes.relim()
		self.axes.autoscale_view(True,True,True)
		self.draw()

	def addData(self, x_s, y_s):
		self.d[0] += x_s
		self.d[1] += y_s
		if len(self.d[0]) > 1000:
			self.d[0] = self.d[0][-1000:]
			self.d[1] = self.d[1][-1000:]


class PidInterface(QWidget):
	def __init__(self):
		super().__init__()
		self.init_parameters()
		self.init_ui()
		self.bser = BinSerial(self.port_name, self.baud_rate)

		self.read_thread = threading.Thread(target=self.read_output)
		self.read_thread.start()

	def init_parameters(self):
		with open('config.yml', 'r') as f:
			config = yaml.load(f.read())
		self.port_name = config['port_name']
		self.baud_rate = config['baud_rate']
		self.kp = config['kp']
		self.ki = config['ki']
		self.kd = config['kd']
		self.sample_time = config['sample_time']
		self.reference = config['reference']

	def set_kp(self, new_kp):
		self.kp = new_kp

	def set_ki(self, new_ki):
		self.ki = new_ki

	def set_kd(self, new_kd):
		self.kd = new_kd

	def set_reference(self, new_reference):
		self.reference = new_reference

	def init_ui(self):
		# Parameters
		self.kp_spin = QDoubleSpinBox()
		self.kp_spin.setMinimum(0)
		self.kp_spin.setMaximum(float('inf'))
		self.kp_spin.setValue(self.kp)
		self.kp_spin.valueChanged.connect(self.set_kp)

		self.ki_spin = QDoubleSpinBox()
		self.ki_spin.setMinimum(0)
		self.ki_spin.setMaximum(float('inf'))
		self.ki_spin.setValue(self.ki)
		self.ki_spin.valueChanged.connect(self.set_ki)

		self.kd_spin = QDoubleSpinBox()
		self.kd_spin.setMinimum(0)
		self.kd_spin.setMaximum(float('inf'))
		self.kd_spin.setValue(self.kd)
		self.kd_spin.valueChanged.connect(self.set_kd)

		self.reference_spin = QDoubleSpinBox()
		self.reference_spin.setMinimum(0)
		self.reference_spin.setMaximum(float('inf'))
		self.reference_spin.setValue(self.reference)
		self.reference_spin.valueChanged.connect(self.set_reference)

		parameters_layout = QFormLayout()
		parameters_layout.addRow('kp', self.kp_spin)
		parameters_layout.addRow('ki', self.ki_spin)
		parameters_layout.addRow('kd', self.kd_spin)
		parameters_layout.addRow('reference', self.reference_spin)

		parameters_group = QGroupBox('Measure parameters')
		parameters_group.setLayout(parameters_layout)

		apply_button = QPushButton('Apply')
		apply_button.clicked.connect(self.sent_parameters)

		pid_layout = QVBoxLayout()
		pid_layout.addWidget(parameters_group)
		pid_layout.addWidget(apply_button)
		pid_layout.addStretch()

		# Display
		self.plot = BeautifulPlot(self)

		# Main
		main_layout = QHBoxLayout()
		main_layout.addLayout(pid_layout)
		main_layout.addWidget(self.plot)

		self.setLayout(main_layout)

	def sent_parameters(self):
		"""run the step response and get the measures"""
		# Write some data to the arduino
		self.bser.write(['float']*4, [self.kp, self.ki, self.kd, self.reference])

	def read_output(self):
		i = 0
		while (True):
			i += 1
			output = self.bser.read(['float'])
			self.plot.addData([i],[output])
			print(output)


if __name__ == '__main__':
	app = QApplication(sys.argv)
	pid_interface = PidInterface()
	pid_interface.show()
	sys.exit(app.exec_())
