#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import (QWidget, QPushButton, QSpinBox, QDoubleSpinBox, QVBoxLayout, QLineEdit, QCheckBox, QHBoxLayout, QFormLayout, QGroupBox, QApplication, QSizePolicy)
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
		self.mode = config['mode']
		self.anti_windup = config['anti_windup']

	def set_sample_time(self, new_sample_time):
		self.sample_time = new_sample_time

	def set_kp(self, new_kp):
		self.kp = new_kp

	def set_ki(self, new_ki):
		self.ki = new_ki

	def set_kd(self, new_kd):
		self.kd = new_kd

	def set_reference(self, new_reference):
		self.reference = new_reference

	def set_mode(self, new_mode):
		self.mode = new_mode

	def set_anti_windup(self, new_anti_windup):
		self.anti_windup = new_anti_windup

	def init_ui(self):
		self.setWindowTitle('PidInterface')

		# Parameters
		self.sample_time_spin = QSpinBox()
		self.sample_time_spin.setMinimum(0)
		self.sample_time_spin.setMaximum(1000)
		self.sample_time_spin.setValue(self.sample_time)
		self.sample_time_spin.valueChanged.connect(self.set_sample_time)

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

		self.mode_check = QCheckBox()
		self.mode_check.setChecked(self.mode)
		self.mode_check.toggled.connect(self.set_mode)

		self.anti_windup_check = QCheckBox()
		self.anti_windup_check.setChecked(self.anti_windup)
		self.anti_windup_check.toggled.connect(self.set_anti_windup)

		parameters_layout = QFormLayout()
		parameters_layout.addRow('sample_time', self.sample_time_spin)
		parameters_layout.addRow('kp', self.kp_spin)
		parameters_layout.addRow('ki', self.ki_spin)
		parameters_layout.addRow('kd', self.kd_spin)
		parameters_layout.addRow('reference', self.reference_spin)
		parameters_layout.addRow('mode', self.mode_check)
		parameters_layout.addRow('anti_windup', self.anti_windup_check)

		parameters_group = QGroupBox('Parameters')
		parameters_group.setLayout(parameters_layout)

		apply_button = QPushButton('Apply')
		apply_button.clicked.connect(self.sent_parameters)

		# Variables
		self.input_edit = QLineEdit()
		self.input_edit.setReadOnly(True)

		self.output_edit = QLineEdit()
		self.output_edit.setReadOnly(True)

		self.integral_edit = QLineEdit()
		self.integral_edit.setReadOnly(True)


		variables_layout = QFormLayout()
		variables_layout.addRow('input', self.input_edit)
		variables_layout.addRow('output', self.output_edit)
		variables_layout.addRow('integral', self.integral_edit)

		variables_group = QGroupBox('Variables')
		variables_group.setLayout(variables_layout)

		pid_layout = QVBoxLayout()
		pid_layout.addWidget(parameters_group)
		pid_layout.addWidget(apply_button)
		pid_layout.addWidget(variables_group)
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
		self.bser.write(['uint32']+['float']*4+['bool']*2, [self.sample_time, self.kp, self.ki, self.kd, self.reference, self.mode, self.anti_windup])

	def read_output(self):
		i = 0
		while (True):
			i += 1
			input, output, integral = self.bser.read(['float']*3)
			self.plot.addData([i],[input])
			self.input_edit.setText(str(input))
			self.output_edit.setText(str(output))
			self.integral_edit.setText(str(integral))


if __name__ == '__main__':
	app = QApplication(sys.argv)
	pid_interface = PidInterface()
	pid_interface.show()
	sys.exit(app.exec_())
