#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
from PyQt5 import (QtCore, QtWidgets, uic)
import yaml
import threading
from binserial import BinSerial
from easyplot import EasyPlot

class SerialInterface(QtWidgets.QWidget):
    def __init__(self):
        super(SerialInterface, self).__init__()
        self.init_config()
        self.init_ui()
        self.init_serial()

    def init_config(self):
        # Load configuration file
        with open('config.yml', 'r') as config_yml:
            config = yaml.safe_load(config_yml)

        # Store serial config
        self.serial = config['serial']

        # Initialise EasyPlot
        self.easyplot = EasyPlot(self)

        # Initialise the subplots
        for subplot in config['subplots']:
            self.easyplot.add_subplot(subplot['pos'], subplot['title'], subplot['min'], subplot['max'], subplot['nb_point'], subplot['polar'])

        # Initialise the plots for reading
        self.read_format = []
        self.plot_data = []
        for i, plot in enumerate(config['data']['read']):
            self.read_format.append(plot['type'])
            if i != 0:
                self.plot_data.append(self.easyplot.add_plot(plot['pos'], plot['label']))

        # Initialise for writing
        # self.write_format = []
        # for widget in config['data']['write']:
        #     self.write_format.append(widget['type'])

    def init_ui(self):
        uic.loadUi('serialinterface.ui', self)

        
        plot_layout = QtWidgets.QVBoxLayout(self.findChild(QtWidgets.QWidget, 'plot_widget'))
        plot_layout.addWidget(self.easyplot)
        plot_layout.setContentsMargins(0, 0, 0, 0)

    def init_serial(self):
        self.bser = BinSerial(self.serial['port'], self.serial['baud'])

        self.read_thread = threading.Thread(target=self.read_serial, args=(self.bser, self.read_format, self.plot_data), daemon=True)
        self.read_thread.start()

        # self.write_thread = threading.Thread(target=self.write_serial, args=(self.bser, self.write_format, self.plot_data), daemon=True)
        # self.write_thread.start()

    def read_serial(self, bser, read_format, plot_data):
        while (True):
            data = bser.read(read_format)
            for j in range(1, len(read_format)):
                plot_data[j-1][1][data[0]] = data[j]

    # def write_serial(self, bser, write_format, plot_data):
    #     """run the step response and get the measures"""
    #     # Write some data to the arduino
    #     while (True):
    #         self.bser.write(write_format, [float(input("write: "))])


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    serialinterface = SerialInterface()
    serialinterface.show()
    sys.exit(app.exec_())
