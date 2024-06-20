#!/usr/bin/env python3
# encoding: utf-8

from struct import Struct
import serial
import time
from threading import Thread
import argparse

import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QComboBox, QLineEdit, QLabel, QHBoxLayout

from ui_common_func import printLog, printCMD, printSerialInput, readSerialPort

## Some color configuration for buttons and interface
background = "#ffffff"
but_background = "#D8E9F1"
but_active = "#3c91bc"



class SerialInterface(QWidget):
    def __init__(self, serial_handler):
        super().__init__()
        self.ser = serial_handler
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Serial Port GPS Antenna Interface")
        self.setGeometry(100, 100, 600, 300)

        layout = QVBoxLayout()

        ## Note that all CMD end with a \r\n at the end that is already added by default
        ## Add buttons to be displayed
        self.addButton(layout, label_in = "Reset", cmd_in = '$CFG REST')
        self.addButton(layout, label_in = "Read Info", cmd_in = '$CFG PROINFO')

        ## Entry with fix base position
        self.addFixConfig(layout, default = [38.27583014802165, -0.6858383729402829, 92])

        ## Selectable menus with commands
        self.addDoubleComboButton(layout, label_in = "Config msgs (1/Hz)", cmd_in = ['$CFG GNSS',
                                                            'LOG {option1} ONTIME {option2}',
                                                            'saveconfig',
                                                            '$CFG QUIT'],
                        options_in1 = ["GPGGA", "GPVTC", "GPRMC", "GPZDA", "GPGSA", "GPGSV"],
                        options_in2 = [10, 5, 2, 1, 0.5, 0.2, 0.1], default_index_in2 = 4)

        ## Button to remove all message types from antenna output
        self.addButton(layout, label_in = "Stop all messages", cmd_in = ['$CFG GNSS',
                                                            'unlogall',
                                                            'saveconfig',
                                                            '$CFG QUIT'])

        self.addComboButton(layout, label_in = "Set Baud Rate", cmd_in = '$CFG UART {option}', options_in = [9600, 115200], default_index_in = 1)

        self.setLayout(layout)
        self.show()

    def sendCMD(self, command, *args, **kwargs) -> None:  
        """
        Función para enviar un comando a través del puerto serie.
        :param command: Comando a enviar.
        :param args: Argumentos adicionales necesarios para formatear el comando.
        :param kwargs: Argumentos adicionales opcionales para el comando.
        """
        if isinstance(command, str):
            command = [command]  # Convertir a lista si es una cadena
        for cmd_item in command:
            formatted_command = cmd_item
            if hasattr(formatted_command, "format"):
                formatted_command = str(formatted_command.format(*args, **kwargs))
            printCMD(str(formatted_command))
            self.ser.write(bytearray((formatted_command + '\r\n').encode("ascii")))
            time.sleep(0.5)

    def addButton(self, layout, label_in, cmd_in) -> None:
        """
            Adds button widget to send command.
            :param root: root tk frame in which to add this new entry. 
            :param label_in: Input of the label to be displayed in the button.
            :param cmd_in: List or single command (as strign) to be sent.
        """

        button = QPushButton(label_in)
        button.clicked.connect(lambda: self.sendCMD(cmd_in))
        layout.addWidget(button)


    def addComboButton(self, layout, label_in, cmd_in, options_in, default_index_in = 0) -> None:
        """
            Adds configuration entry with combobox (selectable item with options provided)
            with a button to send the command
            :param root: root tk frame in which to add this new entry. 
            :param label_in: Input of the label to be displayed in the button.
            :param cmd_in: List or single command (as string) to be sent.
            :param options_in: Options to be displayed in the combobox item.
            :param default_index_in: Default option to be pre-selected in the combobox to be sent.

        """

        combo_layout = QHBoxLayout()
        # combo_label = QLabel(label_in)
        # combo_layout.addWidget(combo_label)

        combo = QComboBox()
        combo.addItems([str(option) for option in options_in])
        combo.setCurrentIndex(default_index_in)
        combo_layout.addWidget(combo)

        button = QPushButton(label_in)
        button.clicked.connect(lambda: self.sendCMD(cmd_in, option=combo.currentText()))
        combo_layout.addWidget(button)
        layout.addLayout(combo_layout)


    def addDoubleComboButton(self, layout, label_in, cmd_in, options_in1, options_in2, default_index_in1 = 0, default_index_in2 = 0) -> None:
        """
            Adds configuration entry with 2 combobox (selectable item with options provided)
            with a button to send the command
            :param root: root tk frame in which to add this new entry. 
            :param label_in: Input of the label to be displayed in the button.
            :param cmd_in: List or single command (as string) to be sent.
            :param options_in1: Options to be displayed in the first combobox item.
            :param options_in1: Options to be displayed in the second combobox item.
            :param default_index_in: Default option to be pre-selected in the combobox to be sent.

        """


        double_combo_layout = QHBoxLayout()
        # combo_label = QLabel(label_in)
        # double_combo_layout.addWidget(combo_label)

        combo1 = QComboBox()
        combo1.addItems(options_in1)
        combo1.setCurrentIndex(default_index_in1)
        double_combo_layout.addWidget(combo1)

        combo2 = QComboBox()
        combo2.addItems([str(option) for option in options_in2])
        combo2.setCurrentIndex(default_index_in2)
        double_combo_layout.addWidget(combo2)

        button = QPushButton(label_in)
        button.clicked.connect(lambda: self.sendCMD(cmd_in, option1=combo1.currentText(), option2=combo2.currentText()))
        double_combo_layout.addWidget(button)

        layout.addLayout(double_combo_layout)
        


    def addFixConfig(self, layout, default) -> None:
        """
            Function that encapsualtes the display of a three entry widget to send 
            position fo be fixed in the GPS base.
            :param root: root tk frame in which to add this new entry. 
            :param default: array with three default values for each coordinate (long
            lat and altitude).
        """
        fix_layout = QHBoxLayout()
        # fix_label = QLabel("Fix Base Position (lat, lon, alt):")
        # fix_layout.addWidget(fix_label)

        self.entry_lat = QLineEdit()
        self.entry_lat.setText(str(default[0]))
        fix_layout.addWidget(self.entry_lat)

        self.entry_lon = QLineEdit()
        self.entry_lon.setText(str(default[1]))
        fix_layout.addWidget(self.entry_lon)

        self.entry_alt = QLineEdit()
        self.entry_alt.setText(str(default[2]))
        fix_layout.addWidget(self.entry_alt)

        button = QPushButton('Fix config')
        command = str("$CFG FIX {lat} {lon} {alt}\r\n")
        button.clicked.connect(lambda: self.sendCMD(command, lat = self.entry_lat.text(),lon = self.entry_lon.text(),alt = self.entry_alt.text()))
        fix_layout.addWidget(button)
        layout.addLayout(fix_layout)

        
    
if __name__ == "__main__":
    ## Parse arguments and set them to be used
    parser = argparse.ArgumentParser(description='Graphical User Interface to handle configuration and feedback from Harxon ts100 Smart GPS Antenna through serial port.')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', type = str, help="Port in which the antenna is connected. Takes /dev/ttyUSB0 as default.")
    parser.add_argument('-b', '--baudrate', default = 115200, type = float, help="Baudrate for communicating with the device. Takes 115200 as default.")
    parser.add_argument('-d', '--debug', action='store_true', help="Runs GUI in debug mode with dummy serial.")
    args = vars(parser.parse_args())

    serial_port = args["port"]
    baudrate = args["baudrate"]
    debug = args["debug"]

    if debug:
        ## Dummy Serial obj to use for testing GUI without antenna
        class SerialDummy():
            def __init__(self, serial_port, baudrate, timeout) -> None:
                pass
            def readline(self) -> None:
                return "b''" # gets filtered all the time
            def write(self, smth) -> None:
                pass
            def close(self) -> None:
                pass

        ser = SerialDummy(serial_port, baudrate, timeout=0.5)
    else:
        ser = serial.Serial(serial_port, baudrate, timeout=0.5)
   
    read_thread = Thread(target=readSerialPort, args=[ser])
    read_thread.start()

    app = QApplication(sys.argv)
    ex = SerialInterface(ser)
    sys.exit(app.exec())
    