#!/usr/bin/env python3
# encoding: utf-8

import serial
import time
from threading import Thread
import argparse

from tkinter import * 
from tkinter import ttk


serial_port = '/dev/ttyUSB0'
baudrate = 115200
background = "#ffffff"
but_background = "#D8E9F1"
but_active = "#3c91bc"

def printLog(str_in):
    print_str = str(str_in)

    print_str = print_str.replace("b'","")
    print_str = print_str.replace("\\r\\n'","")

    print(print_str)


def printCMD(str_in):
    printLog('\033[96;1m' + 'OUT:  ' + str(str_in) + '\033[0m')

def printSerialInput(str_in):
    print_str = str(str_in)
    if print_str == "b''":
        return

    if ">OK" in str(print_str):
        printLog('\033[92;1m' + 'IN:  ' + (print_str) + '\033[0m')
    else:
        printLog('IN:  ' + str(print_str))


def readSerialPort(serial_in):
    """
        Funtion that iteratively reads line from serial port and print it making use of defined
        funcion.
        :param serial_in: Input with serial object with serial port configured
    """
    while(True):
        input_str = serial_in.readline()
        printSerialInput(input_str)


def addButton(root, label_in, cmd_in):
    """
        Adds button widget to send command.
        :param label_in: Input of the label to be displayed in the button.
        :param cmd_in: List or single command (as strign) to be sent.
    """

    but_frame = Frame(root, bg=background, padx=10, pady=10)  
    but_frame.grid(sticky = 'EWNS')     

    but_frame.grid_rowconfigure(0, weight=1)
    but_frame.grid_columnconfigure(0, weight=1)

    def sendCMD():  
        cmd = cmd_in
        if type(cmd) != type(list()):
            cmd = [cmd]

        for cmd_item in cmd:
            printCMD(str(cmd_item))
            ser.write(bytearray(cmd_item.encode("ascii")))
        
    button = Button(but_frame, text=label_in, command=sendCMD, bg = but_background, activebackground=but_active)
    button.grid(row= 0, column = 0, sticky = 'EWNS')

def addComboButton(root, label_in, cmd_in, options_in, default_index_in = 0):
    but_frame = Frame(root, bg=background, padx=10, pady=10)  
    but_frame.grid(sticky = 'EWNS')     

    but_frame.grid_rowconfigure(0, weight=1)
    but_frame.grid_columnconfigure(0, weight=1)
    but_frame.grid_columnconfigure(1, weight=1)

    option_cb = ttk.Combobox(but_frame, values = options_in, state="readonly")
    option_cb.grid(row= 0, column = 0, padx = 5, sticky = 'EWNS')
    option_cb.current(default_index_in)

    def sendCMD():  
        option = option_cb.get()

        cmd = cmd_in
        if type(cmd) != type(list()):
            cmd = [cmd]

        for cmd_item in cmd:
            command = cmd_item
            if hasattr(command, "format"):
                command = str(command.format(option = option))
            printCMD(str(command))
            ser.write(bytearray(command))
        
        
    button = Button(but_frame, text=label_in, command=sendCMD, bg = but_background, activebackground=but_active)
    button.grid(row= 0, column = 1, padx = 5, sticky = 'EWNS')



def addFixConfig(root, default):
    ### Fix buttonreset_frame = Frame(root)  
    fix_frame = Frame(root, bg=background, padx=10, pady=10)  
    fix_frame.grid(sticky = 'EWNS') 

    fix_frame.grid_rowconfigure(0, weight=1)
    fix_frame.grid_columnconfigure(0, weight=1)
    fix_frame.grid_columnconfigure(1, weight=1)
    fix_frame.grid_columnconfigure(2, weight=1)
    fix_frame.grid_columnconfigure(3, weight=1)

    def sendFix():  
        lat = entry_lat.get()
        lon = entry_lon.get()
        alt = entry_alt.get()
        
        command = str('$CFG FIX ' + lat + ' ' + lon + ' ' + alt + '\r\n').encode("ascii")
        printCMD(str(command))
        ser.write(bytearray(command))
    
    entry_lat = Entry(fix_frame)
    entry_lat.grid(row = 1, column = 0, padx = 5, sticky = 'EWNS')

    entry_lon = Entry(fix_frame)
    entry_lon.grid(row = 1, column = 1, padx = 5, sticky = 'EWNS')

    entry_alt = Entry(fix_frame)
    entry_alt.grid(row = 1, column = 2, padx = 5, sticky = 'EWNS')

    if default is not None:
        entry_lat.insert(0, default[0])
        entry_lon.insert(0, default[1])
        entry_alt.insert(0, default[2])

    fix_but = Button(fix_frame, text='FIX', command=sendFix, bg = but_background, activebackground=but_active)
    fix_but.grid(row = 1, column = 3, padx = 5, sticky = 'EWNS')

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Graphical User Interface to handle configuration and feedback from Harxon ts100 Smart GPS Antenna through serial port.')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', type = str, help="Port in which the antenna is connected. Takes /dev/ttyUSB0 as default.")
    parser.add_argument('-b', '--baudrate', default = 115200, type = float, help="Baudrate for communicating with the device. Takes 115200 as default.")
    args = vars(parser.parse_args())

    serial_port = args["port"]
    baudrate = args["baudrate"]

    # ser = None # Use for testing GUI without antenna
    ser = serial.Serial(serial_port, baudrate, timeout=0.5)
    read_thread = Thread(target=readSerialPort, args=[ser])
    read_thread.start()

    root = Tk()

    s = ttk.Style(root)
    s.theme_use('clam')

    root.title("Serial Port GPS Antenna Interface")
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.grid_rowconfigure(1, weight=1)
    root.grid_rowconfigure(2, weight=1)
    root.grid_rowconfigure(3, weight=1)
    root.config(background=background)

    addButton(root, label_in = "Reset", cmd_in = '$CFG REST\r\n')
    addButton(root, label_in = "Read Info", cmd_in = '$CFG PROINFO\r\n')


    addComboButton(root, label_in = "GGA msgs 1s", cmd_in = ['$CFG PROINFO\r\n',
                                                        'LOG GNGGA ONTIME {option}\r\n',
                                                        'saveconfig\r\n',
                                                        '$CFG QUIT\r\n'],
                    options_in = [0.5, 1, 2], default_index_in = 1)

    addComboButton(root, label_in = "Set Baud Rate", cmd_in = '$CFG UART {option}\r\n', options_in = [9600, 115200], default_index_in = 1)

    addFixConfig(root, default = [38.27583014802165, -0.6858383729402829, 92])

    root.mainloop()
    ser.close()