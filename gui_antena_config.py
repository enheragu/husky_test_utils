#!/usr/bin/env python3
# encoding: utf-8

import serial
import time
from threading import Thread

from tkinter import * 


serial_port = '/dev/ttyUSB0'
baudrate = 115200
background = "#ffffff"
but_background = "#D8E9F1"
but_active = "#3c91bc"

def printCMD(input_str):
    print_str = str(input_str)

    print_str = print_str.replace("b'","")
    print_str = print_str.replace("\\r\\n'","")

    print('\033[96;1m' + 'OUT:  ' + (print_str) + '\033[0m')

def printLog(input_str):
    print_str = str(input_str)
    if print_str == "b''":
        return

    print_str = print_str.replace("b'","")
    print_str = print_str.replace("\\r\\n'","")

    if ">OK" in str(print_str):
        print('\033[92;1m' + 'IN:  ' + (print_str) + '\033[0m')
    else:
        print('IN:  ' + str(print_str))


def readSerialPort(serial):
    while(True):
        input_str = ser.readline()
        printLog(input_str)


def addButton(root, but_text, cmd_in):
    ### Reset button
    but_frame = Frame(root, bg=background, padx=10, pady=10)  
    but_frame.grid(sticky = 'EWNS')     

    but_frame.grid_rowconfigure(0, weight=1)
    but_frame.grid_columnconfigure(0, weight=1)

    def sendReset():  
        cmd = cmd_in
        if type(cmd) != type(list()):
            cmd = [cmd]

        for cmd_item in cmd:
            printCMD(str(cmd_item))
            ser.write(bytearray(cmd_item))
        
    button = Button(but_frame, text=but_text, command=sendReset, bg = but_background, activebackground=but_active)
    button.grid(row= 0, column = 0, sticky = 'EWNS')

def addFixConfig(root):
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
        print('\033[1m' + str(command) + '\033[0m')
        ser.write(bytearray(command))
    
    entry_lat = Entry(fix_frame)
    entry_lat.grid(row = 1, column = 0, padx = 5, sticky = 'EWNS')

    entry_lon = Entry(fix_frame)
    entry_lon.grid(row = 1, column = 1, padx = 5, sticky = 'EWNS')

    entry_alt = Entry(fix_frame)
    entry_alt.grid(row = 1, column = 2, padx = 5, sticky = 'EWNS')

    fix_but = Button(fix_frame, text='FIX', command=sendFix, bg = but_background, activebackground=but_active)
    fix_but.grid(row = 1, column = 3, padx = 5, sticky = 'EWNS')

if __name__ == "__main__":
    ser = serial.Serial(serial_port, baudrate, timeout=0.5)
    read_thread = Thread(target=readSerialPort, args=[ser])
    read_thread.start()

    root = Tk()

    root.title("Serial Port GPS Antenna Interface")
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.grid_rowconfigure(1, weight=1)
    root.grid_rowconfigure(2, weight=1)
    root.grid_rowconfigure(3, weight=1)
    root.config(background=background)

    addButton(root, "RESET", b'$CFG REST\r\n')
    addButton(root, "READ INFO", b'$CFG PROINFO\r\n')
    addButton(root, "GGA msgs 1s", [b'$CFG PROINFO\r\n',
                                    b'LOG GNGGA ONTIME 1\r\n',
                                    b'saveconfig\r\n',
                                    b'$CFG QUIT\r\n'])

    addFixConfig(root)

    root.mainloop()
    ser.close()