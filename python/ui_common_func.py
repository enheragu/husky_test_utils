#!/usr/bin/env python3
# encoding: utf-8


def printLog(str_in) -> None:
    """
        Function to filter log to be printed in terminal. Removes a set of extra
        characters and empty messages so they are not printed.
        :param str_in: string with message to be printed.
    """
    print_str = str(str_in)

    print_str = print_str.replace("b'","")
    print_str = print_str.replace("\\r\\n'","")

    print(print_str)
    return print_str


def printCMD(str_in) -> None:
    """
        Function to format outgoing command log to be printed in terminal.
        :param str_in: string with message to be printed.
    """
    return printLog('\033[96;1m' + 'OUT:  ' + str(str_in) + '\033[0m')

def printSerialInput(str_in) -> None:
    """
        Function to format incoming messages log to be printed in terminal.
        :param str_in: string with message to be printed.
    """
    print_str = str(str_in)
    if print_str == "b''":
        return

    if ">OK" in str(print_str):
        return printLog('\033[92;1m' + 'IN:  ' + (print_str) + '\033[0m')
    else:
        return printLog('IN:  ' + str(print_str))


def readSerialPort(serial_in) -> None:
    """
        Funtion that iteratively reads line from serial port and print it making use of defined
        funcion.
        :param serial_in: Input with serial object with serial port configured
    """
    while(True):
        input_str = serial_in.readline()
        return printSerialInput(input_str)