#!/usr/local/bin/python2.7
# encoding: utf-8
'''
cliclient -- shortdesc

cliclient is a description

It defines classes_and_methods

@author:     user_name

@copyright:  2016 organization_name. All rights reserved.

@license:    license

@contact:    user_email
@deffield    updated: Updated
'''

import sys

import serial
import threading
from time import sleep

from server import  (read_from_port, write_to_port, Data, COMPORT, BAUDRATE, Status_OK, Status_AUTO,
                     Status_FORWARD, Status_BACK, Status_LEFT, Status_RIGHT, Status_STOP,
                     ARM_CMD_DOWN, ARM_CMD_GRAB, ARM_CMD_NO, ARM_CMD_RELEASE, ARM_CMD_UP,
                     MINPWM, MIDPWM, MAXPWM)

import curses

serial_conn = None

__all__ = []
__version__ = 0.1
__date__ = '2016-07-04'
__updated__ = '2016-07-04'

DEBUG = 1
TESTRUN = 0
PROFILE = 0

CMD_POS_Y = 3
CMD_POS = 20

def main(stdscr, data):
    
    curses.init_pair(curses.COLOR_GREEN,
                     curses.COLOR_GREEN,
                     curses.COLOR_BLACK)
    attr = curses.color_pair(curses.COLOR_GREEN) | curses.A_BOLD
    stdscr.attron(attr)
    stdscr.box()
    stdscr.refresh()
    
    def wait_connection():
        count = 10
        stdscr.addstr(1, 3, 'Waiting for connection...', attr)
        while(True and not data.is_connect and count > 0):
            for cursor in '|/-\\':
                stdscr.addstr(1, 28, cursor, attr)
                stdscr.refresh()
                sleep(0.3)
            count -= 1
        if count <= 0:
            stdscr.addstr(1, 3, "Connection could not be established...", attr)
            stdscr.refresh()
            sleep(2)
            data.is_close = True
            return False
        stdscr.addstr(1, 3, "Connected                                 ", attr)
        stdscr.refresh()
        return True
        
    if not wait_connection():
        stdscr.erase()
        stdscr.refresh()
        return
    
    stdscr.addstr(CMD_POS_Y, 3, "Current command: ", attr)
    stdscr.addstr(CMD_POS_Y + 1, 3, "Driving    ", attr)
    stdscr.refresh()
    
    while(True):
        k = stdscr.getch()
        
        data._status |= Status_OK

        if k < 256:
            k = chr(k)

            if k == 'q':
                data._status = 0
                data.is_close = True
                break

            elif k == 'a':
                data._status |= Status_AUTO 
                stdscr.addstr(CMD_POS_Y, CMD_POS, 'Auto       ', attr),
            
            elif k == 'z':
                if data.arm:
                    data.arm = False
                    stdscr.addstr(CMD_POS_Y + 1, 3, "Driving    ", attr)
                    stdscr.refresh()
                else:
                    data.arm = True
                    stdscr.addstr(CMD_POS_Y + 1, 3, "Arm control", attr)
                    stdscr.refresh()

            elif k == ' ':
                data._status |= Status_STOP
                data._arm_status = ARM_CMD_NO
                data.drvl = MIDPWM
                data.drvr = MIDPWM 
                stdscr.addstr(CMD_POS_Y, CMD_POS, 'Stop       ')
                stdscr.refresh()
            
        else:
            
            if data.arm:
                if k == curses.KEY_UP:
                    data._arm_status = ARM_CMD_UP
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Arm UP     ', attr)
                    stdscr.refresh()
                elif k == curses.KEY_DOWN:
                    data._arm_status = ARM_CMD_DOWN
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Arm Down   ', attr)
                    stdscr.refresh()
                elif k == curses.KEY_LEFT:
                    data._arm_status = ARM_CMD_GRAB
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Arm Grab   ', attr)
                    stdscr.refresh()
                elif k == curses.KEY_RIGHT:
                    data._arm_status = ARM_CMD_RELEASE
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Arm Release', attr)
                    stdscr.refresh()
            else:
                if k == curses.KEY_UP:
                    data.forward()
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Forward    ', attr)
                    stdscr.refresh()
                elif k == curses.KEY_DOWN:
                    data.back()
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Back       ', attr)
                    stdscr.refresh()
                elif k == curses.KEY_LEFT:
                    data.left()
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Left       ', attr)
                    stdscr.refresh()
                elif k == curses.KEY_RIGHT:
                    data.right()
                    stdscr.addstr(CMD_POS_Y, CMD_POS, 'Right      ', attr)
                    stdscr.refresh()

if __name__ == "__main__":
        
    try:
        
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(1)
        curses.curs_set(0)
        try:
            curses.start_color()
        except:
            pass
        
        data = Data(5)
        
        serial_conn = serial.Serial()
        serial_conn.port = COMPORT
        serial_conn.baudrate = BAUDRATE
        serial_conn.parity = serial.PARITY_EVEN 
        
        receiver = threading.Thread(target=read_from_port, args=(serial_conn, data, stdscr))
        receiver.start() 
        
        transmitter = threading.Thread(target=write_to_port, args=(serial_conn, data))
        transmitter.start()
        
        main(stdscr, data)
    
    finally:       
        # wait for child thread
        transmitter.join()
        receiver.join()
        # Set everything back to normal
        stdscr.keypad(0)
        curses.curs_set(1)
        curses.echo()
        curses.nocbreak()
        curses.endwin()
        stdscr.erase()
        stdscr.refresh()
        sys.exit(0)
