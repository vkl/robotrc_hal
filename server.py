'''
Created on Jun 9, 2016

@author: kladov
'''

from serial import SerialException
from time import sleep

import logging

logging.basicConfig(filename="%s.log" % __name__, filemode='w')

#COMPORT = 'COM12'
COMPORT = 'COM11'
BAUDRATE = 19200

STARTMARKER = '\x81'
STOPMARKER  = '\x8F'
MAXPWM = 127
MIDPWM = 62
MINPWM = 0

serial_conn = None
data = None

Status_STOP     =     0x00 # 0b00000000
Status_FORWARD  =     0x01 # 0b00000001 
Status_BACK     =     0x02 # 0b00000010    
Status_LEFT     =     0x03 # 0b00000011   
Status_RIGHT    =     0x04 # 0b00000100     
Status_SLEFT    =     0x05 # 0b00000101     
Status_SRIGHT   =     0x06 # 0b00000110   
Status_SBLEFT   =     0x07 # 0b00000111     
Status_SBRIGHT  =     0x08 # 0b00001000 
Status_WALL     =     0x10 # 0b00010000
Status_CALIB    =     0x20 # 0b00100000
Status_AUTO     =     0x40 # 0b01000000
Status_OK       =     0x80 # 0b10000000 
ARM_CMD_NO      =     0x00 # No command
ARM_CMD_UP      =     0x01
ARM_CMD_DOWN    =     0x02
ARM_CMD_GRAB    =     0x04
ARM_CMD_RELEASE =     0x08


def bytes2sint(high, low):
    hbyte = ord(high)
    lbyte = ord(low)
    tmp = int((hbyte << 8) | lbyte)
    if tmp >= 0x8000:
        tmp -= 0x10000
    return tmp

def write_to_port(serial_conn, data):
    while True:
        if data is None:
                return
        if data.is_close:
            return
        if serial_conn.is_open:
            if serial_conn.writable():
                serial_conn.write(data.get_cmd()) 
                sleep(0.03)

def read_from_port(serial_conn, data, interface=None):
        buff = []
        i = 0
        while True:
            if data is None:
                return
            if data.is_close:
                return
            if not serial_conn.is_open:
                try:
                    serial_conn.open()
                except SerialException as e:
                    errmsg = "Error: %s The next attempt in %d seconds" % (e.message, data.timeout)
                    data.msg = errmsg
                    if interface:
                        interface.addstr(1, 3, data.msg)
                        interface.refresh()
                    sleep(data.timeout)
                    if data.timeout >= 12:
                        data.is_close = True
                        return
                    else:
                        data.timeout += 2
            else:
                data.is_connect = True
                data.msg = ""
                if serial_conn.readable():
                    tmp = serial_conn.read_until(size=19)
                    flg = False
                    for c in tmp:
                        if c == STARTMARKER:
                            i = 0
                            flg = True
                        elif c == STOPMARKER:
                            if flg:
                                i = 0
                                flg = False
                                data.status |= Status_OK;
                                x = ord(buff[1])
                                y = ord(buff[2])
                                if interface:
                                    interface.addstr(1, 60, "motor 1: %s  " % str(x))
                                    interface.addstr(2, 60, "motor 2: %s  " % str(y))
                                    interface.refresh()
                        else:
                            if flg:
                                if len(buff) <= i: 
                                    buff.append(None)
                                buff[i] = c
                                if i > 21:
                                    i = 0
                                    flg = False
                                else:
                                    i += 1

class Data(object):
    
    is_connect = False
    
    def __init__(self, timeout=1):
        self._current_cmd = '\x00'
        self._timeout = timeout
        self._msg = ""
        self._is_close = False
        self._response = ""
        self._mode = 0
        self._status = 0
        self._drvl = MIDPWM
        self._drvr = MIDPWM
        self._arm_status = ARM_CMD_NO
        self._arm = False
        self._ready = True
    
    def get_cmd(self):
        retval = "%c%c%c%c%c%c" % (STARTMARKER, self._status, self._drvl, self._drvr, self._arm_status, STOPMARKER)
        return retval
    
    def forward(self):
        if self._drvr > self._drvl:
            self._drvl = self._drvr
        if self._drvl > self._drvr:
            self._drvr = self._drvl
        if self._drvr < 87:
            self._drvr = 87
        else:
            self._drvr += 5
        if self._drvl < 87:
            self._drvl = 87
        else:
            self._drvl += 5
        if self._drvl >= MAXPWM:
            self._drvl = MAXPWM
        if self._drvr >= MAXPWM:
            self._drvr = MAXPWM    
    
    def back(self):
        if self._drvr < self._drvl:
            self._drvl = self._drvr
        if self._drvl < self._drvr:
            self._drvr = self._drvl
        if self._drvr > 42:
            self._drvr = 42
        else:
            self._drvr -= 5
        if self._drvl > 42:
            self._drvl = 42
        else:
            self._drvl -= 5
        if self._drvl <= MINPWM:
            self._drvl = MINPWM
        if self._drvr <= MINPWM:
            self._drvr = MINPWM    
    
    def left(self):
        self._drvl += 5
        self._drvr -= 5
        if self._drvl >= MAXPWM:
            self._drvl = MAXPWM
        if self._drvr <= MINPWM:
            self._drvr = MINPWM 
    
    def right(self):
        self._drvl -= 5
        self._drvr += 5
        if self._drvl <= MINPWM:
            self._drvl = MINPWM
        if self._drvr >= MAXPWM:
            self._drvr = MAXPWM 
    
    @property
    def arm(self):
        return self._arm
    
    @arm.setter
    def arm(self, value):
        self._arm = value
    
    @property
    def ready(self):
        return self._ready
    
    @ready.setter
    def ready(self, value):
        self._ready = value
    
    @property
    def drvl(self):
        return self._drvl
    
    @drvl.setter
    def drvl(self, value):
        self._drvl = value
    
    @property
    def drvr(self):
        return self._drvr
    
    @drvr.setter
    def drvr(self, value):
        self._drvr = value
    
    @property
    def status(self):
        return self._status
    
    @status.setter
    def status(self, value):
        self._status = value
    
    @property
    def mode(self):
        return self._mode
    
    @mode.setter
    def mode(self, value):
        self._mode = value
    
    @property
    def response(self):
        return self._response
    
    @response.setter
    def response(self, value):
        self._response = value
    
    @property
    def is_close(self):
        return self._is_close
    
    @is_close.setter
    def is_close(self, value):
        self._is_close = value
    
    @property
    def timeout(self):
        return self._timeout
    
    @timeout.setter
    def timeout(self, value):
        self._timeout = value
        
    @property
    def msg(self):
        return self._msg
    
    @msg.setter
    def msg(self, value):
        self._msg = value
