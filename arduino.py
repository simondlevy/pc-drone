'''
Arduino communications

Copyright (c) 2023 Simon D. Levy

MIT License
'''

import serial
from sys import stdout

class Arduino:

    def __init__(self, portname='/dev/ttyACM0', verbose=False):

        self.port = None

        try:
            self.port = serial.Serial(portname, 115200, timeout=.001)

        except:
            if verbose:
                print('No device available at %s; running in development mode' % portname)
                stdout.flush()

    def write(self, command):

        cmdstr = ('%i,%i,%i,%i' %
                  (command[0], command[1], command[2], command[3]))

        if self.port is None:
            print(cmdstr)
        else:
            self.port.write(((cmdstr + '\n').encode()))

    def readline(self):

        return None if self.port is None else self.port.readline()

    def close(self):

        if self.port is not None:
            self.port.close()
