'''
Arduino communications

Copyright (c) 2023 Simon D. Levy

MIT License
'''

import serial
from sys import stdout

class Arduino:

    def __init__(self, portname='/dev/ttyACM0', verbose=False):

        # Support hot-plugging Arduino
        self.port = None
        self.portname = portname

    def write(self, demands):

        cmdstr = ('%i,%i,%i,%i' %
                  (demands[0], demands[1], demands[2], demands[3]))

        print(cmdstr, end=' ')

        if self.port is None:

            try:
                self.port = serial.Serial(self.portname, 115200, timeout=.001)
            except:
                pass

        else:
            self.port.write(((cmdstr + '\n').encode()))
            print('*', end=' ')

        print()

    def readline(self):

        return None if self.port is None else self.port.readline()

    def close(self):

        if self.port is not None:
            self.port.close()
