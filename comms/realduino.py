'''
Arduino communications

Copyright (c) 2023 Simon D. Levy

MIT License
'''

import serial


class Comms:

    def __init__(self):

        self.port = serial.Serial('/dev/ttyACM0', 115200, timeout=.001)

    def write(self, command):

        self.port.write(command)

    def readline(self):

        return self.port.readline()

    def close(self):

        self.port.close()
