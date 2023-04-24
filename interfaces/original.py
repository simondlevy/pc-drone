'''
Original configuration (Arduino + OpenCV) for pc-drone

Copyright (c) 2023 perrystao, Simon D. Levy

MIT License
'''

from state.visual import StateEstimator
from arduino import Arduino


class Interface:

    def __init__(self, log_dir, timestamp):

        self.estimator = StateEstimator(log_dir, timestamp)

        self.arduino = Arduino(verbose=True)

    def acquireState(self):

        return self.estimator.acquire()

    def display(self, command, flighttoc, flighttic, x_target, ypos_target):

        return self.estimator.display(
                command, flighttoc, flighttic, x_target, ypos_target)

    def getState(self):

        return self.estimator.getState()

    def isReady(self):

        return self.estimator.isReady()

    def record(self):

        self.estimator.record()

    def sendCommand(self, command):

        self.arduino.write(command)

    def getCommandResponse(self):

        return self.arduino.readline()

    def resetComms(self):

        self.arduino.close()
        self.arduino = Arduino()

    def close(self):

        # re-open and then close the serial port which will w for Arduino Uno to do
        # a reset this forces the quadcopter to power off motors.  Will need to
        # power cycle the drone to reconnect
        self.resetComms()
        self.arduino.close()

        self.estimator.close()

    def takeSnapshot(self, index):

        self.estimator.snapshot(index)
