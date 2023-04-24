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

    def closeComms(self):

        self.arduino.close()

    def takeSnapshot(self, index):

        self.estimator.snapshot(index)
