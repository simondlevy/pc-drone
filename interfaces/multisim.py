'''
pc-drone using MultiSim

Copyright (c) 2023 Simon D. Levy

MIT License
'''
import numpy as np

from multicopter_server import MulticopterServer


class Interface(MulticopterServer):

    def __init__(self, log_dir, timestamp):

        MulticopterServer.__init__(self)

    # MulticopterServer methods ----------------------------------------------

    def getMotors(self, t, state, stickDemands):

        return np.array([0.6, 0.6, 0.6, 0.6])

    # PC-Drone Interface methods ---------------------------------------------

    def acquireState(self):

        return True

    def display(self, command, flighttoc, flighttic, x_target, ypos_target):

        return 0

    def getState(self):

        return None

    def isReady(self):

        self.start()

        return True

    def record(self):

        pass

    def sendCommand(self, command):

        pass

    def getCommandResponse(self):

        return ''

    def reset(self):

        pass

    def close(self):

        pass

    def takeSnapshot(self, index):

        pass
