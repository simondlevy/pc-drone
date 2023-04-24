'''
pc-drone using MultiSim

Additional libraries required:

    https://github.com/simondlevy/MulticopterSim/tree/master/FlightControllers/python

    https://github.com/simondlevy/kbhit

Copyright (c) 2023 Simon D. Levy

MIT License
'''
import numpy as np

from kbhit import KBHit
from multicopter_server import MulticopterServer


class Interface(MulticopterServer):

    def __init__(self, log_dir, timestamp):

        MulticopterServer.__init__(self)

        self.kb = KBHit()

    # MulticopterServer methods ----------------------------------------------

    def getMotors(self, t, state, stickDemands):

        return np.array([0.6, 0.6, 0.6, 0.6])

    # PC-Drone Interface methods ---------------------------------------------

    def acquireState(self):

        return True

    def display(self, command, flighttoc, flighttic, x_target, ypos_target):
        '''
        Displays current status
        '''
        print('thr=%d rol=%d pit=%d yaw=%d' % (
                command[0], command[1], command[2], command[3]))


    def getKeyboardInput(self):
        '''
        Returns ASCII code of key pressed by user:

        ESC      - quit
        spacebar - take snapshot
        w        - take off and hover in place
        e        - ?
        s        - land
        r        - reset
        1        - take off
        2        - land
        3        - fly box pattern
        4        - fly to left spot
        5        - fly to right spot
        6        - rotate 90 left
        7        - rotate 90 right
        '''
        return ord(self.kb.getch()) if self.kb.kbhit() else 0

    def getState(self):

        return None

    def isReady(self):

        self.start()

        return True

    def record(self):

        pass

    def sendCommand(self, command):

        pass

    def reset(self):

        pass

    def close(self):

        print('done')
        self.kb.set_normal_term()

    def takeSnapshot(self, index):

        pass
