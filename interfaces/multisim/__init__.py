'''
pc-drone using MultiSim

Additional libraries required:

    https://github.com/simondlevy/MulticopterSim/tree/master/FlightControllers/python

    https://github.com/simondlevy/kbhit

Copyright (c) 2023 Simon D. Levy

MIT License
'''

import numpy as np
from time import time

from kbhit import KBHit
from multicopter_server import MulticopterServer


class Interface(MulticopterServer):

    def __init__(self, log_dir, timestamp):

        MulticopterServer.__init__(self)

        self.kb = KBHit()

        # throttle, roll, pitch, yaw
        self.command = None

        # (zpos, xypos, theta)
        self.state = None

        self.previousUpdateTime = time()

    # MulticopterServer methods ----------------------------------------------

    def getMotors(self, t, state, _):

        # Check quit by stopping simulation
        self.previousUpdateTime = time()

        # Convert MultiSim state into PC-Drone state
        zpos = 50 -state[MulticopterServer.STATE_Z] / 10
        xypos = 0, 0
        theta = 0
        self.state = zpos, xypos, theta

        if self.command is None:
            return np.array([0, 0, 0, 0])

        cthr, crol, cpit, cyaw = self.command

        # Normalize command to [0, 1], [-1,+1], [-1,+1], [-1,+1]
        thr = (cthr - 1000) / 1000

        return np.array([thr]*4)

    # PC-Drone Interface methods ---------------------------------------------

    def acquiredState(self):

        # Quit after a lack up updates from simulator
        return time() - self.previousUpdateTime < 1.0

    def display(self, command, flighttoc, flighttic, x_target, ypos_target):
        '''
        Displays current status
        '''
        return
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
        '''
        Returns current vehicle state: (zpos, xypos, theta)
        '''
        # print(self.state)
        return self.state

    def isReady(self):

        self.start()

        return True

    def record(self):

        pass

    def sendCommand(self, command):

        self.command = command

    def reset(self):

        pass

    def close(self):

        self.kb.set_normal_term()

    def takeSnapshot(self, index):

        pass
