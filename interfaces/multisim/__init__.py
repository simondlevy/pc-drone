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

        self.alt_prev = 0

    # MulticopterServer methods ----------------------------------------------

    def getMotors(self, t, ms_state, _):

        # Check quit by stopping simulation
        self.previousUpdateTime = time()

        # Convert MultiSim state into PC-Drone state
        self.state = (-ms_state[MulticopterServer.STATE_Z],   # NED => ENU
                      (0, 0), 0)

        # Wait until fly_drone script is ready
        if self.command is None:
            return np.array([0, 0, 0, 0])

        # Get command from main program
        cthr, crol, cpit, cyaw = self.command

        # Convert throttle from [1000,2000] to [0,1]
        thr = (cthr - 1000) / 1000

        # Convert rest of command from[1000,2000] to [-1,+1]
        rol = (crol - 1500) / 500
        pit = (cpit - 1500) / 500
        yaw = (cyaw - 1500) / 500

        # XXX fake up altitude PID for now
        Kp = 1.0
        Z_TARGET = 15
        alt = self.state[0]
        vel = 500 * (alt - self.alt_prev)
        self.alt_prev = alt
        velError = (Z_TARGET - alt) - vel
        thr = Kp * velError

        # Constrain throttle to [0,1)
        thr = 0 if thr < 0 else 0.999999 if thr > 1 else thr

        # Mix commands to get motor values
        m1 = thr - rol - pit + yaw
        m2 = thr + rol + pit + yaw
        m3 = thr + rol - pit - yaw
        m4 = thr - rol + pit - yaw

        return np.array([m1, m2, m3, m4])

    # PC-Drone Interface methods ---------------------------------------------

    def acquiredState(self):

        # Quit after a lack up updates from simulator
        return time() - self.previousUpdateTime < 1.0

    def display(
            self,
            command,
            flighttoc,
            flighttic,
            x_target,
            y_target,
            state,
            flying):
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
        Returns current vehicle state: (zpos, (xpos, ypos), theta)
        '''
        return self.state

    def isReady(self):

        self.start()

        return True

    def sendCommand(self, command):

        self.command = command

    def close(self):

        self.kb.set_normal_term()

    def halt(self):
        pass

    def record(self):
        pass

    def reset(self):
        pass

    def takeSnapshot(self, index):
        pass
