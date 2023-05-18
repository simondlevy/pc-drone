#!/usr/bin/python3

'''
Main drone-flying program

Copyright (c) 2023 perrystao, Simon D. Levy

MIT License
'''

import numpy as np
import pickle
import os
import timeit
from datetime import datetime

# Un-comment one of these for your project
# from interfaces.original import Interface, params
from interfaces.multisim import Interface, params
# from interfaces.mocap import Interface, params

LOG_DIR = './logs'


class DroneFlyer:

    # Flight modes
    _NORMAL_FM = 0
    _LANDING_FM = 1
    _PROGRAM_SEQ_FM = 2

    def __init__(self, interface, timestamp):
        '''
        Creates our DroneFlyer object.
        Parameters:
            interface  the state estimator / actual interface
            timestamp  time stamp for data logging
        '''

        self.interface = interface
        self.timestamp = timestamp

        self.throttle = 1000
        self.roll = 1500  # moves left/right
        self.pitch = 1500  # moves front back
        self.yaw = 1500  # self.yaw, rotates the drone

        self.xypos = (350, 250)
        self.theta = 0

        self.command = ''
        self.flying = False
        self.no_position_cnt = 0

        self.dx, self.dy, self.dz = 0, 0, 0
        self.xspeed, self.yspeed, self.zspeed = 0, 0, 0
        self.e_dz, self.e_dx, self.e_dy, self.e_dt = 0, 0, 0, 0
        self.e_iz, self.e_ix, self.e_iy, self.e_it = 0, 0, 0, 0
        self.e_d2z, self.e_d2x, self.e_d2y, self.e_d2t = 0, 0, 0, 0

        self.ROLL_MID = 1500
        self.PITCH_MID = 1500
        self.YAW_MID = 1500

        self.x_target = 300
        self.ypos_target = 200
        self.theta_target = 0  # 45.0/180.0*np.pi

        self.x_targ_seq = [self.x_target]
        self.ypos_targ_seq = [self.ypos_target]
        self.zpos_targ_seq = [params.Z_TARGET]
        self.theta_targ_seq = [self.theta_target]
        self.flighttic = timeit.default_timer()
        self.flighttoc = timeit.default_timer()
        self.flightnum = 0

        self.flt_mode = self._NORMAL_FM

        self.recording_data = False

        self.controlvarnames = None
        self.controldata = None
        self.flightdata = None

        self.snapnum = 100

    def begin(self):
        '''
        Returns True if interface devices started successfully, False otherwise
        '''
        return self.interface.isReady()

    def step(self):
        '''
        Runs one step of the interface (acquire data, send commands).
        Returns True if step was successful, False otherwise.
        '''
        # vehicle state: (z, xypos, theta)
        state = self.interface.getState()

        if self.flying:

            # state estimator failed; cut the throttle!
            if state is None:
                self.no_position_cnt += 1
                if self.no_position_cnt > 15:
                    self.throttle = 1000 
                    self.flying = False

            # state estimator working; use state to get demands
            else:
                z, dz, self.xypos, self.theta = state
                if self.flt_mode == self._LANDING_FM:
                    self.throttle -= 20
                else:
                    self._run_pid_controller(z, dz)

        # serial comms - write to Arduino
        command = (self.throttle, self.roll, self.pitch, self.yaw)
        self.interface.sendCommand(command)

        key = self.interface.getKeyboardInput()

        if key == 27:  # exit on ESC
            return False

        elif key == ord('w'):  # takeoff
            self.flying = True
            self.flt_mode = self._NORMAL_FM

        elif key == ord('s'):  # land
            self.flt_mode = self._LANDING_FM

        # read next state data
        return self.interface.acquiredState()
    
    def _run_pid_controller(self, z, dz):

        velError = (params.Z_TARGET - z) - dz

        self.throttle = 1000 * (1 + (params.Kpz * velError))


def main():

    # Create logging directory if needed
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    timestamp = '{:%Y_%m_%d_%H_%M}'.format(datetime.now())

    # Create interface
    interface = Interface(LOG_DIR, timestamp)

    # Instantiate DroneFlyer
    flyer = DroneFlyer(interface, timestamp)

    # If ready, run to error or completion or CTRL-C
    if flyer.begin():

        try:

            while flyer.step():
                pass

        except KeyboardInterrupt:
            interface.close()
            exit(0)

    interface.close()


main()
