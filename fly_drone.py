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

from interfaces.original import Interface, params

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
        # vehicle state: (zpos, xypos, theta)
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
                zpos, self.xypos, self.theta = state
                if self.flt_mode == self._LANDING_FM:
                    self.throttle -= 20
                else:
                    self._run_pid_controller(zpos)

        # serial comms - write to Arduino
        self.throttle = self._clamp(self.throttle, 1000, 2000)
        self.yaw = self._clamp(self.yaw, 1000, 2000)
        command = (self.throttle, self.roll, self.pitch, self.yaw)
        self.interface.sendCommand(command)

        self.flighttoc = timeit.default_timer()

        self.interface.display(
                command,
                self.flighttoc,
                self.flighttic,
                self.x_target,
                self.ypos_target)

        key = self.interface.getKeyboardInput()

        if self.flying:

            self.interface.record()

            if self.xypos is None:
                self.xypos = np.zeros(2)
                zpos = 0

            self._build_flight_data(zpos)

            if len(self.x_targ_seq) > 1:
                self.x_target = self.x_targ_seq.pop(0)
                self.ypos_target = self.ypos_targ_seq.pop(0)
                params.Z_TARGET = self.zpos_targ_seq.pop(0)
                self.theta_target = self.theta_targ_seq.pop(0)
            elif self.flt_mode == self._PROGRAM_SEQ_FM:
                self.flt_mode = self._LANDING_FM

        elif self.recording_data:
            self._save_data()
            self.recording_data = False

        if key == 27:  # exit on ESC
            return False

        elif key == 32:  # space - take a snapshot and save it
            self.interface.takeSnapshot(self.snapnum)
            self.snapnum += 1

        elif key == ord('w'):  # takeoff
            self._take_off()
            self.flt_mode = self._NORMAL_FM

        elif key == ord('e'):  # takeoff and follow flight sequence
            self._take_off()
            self._build_flight_sequence() 
            self.flt_mode = self._PROGRAM_SEQ_FM

        elif key == ord('s'):  # land
            self.flt_mode = self._LANDING_FM

        # r - reset the serial port so Arduino will bind to another CX-10
        elif key == ord('r'):
            self.interface.reset()

        # number keys 1 - 7: fly patterns
        elif key >= ord('1') and key <= ord('7'):
            self._fly_patterns(key)

        # read next state data
        return self.interface.acquiredState()

    def _run_pid_controller(self, zpos):

        # Store the old velocity error to compute its first derivative below
        self.e_dz_old = self.e_dz

        # Altitude PID control compute velocity error deriviate as difference
        # between actual altitude and altitude target
        self.e_dz = zpos - params.Z_TARGET

        self.e_iz += self.e_dz
        self.e_iz = self._clamp(self.e_iz, -params.Kzwindup, params.Kzwindup)

        e_d2z = self.e_dz - self.e_dz_old

        self.throttle = (params.THROTTLE_MID -
                         params.Kz *
                         (params.Kpz * self.e_dz +
                          params.Kiz * self.e_iz +
                          params.Kdz * e_d2z))

        e_dx_old = self.e_dx
        e_dx = self.xypos[0]-self.x_target
        self.e_ix += e_dx
        self.e_ix = self._clamp(self.e_ix, -params.Kxwindup, params.Kxwindup)
        e_d2x = e_dx - e_dx_old

        xcommand = -params.Kx * (
                self.e_dx * params.Kpx +
                params.Kix * self.e_ix +
                params.Kdx * e_d2x)

        self.e_dy_old = self.e_dy
        e_dy = self.xypos[1] - self.ypos_target
        self.e_iy += e_dy
        self.e_iy = self._clamp(self.e_iy, -params.Kywindup, params.Kywindup)
        self.e_d2y = self.e_dy - self.e_dy_old

        ycommand = (-params.Ky *
                    (e_dy * params.Kpy +
                     params.Kiy * self.e_iy +
                     params.Kdy * self.e_d2y))

        # commands are calculated in camera reference frame
        self.roll = (xcommand * np.cos(self.theta) + ycommand *
                     np.sin(self.theta) + self.ROLL_MID)
        self.pitch = (-xcommand * np.sin(self.theta) + ycommand *
                      np.cos(self.theta) + self.PITCH_MID)
        self.e_dt_old = self.e_dt
        self.e_dt = self.theta-self.theta_target
        # angle error should always be less than 180degrees (pi
        # radians)
        if (self.e_dt > np.pi):
            self.e_dt -= 2*np.pi
        elif (self.e_dt < (-np.pi)):
            self.e_dt += 2*np.pi

        self.e_it += self.e_dt
        self.e_it = self._clamp(self.e_it, -200000, 200000)
        self.e_d2t = self.e_dt - self.e_dt_old
        self.yaw = params.Kt * (
                self.e_dt * params.Kpt + params.Kit * self.e_it + params.Kdt *
                self.e_d2t) + self.YAW_MID
        if zpos > 0:
            self.roll = self._clamp(self.roll, 1000, 2000)
            self.pitch = self._clamp(self.pitch, 1000, 2000)
        else:
            self.roll = self._clamp(self.roll, 1400, 1600)
            self.pitch = self._clamp(self.pitch, 1400, 1600)
        self.no_position_cnt = 0

    def _take_off(self):

        self.throttle = params.THROTTLE_MID
        self.roll = self.ROLL_MID  # turns left
        self.pitch = self.PITCH_MID
        self.e_ix = 0
        self.e_iy = 0
        self.e_iz = 0
        self.yaw = 1500  # self.yaw, rotates the drone
        self.flying = True
        self.recording_data = True
        self.flightdata = np.zeros(23)
        self.flighttic = timeit.default_timer()
        self.flighttoc = 0
        self.flightnum += 1

        # reload(params)  # ???
        # this lists out all the variables in module params
        # and records their values.
        self.controlvarnames = [item for item in
                                dir(params) if not item.startswith('__')]
        self.controldata = [eval('params.'+item)
                            for item in self.controlvarnames]

    def _flight_sequence(
            self, seqname, xseq_list, yseq_list, zseq_list, tseq_list):
        # This function takes sequence lists and returns sequence lists.
        # Internally it uses numpy arrays.
        #
        # THe sequence lists must have some length so that the starting
        # position is known. Empty lists are not allowed.
        xseq = np.array(xseq_list)
        yseq = np.array(yseq_list)
        zseq = np.array(zseq_list)
        tseq = np.array(tseq_list)

        seqrate = 2

        if seqname == 'land':
            zpoints = int(np.abs(np.round((zseq[-1]-45)/seqrate)))
            zseq = np.concatenate((zseq, np.linspace(zseq[-1], 30, zpoints)))
            xseq = np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
            tseq = np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))

        elif seqname == 'takeoff':
            zpoints = int(np.abs(np.round((zseq[-1]-65)/seqrate)))
            zseq = np.concatenate((zseq, np.linspace(zseq[-1], 65, zpoints)))
            xseq = np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
            tseq = np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))

        elif seqname == 'box':  # goes in a 10cm box pattern
            pts = int(np.abs(np.round((75)/seqrate)))
            fwd = np.linspace(0, 75, pts)
            xseq = np.concatenate((xseq, fwd+xseq[-1]))
            xseq = np.concatenate((xseq, np.ones(pts)*xseq[-1]))
            xseq = np.concatenate((xseq, (-1*fwd)+xseq[-1]))
            xseq = np.concatenate((xseq, np.ones(pts)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(pts)*yseq[-1]))
            yseq = np.concatenate((yseq, (-1*fwd)+yseq[-1]))
            yseq = np.concatenate((yseq, np.ones(pts)*yseq[-1]))
            yseq = np.concatenate((yseq, (fwd)+yseq[-1]))
            zseq = np.concatenate((zseq, np.ones(4*pts)*zseq[-1]))
            tseq = np.concatenate((tseq, np.ones(pts)*tseq[-1]))

        elif seqname == 'up':
            zpoints = np.abs(np.round(12/seqrate))
            zseq = np.concatenate(
                    (zseq, np.linspace(zseq[-1], zseq[-1]+12, zpoints)))
            xseq = np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
            tseq = np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))

        elif seqname == 'down':
            zpoints = np.abs(np.round(12/seqrate))
            zseq = np.concatenate((zseq,
                                  np.linspace(zseq[-1], zseq[-1]-12, zpoints)))
            xseq = np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
            tseq = np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))

        elif seqname == 'left_spot':
            xpoints = int(np.abs(np.round((xseq[-1]-200)/1)))
            xseq = np.concatenate((xseq, np.linspace(xseq[-1], 200, xpoints)))
            yseq = np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
            zseq = np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
            tseq = np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))

        elif seqname == 'right_spot':
            xpoints = int(np.abs(np.round((xseq[-1]-400)/1)))
            xseq = np.concatenate((xseq, np.linspace(xseq[-1], 400, xpoints)))
            yseq = np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
            zseq = np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
            tseq = np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))

        elif seqname == 'hover':
            xpoints = 300  # 15s of hovering in one spot
            xseq = np.concatenate((xseq, np.ones(xpoints)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
            zseq = np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
            tseq = np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))

        # this code does not take care of rotating past 180 degrees
        elif seqname == 'rot90_left':
            xpoints = 150
            self.theta_endpoint = tseq[-1] + np.pi / 2
            if (self.theta_endpoint > np.pi):
                self.theta_endpoint -= 2*np.pi
            # elif (e_dt < (-np.pi)):  # XXX e_dt undefined
            #     self.theta_endpoint += 2 * np.pi
            xseq = np.concatenate((xseq, np.ones(xpoints) * xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(xpoints) * yseq[-1]))
            zseq = np.concatenate((zseq, np.ones(xpoints) * zseq[-1]))
            tseq = np.concatenate((
                tseq, np.linspace(tseq[-1], self.theta_endpoint, xpoints)))

        # this code does not take care of rotating past 180 degrees
        elif seqname == 'rot90_right':
            xpoints = 150
            self.theta_endpoint = tseq[-1] - np.pi/2
            if (self.theta_endpoint > np.pi):
                self.theta_endpoint -= 2*np.pi
            # elif (e_dt < (-np.pi)):  # XXX e_dt undefined
            #     self.theta_endpoint += 2 * np.pi
            xseq = np.concatenate((xseq, np.ones(xpoints)*xseq[-1]))
            yseq = np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
            zseq = np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
            tseq = np.concatenate((tseq, np.linspace(tseq[-1],
                                  self.theta_endpoint, xpoints)))

        return list(xseq), list(yseq), list(zseq), list(tseq)

    def _build_flight_data(self, zpos):

        self.flightdata = (
                np.vstack(
                    (self.flightdata,
                     np.array(
                         [(self.flighttoc - self.flighttic),
                          self.xypos[0], self.xypos[1], zpos,
                          self.dx, self.dy, self.dz,
                          self.e_dx, self.e_ix, self.e_d2x, self.e_dy,
                          self.e_iy, self.e_d2y, self.e_dz, self.e_iz,
                          self.e_d2z,
                          self.xspeed, self.yspeed, self.zspeed,
                          self.throttle, self.roll, self.pitch, self.yaw]))))

    def _save_data(self):

        np.save(LOG_DIR + '/' + self.timestamp + '_flt' +
                str(self.flightnum) +
                '_' + 'self.flightdata.npy', self.flightdata)
        np.save(LOG_DIR + '/' + self.timestamp + '_flt' +
                str(self.flightnum) +
                '_' + 'self.controldata.npy', self.controldata)
        with open(LOG_DIR + '/' + self.timestamp + '_flt' +
                  str(self.flightnum) + '_' + 'self.controlvarnames.npy',
                  'wb') as f:
            pickle.dump(self.controlvarnames, f)

    def _build_flight_sequence(self):

        self.x_targ_seq = [self.x_target]
        self.ypos_targ_seq = [self.ypos_target]
        self.zpos_targ_seq = [params.Z_TARGET]
        self.theta_targ_seq = [self.theta_target]

        (self.x_targ_seq, self.ypos_targ_seq,
         self.zpos_targ_seq, self.theta_targ_seq) = \
            self._flight_sequence('hover', self.x_targ_seq,
                                  self.ypos_targ_seq, self.zpos_targ_seq,
                                  self.theta_targ_seq)

        (self.x_targ_seq, self.ypos_targ_seq,
         self.zpos_targ_seq, self.theta_targ_seq) = \
            self._flight_sequence('right_spot', self.x_targ_seq,
                                  self.ypos_targ_seq, self.zpos_targ_seq,
                                  self.theta_targ_seq)

        (self.x_targ_seq, self.ypos_targ_seq,
         self.zpos_targ_seq, self.theta_targ_seq) = \
            self._flight_sequence('left_spot', self.x_targ_seq,
                                  self.ypos_targ_seq, self.zpos_targ_seq,
                                  self.theta_targ_seq)

    def _fly_patterns(self, key):

        commands = ('takeoff', 'land', 'box', 'left_spot',
                    'right_spot', 'rotate90_left', 'rotate90_right')

        command = commands[key - ord('1')]

        (self.x_targ_seq,
         self.ypos_targ_seq,
         self.zpos_targ_seq,
         self.theta_targ_seq) = (
                self._flight_sequence(command,
                                      self.x_targ_seq,
                                      self.ypos_targ_seq,
                                      self.zpos_targ_seq,
                                      self.theta_targ_seq))

    def _clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)


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
