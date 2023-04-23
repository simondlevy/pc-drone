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

# Uncomment one of these
from state.visual import StateEstimator
# from state.simulator import StateEstimator

# Uncomment one of these
# from comms.mock import Comms
from comms.arduino import Comms

import pids

LOG_DIR = './logs'


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def flight_sequence(seqname, xseq_list, yseq_list, zseq_list, tseq_list):
    # This function takes sequence lists and returns sequence lists.
    # Internally it uses numpy arrays.
    #
    # THe sequence lists must have some length so that the starting position
    # is known. Empty lists are not allowed.
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
        tseq = np.concatenate(
                (tseq, np.linspace(tseq[-1], self.theta_endpoint, xpoints)))

    return list(xseq), list(yseq), list(zseq), list(tseq)


class FlyDrone:

    # Flight modes
    NORMAL_FM = 0
    LANDING_FM = 1
    PROGRAM_SEQ_FM = 2

    def __init__(self, timestamp):

        self.timestamp = timestamp

        self.throttle = 1000
        self.roll = 1500  # moves left/right
        self.pitch = 1500  # moves front back
        self.yaw = 1500  # self.yaw, rotates the drone

        self.zpos = 50
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
        # dz_old = 0 # dx_old = 0 # dy_old = 0

        self.THROTTLE_MID = pids.THROTTLE_MID
        self.ROLL_MID = pids.ROLL_MID
        self.PITCH_MID = pids.PITCH_MID
        self.YAW_MID = pids.YAW_MID

        self.x_target = 300
        self.ypos_target = 200
        self.zpos_target = 65
        self.theta_target = 0  # 45.0/180.0*np.pi

        # speeds = ''
        self.x_targ_seq = [self.x_target]
        self.ypos_targ_seq = [self.ypos_target]
        self.zpos_targ_seq = [self.zpos_target]
        self.theta_targ_seq = [self.theta_target]
        # tic = timeit.default_timer()
        # toc = 0
        self.flighttic = timeit.default_timer()
        self.flighttoc = timeit.default_timer()
        self.flightnum = 0


def main():

    # Create logging directory if needed
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    timestamp = '{:%Y_%m_%d_%H_%M}'.format(datetime.now())

    # Try to open comms; exit on failure
    try:
        comms = Comms()
    except Exception as e:
        print('Failed to open comms: %s' % str(e))
        exit(0)

    # Try to create state estimator; exit on failure
    try:
        state = StateEstimator(LOG_DIR, timestamp)
    except Exception as e:
        print('Failed to create state estimator: %s' % str(e))
        exit(0)

    self = FlyDrone(timestamp)

    flt_mode = self.NORMAL_FM

    recording_data = 0

    controlvarnames = None
    controldata = None
    flightdata = None

    rval = state.ready()

    ii = 100

    while rval:

        # toc_old = toc
        # toc = timeit.default_timer()
        # prints out time since the last frame was read
        # print('deltaT: %0.4f  fps: %0.1f' %
        #       (toc - toc_old, 1/(toc-toc_old)))
        # toc2 = timeit.default_timer()
        # print('deltaT_execute_undistort: %0.4f' % (toc2 - toc))

        self.xypos, self.zpos, self.theta = state.update()

        # toc2 = timeit.default_timer()

        # print('deltaT_execute_blob_detect: %0.4f' % (toc2 - toc))

        print(self.flying)

        if self.flying:

            try:

                if flt_mode != self.LANDING_FM:
                    # print('Zpos: %i Xpos: %i Ypos: %i' %
                    #       (self.zpos, self.xypos[0], self.xypos[1]))
                    e_dz_old = e_dz
                    e_dz = self.zpos-self.zpos_target
                    e_iz += e_dz
                    e_iz = clamp(e_iz, -10000, 10000)
                    e_d2z = e_dz-e_dz_old
                    self.throttle = (pids.Kz *
                                (e_dz * pids.Kpz +
                                 pids.Kiz * e_iz +
                                 pids.Kdz * e_d2z) +
                                self.THROTTLE_MID)
                    e_dx_old = e_dx
                    e_dx = self.xypos[0]-self.x_target
                    e_ix += e_dx
                    e_ix = clamp(e_ix, -200000, 200000)
                    e_d2x = e_dx - e_dx_old

                    xcommand = pids.Kx * (
                            e_dx * pids.Kpx +
                            pids.Kix * e_ix +
                            pids.Kdx * e_d2x)

                    e_dy_old = e_dy
                    e_dy = self.xypos[1] - self.ypos_target
                    e_iy += e_dy
                    e_iy = clamp(e_iy, -200000, 200000)
                    e_d2y = e_dy-e_dy_old

                    ycommand = (pids.Ky *
                                (e_dy * pids.Kpy +
                                 pids.Kiy * e_iy +
                                 pids.Kdy * e_d2y))

                    # commands are calculated in camera reference frame
                    self.roll = (xcommand * np.cos(self.theta) + ycommand *
                               np.sin(self.theta) + self.ROLL_MID)
                    self.pitch = (-xcommand * np.sin(self.theta) + ycommand *
                                np.cos(self.theta) + self.PITCH_MID)
                    e_dt_old = e_dt
                    e_dt = self.theta-self.theta_target
                    # angle error should always be less than 180degrees (pi
                    # radians)
                    if (e_dt > np.pi):
                        e_dt -= 2*np.pi
                    elif (e_dt < (-np.pi)):
                        e_dt += 2*np.pi

                    e_it += e_dt
                    e_it = clamp(e_it, -200000, 200000)
                    e_d2t = e_dt-e_dt_old
                    self.yaw = pids.Kt * (
                            e_dt * pids.Kpt + pids.Kit * e_it + pids.Kdt *
                            e_d2t) + YAW_MID
                    if self.zpos > 0:
                        # print('highalt')
                        self.roll = clamp(self.roll, 1000, 2000)
                        self.pitch = clamp(self.pitch, 1000, 2000)
                    else:
                        # print('lowalt')
                        self.roll = clamp(self.roll, 1400, 1600)
                        self.pitch = clamp(self.pitch, 1400, 1600)
                    self.no_position_cnt = 0
                else:  # landing mode
                    self.throttle = self.throttle-20

            except Exception:
                # print(e)
                self.no_position_cnt += 1
                # print('STOPPED. no position or error. ')
                if self.no_position_cnt > 15:
                    self.throttle = 1000
                    self.flying = False

        # Serial comms - write to Arduino
        self.throttle = clamp(self.throttle, 1000, 2000)
        self.yaw = clamp(self.yaw, 1000, 2000)
        command = '%i,%i,%i,%i' % (self.throttle, self.roll, self.pitch, self.yaw)
        # print('[PC]: '+command)
        comms.write((command+'\n').encode())

        # Serial comms - read back from Arduino
        data = comms.readline()
        while data:
            print('[AU]: '+data.rstrip('\n'))  # strip out the new lines
            # (better to do .read() in the long run for this reason
            data = comms.readline()

        # Monitor keyboard
        # speeds = 'dz:  %+5.2f dx:  %+5.2f  dy: %+5.2f' % (dz, dx, dy)
        # targets = ('tsz: %+5.2f tsx: %+5.2f tsy: %+5.2f' %
        #            (zspeed, xspeed, yspeed))
        # gains = ('Kpz: %+5.2f Kiz: %+5.2f Kdz: %+5.2f' %
        #          (pids.Kpz, pids.Kiz, pids.Kdz))
        # errors_z = ('e_dz: %+5.2f e_iz: %+5.2f e_d2z: %+5.2f' %
        #             (e_dz, e_iz, e_d2z))

        self.flighttoc = timeit.default_timer()

        key = state.display(
                command, self.flighttoc, self.flighttic, self.x_target, self.ypos_target)

        # toc2 = timeit.default_timer()
        # print('deltaT_execute_imshow: %0.4f' % (toc2 - toc))
        # toc2 = timeit.default_timer()
        # print('deltaT_execute_waitkey: %0.4f' % (toc2 - toc))
        # key = ord('0')

        if self.flying:

            state.record()

            if self.xypos is None:
                self.xypos = np.zeros(2)
                self.zpos = 0

            flightdata = np.vstack((flightdata,
                                    np.array([self.flighttoc - self.flighttic,
                                              self.xypos[0], self.xypos[1],
                                              self.zpos, self.dx, self.dy, self.dz,
                                              self.e_dx, self.e_ix, self.e_d2x, self.e_dy,
                                              self.e_iy, self.e_d2y, self.e_dz, self.e_iz,
                                              self.e_d2z, self.xspeed, self.yspeed,
                                              self.zspeed, self.throttle, self.roll,
                                              self.pitch, self.yaw])))
            if len(self.x_targ_seq) > 1:
                self.x_target = self.x_targ_seq.pop(0)
                self.ypos_target = self.ypos_targ_seq.pop(0)
                self.zpos_target = self.zpos_targ_seq.pop(0)
                self.theta_target = self.theta_targ_seq.pop(0)
                print('seq len %i' % len(self.x_targ_seq))
            elif flt_mode == self.PROGRAM_SEQ_FM:
                flt_mode = self.LANDING_FM

        elif recording_data:
            np.save(LOG_DIR + '/' + self.timestamp + '_flt' + str(self.flightnum) +
                    '_' + 'flightdata.npy', flightdata)
            np.save(LOG_DIR + '/' + self.timestamp + '_flt' + str(self.flightnum) +
                    '_' + 'controldata.npy', controldata)
            with open(LOG_DIR + '/' + self.timestamp + '_flt' +
                      str(self.flightnum) + '_' + 'controlvarnames.npy',
                      'wb') as f:
                pickle.dump(controlvarnames, f)
            recording_data = 0

        if key == 27:  # exit on ESC
            break
        elif key == 32:  # space - take a snapshot and save it
            state.snapshot(ii)
            ii += 1
        elif key == 119:  # w
            self.throttle = self.THROTTLE_MID
            self.roll = self.ROLL_MID  # turns left
            self.pitch = self.PITCH_MID
            e_ix = 0
            e_iy = 0
            e_iz = 0
            self.yaw = 1500  # self.yaw, rotates the drone
            self.flying = True
            recording_data = 1
            flightdata = np.zeros(23)
            self.flighttic = timeit.default_timer()
            self.flighttoc = 0
            self.flightnum += 1

            # reload(pids)  # ???
            # this lists out all the variables in module pids
            # and records their values.
            controlvarnames = [item for item in
                               dir(pids) if not item.startswith('__')]
            controldata = [eval('pids.'+item) for item in controlvarnames]
            flt_mode = self.NORMAL_FM
            print('START FLYING')
        elif key == ord('e'):
            self.throttle = self.THROTTLE_MID
            self.roll = self.ROLL_MID  # turns left
            self.pitch = self.PITCH_MID
            e_ix = 0
            e_iy = 0
            e_iz = 0
            self.yaw = 1500  # self.yaw, rotates the drone
            self.flying = True
            recording_data = 1
            flightdata = np.zeros(23)
            self.flighttic = timeit.default_timer()
            self.flighttoc = 0
            self.flightnum += 1

            # reload(pids)  # ???
            # this lists out all the variables in module pids
            # and records their values.
            controlvarnames = [item for item in
                               dir(pids) if not item.startswith('__')]
            controldata = [eval('pids.'+item) for item in controlvarnames]

            self.x_targ_seq = [self.x_target]
            self.ypos_targ_seq = [self.ypos_target]
            self.zpos_targ_seq = [self.zpos_target]
            self.theta_targ_seq = [self.theta_target]

            self.x_targ_seq, self.ypos_targ_seq, self.zpos_targ_seq, self.theta_targ_seq = \
                flight_sequence('hover', self.x_targ_seq, self.ypos_targ_seq,
                                self.zpos_targ_seq, self.theta_targ_seq)

            self.x_targ_seq, self.ypos_targ_seq, self.zpos_targ_seq, self.theta_targ_seq = \
                flight_sequence('right_spot', self.x_targ_seq, self.ypos_targ_seq,
                                self.zpos_targ_seq, self.theta_targ_seq)

            self.x_targ_seq, self.ypos_targ_seq, self.zpos_targ_seq, self.theta_targ_seq = \
                flight_sequence('left_spot', self.x_targ_seq, self.ypos_targ_seq,
                                self.zpos_targ_seq, self.theta_targ_seq)

            flt_mode = self.PROGRAM_SEQ_FM

            print('START FLYING')

        elif key == 115:  # s
            # throttle = 1000
            # flying = False
            flt_mode = self.LANDING_FM

        # r - reset the serial port so Arduino will bind to another CX-10
        elif key == 114:
            comms.close()
            comms = Comms()

        elif key >= ord('1') and key <= ord('7'):

            commands = ('takeoff', 'land', 'box', 'left_spot',
                        'right_spot', 'rotate90_left', 'rotate90_right')

            command = commands[key - ord('1')]

            (self.x_targ_seq, self.ypos_targ_seq, self.zpos_targ_seq, self.theta_targ_seq) = (
                    flight_sequence(command,
                                    self.x_targ_seq,
                                    self.ypos_targ_seq,
                                    self.zpos_targ_seq,
                                    self.theta_targ_seq))

        # print out the time needed to execute everything except the image
        # reload
        # toc2 = timeit.default_timer()
        # print('deltaT_execute_other: %0.4f' % (toc2 - toc))

        # read next state data
        rval = state.acquire()

        # toc2 = timeit.default_timer()
        # print('deltaT_execute_nextframe: %0.4f' % (toc2 - toc))

    # close the connection
    comms.close()
    # re-open the serial port which will w for Arduino Uno to do a reset
    # this forces the quadcopter to power off motors.  Will need to power
    # cycle the drone to reconnect
    comms = Comms()
    comms.close()
    # close it again so it can be reopened the next time it is run.
    state.close()


main()
