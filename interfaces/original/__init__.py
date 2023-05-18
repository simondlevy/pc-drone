'''
Original configuration (Arduino + OpenCV) for pc-drone

Copyright (c) 2023 perrystao, Simon D. Levy

MIT License
'''

import cv2
import numpy as np
import itertools

from interfaces.original.blobs import get_keypoints, init_params
from arduino import Arduino


class Interface:

    CAL_FILE = 'interfaces/original/camera_cal_data_2016_03_25_15_23.npz'
    SNAP_FILE = 'drone_track_640_480_USBFHD01M'

    def __init__(self, log_dir, timestamp):
        '''
        Creates an Interface object supporting state estimation and vehicle
        commands
        '''
        self.params = init_params()

        # load calibration data to undistort images
        calfile = np.load(self.CAL_FILE)
        newcameramtx = calfile['newcameramtx']
        self.roi = calfile['roi']
        mtx = calfile['mtx']
        dist = calfile['dist']

        frame_size = (640, 480)

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
                mtx, dist, None, newcameramtx, frame_size, cv2.CV_32FC1)

        self.vc = cv2.VideoCapture(0)

        width = 640
        height = 480
        fps = 30
        self.wait_time = 1
        self.vc.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.vc.set(cv2.CAP_PROP_FPS, fps)

        fourcc = cv2.VideoWriter_fourcc(*'DIVX')

        self.video_out = cv2.VideoWriter(
                log_dir + '/' + timestamp + '_video.avi',
                fourcc, 20.0, (width, height), 1)

        self.frame_o = None
        self.key = None

        self.arduino = Arduino(verbose=True)

        self.message = None
        self.message_age = 0

    def acquiredState(self):
        '''
        Acquires current state, returning True on success, False on failure
        '''
        rval, self.frame_o = self.vc.read()

        return rval

    def display(self, command, flighttoc, flighttic, x_target, y_target):
        '''
        Displays current status.
        '''

        cmdstr = 'thr=%d rol=%d pit=%d yaw=%d' % (
                command[0], command[1], command[2], command[3])

        self._put_text(self.frame, cmdstr, (10, 50))

        self._put_text(self.frame, 'Time: %5.3f' %
                       (flighttoc - flighttic), (10, 75))

        cv2.rectangle(self.frame, (int(x_target)-5, int(y_target)-5),
                      (int(x_target)+5, int(y_target)+5), (255, 0, 0),
                      thickness=1, lineType=8, shift=0)

        # dst=cv2.resize(self.frame, (1280,960), cv2.INTER_NEAREST)
        cv2.imshow('Hit ESC to exit', self.frame)

    def getKeyboardInput(self):
        '''
        Returns ASCII code of key pressed by user:

        ESC      - quit
        spacebar - take snapshot
        w        - take off and hover in place
        e        - take off and follow a flight sequence
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
        return cv2.waitKey(self.wait_time)
        # dst=cv2.resize(frame, (1280,960), cv2.INTER_NEAREST)

    def getState(self):
        '''
        Returns current vehicle state: (zpos, xypos, theta)
        '''
        frame_undistort = self._undistort_crop(
                self.frame_o, self.map1, self.map2, self.roi)

        # frame = cv2.GaussianBlur(frame_undistort, (3, 3), 0)
        frame = frame_undistort

        # Assume no keypoints found
        newimg = frame_undistort
        state = None

        keypoints = get_keypoints(frame, self.params)

        if keypoints is not None:

            if len(keypoints) == 4:

                (state, newimg) = self._get_state_and_image_from_keypoints(
                        frame, keypoints)

                self.message = ('zpos=%d  xypos=%d,%d  theta=%d' %
                                (int(state[0]),
                                 int(state[1][0]),
                                 int(state[1][1]),
                                 int(np.degrees(state[2]))))

                self.message_age = 0

            else:
                self.message_age += 1

        if self.message is not None:

            # Fade out stale state messages over time
            level = (255 - self.message_age)
            if level > 128:
                self._put_text(newimg, self.message, (10, 25),
                               color=(level, level, level))

        self.frame = newimg

        return state

    def isReady(self):
        '''
        Returns True if interface is ready, False otherwise
        '''
        retval = False

        if self.vc.isOpened():  # try to get the first frame
            retval, self.frame_o = self.vc.read()

        return retval

    def record(self):
        '''
        Records one data frame
        '''
        frame_pad = cv2.copyMakeBorder(self.frame, 91, 0, 75, 00,
                                       cv2.BORDER_CONSTANT,
                                       value=[255, 0, 0])
        self.video_out.write(frame_pad)

    def sendCommand(self, command):
        '''
        Sends a command to the controller.
        Command is a four-tuple (throttle, roll, pitch, yaw).
        Each value must be in the interval [1000,2000].
        '''
        self.arduino.write(command)

    def reset(self):
        '''
        Resets the interface; e.g., restarts Arduino
        '''
        self.arduino.close()
        self.arduino = Arduino()

    def close(self):
        '''
        Closes the interface at the end of the run
        '''

        # re-open and then close the serial port which will w for Arduino Uno
        # to do a reset this forces the quadcopter to power off motors.  Will
        # need to power cycle the drone to reconnect
        self.reset()
        self.arduino.close()

        self.vc.release()
        self.video_out.release()

    def takeSnapshot(self, index):
        '''
        Takes a snapshot of the current interface status.
        index index of current iteration
        '''
        cv2.imwrite(self.SNAP_FILE+str(index)+'.jpg', self.frame)

    def _put_text(self, frame, text, pos, color=(255, 255, 255)):

        cv2.putText(frame, text, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

    def _undistort_crop(self, orig_img, map1, map2, roi):
        # cv2.remap(src, map1, map2,
        #           interpolation[, dst[, borderMode[, borderValue]]]) -> dst
        dst = cv2.remap(orig_img, map1, map2, cv2.INTER_LINEAR)
        x, y, w, h = roi
        crop_frame = dst[y:y+h, x:x+w]
        return crop_frame

    def _get_state_and_image_from_keypoints(self, frame, keypoints):

        pts = np.array([keypoints[i].pt for i in range(4)])
        # x,y=zip(*pts)

        # Calculate distances between all combinations of points
        dis_vectors = [lft - rgt
                       for lft, rgt in itertools.combinations(pts, 2)]
        dcalc = [np.linalg.norm(dis_vectors[i]) for i in range(6)]

        # find the closest point to all of them, that is the middle point
        mean_a = np.array([dcalc[i] for i in [0, 1, 2]]).sum()/4.0
        mean_b = np.array([dcalc[i] for i in [0, 3, 4]]).sum()/4.0
        mean_c = np.array([dcalc[i] for i in [1, 3, 5]]).sum()/4.0
        mean_d = np.array([dcalc[i] for i in [2, 4, 5]]).sum()/4.0
        middlepoint = np.argmin(np.array([mean_a, mean_b, mean_c, mean_d]))

        # find two furthest points, those are left and right sidepoints
        idx = np.argmax(dcalc)
        max_dist_val = np.max(dcalc)

        blob_center = keypoints[middlepoint].pt

        sidepts = ([0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3])[idx]

        # the frontpoint is the remaining one.
        frontpoint = 6 - np.array(sidepts+[middlepoint]).sum()

        # now determine which side point is the left one
        # http://stackoverflow.com/questions/1560492/
        # how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
        a = keypoints[middlepoint].pt
        b = keypoints[frontpoint].pt
        c = keypoints[sidepts[0]].pt
        if ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) < 0:
            leftpt = sidepts[0]
            rightpt = sidepts[1]
        else:
            leftpt = sidepts[1]
            rightpt = sidepts[0]

        # Calculate angle frontpoint - midpoint
        offset_line = np.array(
                keypoints[rightpt].pt)-np.array(keypoints[leftpt].pt)

        theta = -np.arctan2(offset_line[1], offset_line[0])

        img_with_midpoint = cv2.drawKeypoints(
                frame, [keypoints[middlepoint]], np.array([]), (0, 0, 255),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        img_with_midpoint_frontpoint = cv2.drawKeypoints(
                img_with_midpoint, [keypoints[frontpoint]], np.array([]),
                (255, 0, 0),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        keypoints_side = [keypoints[i] for i in [leftpt]]

        img_with_keypoints1 = cv2.drawKeypoints(
                img_with_midpoint_frontpoint, keypoints_side, np.array([]),
                (0, 255, 0),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        keypoints_side = [keypoints[i] for i in [rightpt]]

        img_with_keypoints = cv2.drawKeypoints(
                img_with_keypoints1, keypoints_side, np.array([]),
                (255, 255, 255),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return (max_dist_val, blob_center, theta), img_with_keypoints
