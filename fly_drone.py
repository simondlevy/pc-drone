#!/usr/bin/python3

'''
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao

Python2=>3 translation by Simon D. Levy

MIT License
'''

import cv2
import numpy as np
import pickle
import os
# import serial
import time
import timeit
from datetime import datetime
import itertools

from mockduino import MockArduino

import control_params as cp

SAVE_VIDEO_DIR = './videos'

FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.8
FONT_COLOR = (255, 255, 255)


def putText(frame, text, pos):
    cv2.putText(frame, text, pos,
                FONT_FACE, FONT_SCALE, FONT_COLOR, 2, cv2.LINE_AA)


# create maps for undistortion
def init_undistort(mtx, dist, newcameramtx):
    # cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs,
    #                              R, newCameraMatrix, size,
    #                             m1type[, map1[, map2]]) -> map1, map2
    frame_size = (640, 480)
    map1, map2 = cv2.initUndistortRectifyMap(
            mtx, dist, None, newcameramtx, frame_size, cv2.CV_32FC1)
    return map1, map2


# this is a faster undistort_crop that only does remapping. Requires call to
# init_undistort first to to create the map1 and map2
def undistort_crop(orig_img, map1, map2, roi):
    # cv2.remap(src, map1, map2,
    #           interpolation[, dst[, borderMode[, borderValue]]]) -> dst
    dst = cv2.remap(orig_img, map1, map2, cv2.INTER_LINEAR)
    x, y, w, h = roi
    crop_frame = dst[y:y+h, x:x+w]
    return crop_frame


def handle_good_keypoints(frame, keypoints):

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

    # print(max_dist_val)

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

    message = ('dist=%d  center=%d,%d theta=%d' %
               (int(max_dist_val), 
                int(keypoints[middlepoint].pt[0]),
                int(keypoints[middlepoint].pt[1]),
                int(np.degrees(theta))))

    max_blob_dist = max_dist_val
    blob_center = keypoints[middlepoint].pt
    keypoints[middlepoint].pt[1]

    return img_with_keypoints, blob_center, max_blob_dist, theta, message


def add_blobs(crop_frame, params):

    # frame = cv2.GaussianBlur(crop_frame, (3, 3), 0)
    frame = crop_frame

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of green color in HSV
    # lower_green = np.array([60,20,20])
    # upper_green = np.array([80,255,255])

    # Actually purple
    lower_green = np.array([115, 50, 10])
    upper_green = np.array([160, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=1)

    # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(frame,frame, mask= mask)
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    reversemask = 255-mask
    keypoints = detector.detect(reversemask)

    # Assume no keypoints found
    message = 'No blobs'

    if keypoints is not None:

        if len(keypoints) > 4:
            message = '%d blob(s)' % len(keypoints)
            keypoints = sorted(keypoints, key=(lambda s: s.size))
            keypoints = keypoints[0:3]

        if len(keypoints) == 4:
            img_with_keypoints, blob_center, max_blob_dist, theta, message = \
                    handle_good_keypoints(frame, keypoints)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the
        # circle corresponds to the size of blob
        else:
            message = '%d blob(s)' % len(keypoints)
            img_with_keypoints = crop_frame
            max_blob_dist = None
            blob_center = None
            theta = None

    else:
        img_with_keypoints = crop_frame
        max_blob_dist = None
        blob_center = None
        theta = None

    putText(img_with_keypoints, message, (10, 25))

    return (img_with_keypoints,
            max_blob_dist, blob_center, theta)  # , keypoint_in_orders


def init_params():

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 256

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    return params


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def openArduino():

    # return serial.Serial('/dev/ttyACM0', 115200, timeout=.001)
    return MockArduino()


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
        zpoints = np.abs(np.round((zseq[-1]-45)/seqrate))
        zseq = np.concatenate((zseq, np.linspace(zseq[-1], 30, zpoints)))
        xseq = np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
        yseq = np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
        tseq = np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))

    elif seqname == 'takeoff':
        zpoints = np.abs(np.round((zseq[-1]-65)/seqrate))
        zseq = np.concatenate((zseq, np.linspace(zseq[-1], 65, zpoints)))
        xseq = np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
        yseq = np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
        tseq = np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))

    elif seqname == 'box':  # goes in a 10cm box pattern
        pts = np.abs(np.round((75)/seqrate))
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
        xpoints = np.abs(np.round((xseq[-1]-200)/1))
        xseq = np.concatenate((xseq, np.linspace(xseq[-1], 200, xpoints)))
        yseq = np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq = np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        tseq = np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))

    elif seqname == 'right_spot':
        xpoints = np.abs(np.round((xseq[-1]-400)/1))
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
        theta_endpoint = tseq[-1] + np.pi / 2
        if (theta_endpoint > np.pi):
            theta_endpoint -= 2*np.pi
        # elif (e_dt < (-np.pi)):  # XXX e_dt undefined
        #     theta_endpoint += 2 * np.pi
        xseq = np.concatenate((xseq, np.ones(xpoints) * xseq[-1]))
        yseq = np.concatenate((yseq, np.ones(xpoints) * yseq[-1]))
        zseq = np.concatenate((zseq, np.ones(xpoints) * zseq[-1]))
        tseq = np.concatenate((
            tseq, np.linspace(tseq[-1], theta_endpoint, xpoints)))

    # this code does not take care of rotating past 180 degrees
    elif seqname == 'rot90_right':
        xpoints = 150
        theta_endpoint = tseq[-1] - np.pi/2
        if (theta_endpoint > np.pi):
            theta_endpoint -= 2*np.pi
        # elif (e_dt < (-np.pi)):  # XXX e_dt undefined
        #     theta_endpoint += 2 * np.pi
        xseq = np.concatenate((xseq, np.ones(xpoints)*xseq[-1]))
        yseq = np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq = np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        tseq = np.concatenate(
                (tseq, np.linspace(tseq[-1], theta_endpoint, xpoints)))

    return list(xseq), list(yseq), list(zseq), list(tseq)


def main():

    # load calibration data to undistort images
    calfile = np.load('camera_cal_data_2016_03_25_15_23.npz')
    newcameramtx = calfile['newcameramtx']
    roi = calfile['roi']
    mtx = calfile['mtx']
    dist = calfile['dist']
    map1, map2 = init_undistort(mtx, dist, newcameramtx)

    cv2.namedWindow('preview')

    vc = cv2.VideoCapture(0)

    fname = 'drone_track_640_480_USBFHD01M'
    width = 640
    height = 480
    fps = 30
    wait_time = 1
    vc.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    vc.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    vc.set(cv2.CAP_PROP_FPS, fps)

    if not os.path.exists(SAVE_VIDEO_DIR):
        os.makedirs(SAVE_VIDEO_DIR)

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')

    timestamp = '{:%Y_%m_%d_%H_%M}'.format(datetime.now())

    out = cv2.VideoWriter(
            SAVE_VIDEO_DIR + '/flight_data\\' + timestamp + '_video.avi',
            fourcc, 20.0, (width, height), 1)

    throttle = 1000
    aileron = 1500  # moves left/right
    elevator = 1500  # moves front back
    rudder = 1500  # yaw, rotates the drone

    zpos = 50
    xypos = (350, 250)
    theta = 0

    command = ''
    start_flying = 0
    no_position_cnt = 0

    dz = 0
    dx = 0
    dy = 0
    xspeed = 0
    yspeed = 0
    zspeed = 0
    # dz_old = 0
    # dx_old = 0
    # dy_old = 0

    e_dz = 0
    e_dx = 0
    e_dy = 0
    e_dt = 0
    e_iz = 0
    e_ix = 0
    e_iy = 0
    e_it = 0
    e_d2z = 0
    e_d2x = 0
    e_d2y = 0
    e_d2t = 0

    THROTTLE_MID = cp.THROTTLE_MID
    ELEVATOR_MID = cp.ELEVATOR_MID
    AILERON_MID = cp.AILERON_MID
    RUDDER_MID = cp.RUDDER_MID

    # speeds = ''

    x_target = 300
    ypos_target = 200
    zpos_target = 65
    theta_target = 0  # 45.0/180.0*np.pi

    x_targ_seq = [x_target]
    ypos_targ_seq = [ypos_target]
    zpos_targ_seq = [zpos_target]
    theta_targ_seq = [theta_target]

    # tic = timeit.default_timer()
    # toc = 0
    flighttic = timeit.default_timer()
    flighttoc = timeit.default_timer()
    flightnum = 0

    # Flight modes
    NORMAL_FM = 0
    LANDING_FM = 1
    PROGRAM_SEQ_FM = 2

    flt_mode = NORMAL_FM

    recording_data = 0

    controlvarnames = None
    controldata = None
    flightdata = None

    params = init_params()

    try:

        arduino = openArduino()

        time.sleep(1)  # give the connection a second to settle

        if vc.isOpened():  # try to get the first frame
            rval, frame_o = vc.read()
            # frame_undistort=undistort_crop(np.rot90(frame_o, 2))
            frame_undistort = undistort_crop(frame_o, map1, map2, roi)
            frame, zpos, xypos, theta = add_blobs(frame_undistort, params)
            # frame, zpos, xypos=add_blobs(frame_o)
        else:
            rval = False
        ii = 100
        while rval:
            # toc_old = toc
            # toc = timeit.default_timer()
            # prints out time since the last frame was read
            # print('deltaT: %0.4f  fps: %0.1f' %
            #       (toc - toc_old, 1/(toc-toc_old)))

            frame_undistort = undistort_crop(frame_o, map1, map2, roi)
            # toc2 = timeit.default_timer()
            # print('deltaT_execute_undistort: %0.4f' % (toc2 - toc))

            frame, zpos, xypos, theta = add_blobs(frame_undistort, params)

            # toc2 = timeit.default_timer()

            # print('deltaT_execute_blob_detect: %0.4f' % (toc2 - toc))

            if start_flying:
                try:
                    if flt_mode != LANDING_FM:
                        # print('Zpos: %i Xpos: %i Ypos: %i' %
                        #       (zpos, xypos[0], xypos[1]))
                        e_dz_old = e_dz
                        e_dz = zpos-zpos_target
                        e_iz += e_dz
                        e_iz = clamp(e_iz, -10000, 10000)
                        e_d2z = e_dz-e_dz_old
                        throttle = (cp.Kz * (e_dz * cp.Kpz + cp.Kiz * e_iz +
                                    cp.Kdz * e_d2z) + THROTTLE_MID)
                        e_dx_old = e_dx
                        e_dx = xypos[0]-x_target
                        e_ix += e_dx
                        e_ix = clamp(e_ix, -200000, 200000)
                        e_d2x = e_dx - e_dx_old

                        xcommand = cp.Kx * (
                                e_dx * cp.Kpx + cp.Kix * e_ix + cp.Kdx * e_d2x)

                        e_dy_old = e_dy
                        e_dy = xypos[1] - ypos_target
                        e_iy += e_dy
                        e_iy = clamp(e_iy, -200000, 200000)
                        e_d2y = e_dy-e_dy_old

                        ycommand = (cp.Ky *
                                    (e_dy * cp.Kpy + cp.Kiy * e_iy + cp.Kdy *
                                     e_d2y))

                        # commands are calculated in camera reference frame
                        aileron = (xcommand * np.cos(theta) + ycommand *
                                   np.sin(theta) + AILERON_MID)
                        elevator = (-xcommand * np.sin(theta) + ycommand *
                                    np.cos(theta) + ELEVATOR_MID)
                        e_dt_old = e_dt
                        e_dt = theta-theta_target
                        # angle error should always be less than 180degrees (pi
                        # radians)
                        if (e_dt > np.pi):
                            e_dt -= 2*np.pi
                        elif (e_dt < (-np.pi)):
                            e_dt += 2*np.pi

                        e_it += e_dt
                        e_it = clamp(e_it, -200000, 200000)
                        e_d2t = e_dt-e_dt_old
                        rudder = cp.Kt * (
                                e_dt * cp.Kpt + cp.Kit * e_it + cp.Kdt *
                                e_d2t) + RUDDER_MID
                        if zpos > 0:
                            # print('highalt')
                            aileron = clamp(aileron, 1000, 2000)
                            elevator = clamp(elevator, 1000, 2000)
                        else:
                            # print('lowalt')
                            aileron = clamp(aileron, 1400, 1600)
                            elevator = clamp(elevator, 1400, 1600)
                        no_position_cnt = 0
                    else:  # landing mode
                        throttle = throttle-20

                except Exception:
                    # print(e)
                    no_position_cnt += 1
                    # print('STOPPED. no position or error. ')
                    if no_position_cnt > 15:
                        throttle = 1000
                        start_flying = 0

            # Serial comms - write to Arduino
            throttle = clamp(throttle, 1000, 2000)
            rudder = clamp(rudder, 1000, 2000)
            command = '%i,%i,%i,%i' % (throttle, aileron, elevator, rudder)
            # print('[PC]: '+command)
            arduino.write((command+'\n').encode())

            # Serial comms - read back from Arduino
            data = arduino.readline()
            while data:
                print('[AU]: '+data.rstrip('\n'))  # strip out the new lines
                # (better to do .read() in the long run for this reason
                data = arduino.readline()

            # Monitor keyboard

            # speeds = 'dz:  %+5.2f dx:  %+5.2f  dy: %+5.2f' % (dz, dx, dy)

            # targets = ('tsz: %+5.2f tsx: %+5.2f tsy: %+5.2f' %
            #            (zspeed, xspeed, yspeed))

            # gains = ('Kpz: %+5.2f Kiz: %+5.2f Kdz: %+5.2f' %
            #          (cp.Kpz, cp.Kiz, cp.Kdz))

            # errors_z = ('e_dz: %+5.2f e_iz: %+5.2f e_d2z: %+5.2f' %
            #             (e_dz, e_iz, e_d2z))

            flighttoc = timeit.default_timer()
            putText(frame, 'Command: ' + command, (10, 50))

            putText(frame, 'Time: %5.3f' % (flighttoc - flighttic), (10, 75))

            cv2.rectangle(frame, (int(x_target)-5, int(ypos_target)-5),
                          (int(x_target)+5, int(ypos_target)+5), (255, 0, 0),
                          thickness=1, lineType=8, shift=0)

            # dst=cv2.resize(frame, (1280,960), cv2.INTER_NEAREST)
            cv2.imshow('preview', frame)
            # toc2 = timeit.default_timer()
            # print('deltaT_execute_imshow: %0.4f' % (toc2 - toc))

            key = cv2.waitKey(wait_time)
            # toc2 = timeit.default_timer()
            # print('deltaT_execute_waitkey: %0.4f' % (toc2 - toc))
            # key = ord('0')
            if start_flying:
                # start recording to video when flying
                frame_pad = cv2.copyMakeBorder(frame, 91, 0, 75, 00,
                                               cv2.BORDER_CONSTANT,
                                               value=[255, 0, 0])
                out.write(frame_pad)
                if xypos is None:
                    xypos = np.zeros(2)
                    zpos = 0
                flightdata = np.vstack((flightdata,
                                        np.array([flighttoc - flighttic,
                                                  xypos[0], xypos[1],
                                                  zpos, dx, dy, dz,
                                                  e_dx, e_ix, e_d2x, e_dy,
                                                  e_iy, e_d2y, e_dz, e_iz,
                                                  e_d2z, xspeed, yspeed,
                                                  zspeed, throttle, aileron,
                                                  elevator, rudder])))
                if len(x_targ_seq) > 1:
                    x_target = x_targ_seq.pop(0)
                    ypos_target = ypos_targ_seq.pop(0)
                    zpos_target = zpos_targ_seq.pop(0)
                    theta_target = theta_targ_seq.pop(0)
                    print('seq len %i' % len(x_targ_seq))
                elif flt_mode == PROGRAM_SEQ_FM:
                    flt_mode = LANDING_FM

            elif recording_data:
                np.save('flight_data\\' + timestamp + '_flt' + str(flightnum) +
                        '_' + 'flightdata.npy', flightdata)
                np.save('flight_data\\' + timestamp + '_flt' + str(flightnum) +
                        '_' + 'controldata.npy', controldata)
                with open('flight_data\\' + timestamp + '_flt' +
                          str(flightnum) + '_' + 'controlvarnames.npy',
                          'wb') as f:
                    pickle.dump(controlvarnames, f)
                recording_data = 0

            if key == 27:  # exit on ESC
                break
            elif key == 32:  # space - take a snapshot and save it
                cv2.imwrite(fname+str(ii)+'.jpg', frame)
                ii += 1
            elif key == 119:  # w
                throttle = THROTTLE_MID
                aileron = AILERON_MID  # turns left
                elevator = ELEVATOR_MID
                e_ix = 0
                e_iy = 0
                e_iz = 0
                rudder = 1500  # yaw, rotates the drone
                start_flying = 1
                recording_data = 1
                flightdata = np.zeros(23)
                flighttic = timeit.default_timer()
                flighttoc = 0
                flightnum += 1

                # reload(cp)  # ???
                # this lists out all the variables in module cp
                # and records their values.
                controlvarnames = [item for item in
                                   dir(cp) if not item.startswith('__')]
                controldata = [eval('cp.'+item) for item in controlvarnames]
                flt_mode = NORMAL_FM
                print('START FLYING')
            elif key == ord('e'):
                throttle = THROTTLE_MID
                aileron = AILERON_MID  # turns left
                elevator = ELEVATOR_MID
                e_ix = 0
                e_iy = 0
                e_iz = 0
                rudder = 1500  # yaw, rotates the drone
                start_flying = 1
                recording_data = 1
                flightdata = np.zeros(23)
                flighttic = timeit.default_timer()
                flighttoc = 0
                flightnum += 1

                # reload(cp)  # ???
                # this lists out all the variables in module cp
                # and records their values.
                controlvarnames = [item for item in
                                   dir(cp) if not item.startswith('__')]
                controldata = [eval('cp.'+item) for item in controlvarnames]

                x_targ_seq = [x_target]
                ypos_targ_seq = [ypos_target]
                zpos_targ_seq = [zpos_target]
                theta_targ_seq = [theta_target]

                x_targ_seq, ypos_targ_seq, zpos_targ_seq, theta_targ_seq = \
                    flight_sequence('hover', x_targ_seq, ypos_targ_seq,
                                    zpos_targ_seq, theta_targ_seq)

                x_targ_seq, ypos_targ_seq, zpos_targ_seq, theta_targ_seq = \
                    flight_sequence('right_spot', x_targ_seq, ypos_targ_seq,
                                    zpos_targ_seq, theta_targ_seq)

                x_targ_seq, ypos_targ_seq, zpos_targ_seq, theta_targ_seq = \
                    flight_sequence('left_spot', x_targ_seq, ypos_targ_seq,
                                    zpos_targ_seq, theta_targ_seq)

                flt_mode = PROGRAM_SEQ_FM

                print('START FLYING')

            elif key == 115:  # s
                # throttle=1000
                # start_flying=0
                flt_mode = LANDING_FM

            # r - reset the serial port so Arduino will bind to another CX-10
            elif key == 114:
                arduino.close()
                arduino = openArduino()

            elif key >= ord('1') and key <= ord('7'):

                commands = ('takeoff', 'land', 'box', 'left_spot',
                            'right_spot', 'rotate90_left', 'rotate90_right')

                command = commands[key - ord('1')]

                (x_targ_seq, ypos_targ_seq, zpos_targ_seq, theta_targ_seq) = (
                        flight_sequence(command, x_targ_seq,
                                        ypos_targ_seq, zpos_targ_seq,
                                        theta_targ_seq))

            # print out the time needed to execute everything except the image
            # reload
            # toc2 = timeit.default_timer()
            # print('deltaT_execute_other: %0.4f' % (toc2 - toc))

            # read next frame
            rval, frame_o = vc.read()
            # toc2 = timeit.default_timer()
            # print('deltaT_execute_nextframe: %0.4f' % (toc2 - toc))

    finally:
        # close the connection
        arduino.close()
        # re-open the serial port which will w for Arduino Uno to do a reset
        # this forces the quadcopter to power off motors.  Will need to power
        # cycle the drone to reconnect
        arduino = openArduino()
        arduino.close()
        # close it again so it can be reopened the next time it is run.
        vc.release()
        cv2.destroyWindow('preview')
        out.release()


main()
