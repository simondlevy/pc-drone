import cv2
import numpy as np
import pickle
import os
import time
import timeit
from datetime import datetime
import itertools

from state.visual.blobs import init_params, get_keypoints

def put_text(frame, text, pos):
    cv2.putText(frame, text, pos,
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)


def add_blobs(crop_frame, params):

    # frame = cv2.GaussianBlur(crop_frame, (3, 3), 0)
    frame = crop_frame

    # Assume no keypoints found
    message = 'No keypoints'
    img_with_keypoints = crop_frame
    max_blob_dist = None
    blob_center = None
    theta = None

    keypoints = get_keypoints(frame, params)

    if keypoints is not None:

        if len(keypoints) == 4:
            img_with_keypoints, blob_center, max_blob_dist, theta, message = \
                    handle_good_keypoints(frame, keypoints)

        else:
            message = '%d keypoints' % len(keypoints)

    put_text(img_with_keypoints, message, (10, 25))

    return (img_with_keypoints,
            max_blob_dist, blob_center, theta)  # , keypoint_in_orders


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


