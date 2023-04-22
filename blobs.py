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

def get_keypoints(frame, params):

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color bounds
    lower = np.array([115, 50, 10])
    upper = np.array([160, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)

    # mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=1)

    # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(frame,frame, mask= mask)
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    reversemask = 255 - mask
    return detector.detect(reversemask)
