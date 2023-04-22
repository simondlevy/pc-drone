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

