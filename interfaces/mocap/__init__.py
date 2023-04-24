'''
PC-Drone with OptiTrack motion-capture state estimator

Copyright (c) 2023 Simon D. Levy

MIT License
'''

from arduino import Arduino


class Interface:

    def __init__(self, log_dir, timestamp):
        '''
        Creates an Interface object supporting state estimation and vehicle
        commands
        '''
        self.arduino = Arduino(verbose=True)

    def acquiredState(self):
        '''
        Acquires current state, returning True on success, False on failure
        '''
        return True

    def display(self, command, flighttoc, flighttic, x_target, ypos_target):
        '''
        Displays current status
        '''
        pass

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
        return 0

    def getState(self):
        '''
        Returns current vehicle state: (zpos, xypos, theta)
        '''

        return None

    def isReady(self):
        '''
        Returns True if interface is ready, False otherwise
        '''

        return True

    def record(self):
        '''
        Records one data frame
        '''
        pass

    def sendCommand(self, command):
        '''
        Sends a command to the controller.
        Command is a tuple (throttle, roll, pitch, yaw), with each value in the
        interval [1000,2000]
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

        self.estimator.close()

    def takeSnapshot(self, index):
        '''
        Takes a snapshot of the current interface status.
        index index of current iteration
        '''
        pass
