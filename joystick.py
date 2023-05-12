#!/usr/bin/python3

"""
Team Steeze

Joystick
"""

import os
import sys
import pygame
import time
from threading import Thread

from pygame.locals import JOYBUTTONDOWN, JOYAXISMOTION
from pygame.locals import MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ["SDL_VIDEODRIVER"] = "dummy"


class Joystick(object):

    def __init__(self):

        pygame.init()

        pygame.event.set_blocked((MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN))

        self.sticks = 1000, 1500, 1500, 1500
        self.done = False
        self.clicked = 0

        if pygame.joystick.get_count() == 0:
            print("No joysticks detected.")
            self.quit(1)

        self.joy = pygame.joystick.Joystick(0)

        self.joy.init()

        numaxes = self.joy.get_numaxes()

        self.axis = []

        for i in range(numaxes):
            self.axis.append(self.joy.get_axis(i))

    def get_pwm(self, index, invert=False):
        value = int(round(self.axis[index] * 500 + 1500))
        return 3000 - value if index == 3 else value

    def run(self):

        while True:

            for event in [pygame.event.wait(), ] + pygame.event.get():

                if event.type == JOYAXISMOTION:

                    self.axis[event.axis] = event.value

                    self.sticks = (
                            self.get_pwm(3, True),
                            self.get_pwm(0),
                            self.get_pwm(1),
                            self.get_pwm(2))

                elif event.type == JOYBUTTONDOWN:

                    self.clicked += 1

                    if self.clicked == 2:
                        self.quit()
                        break

    def quit(self, status=0):
        self.done = True
        pygame.quit()
        sys.exit(status)


def run_thread(program):

    count = 0

    while not program.done:
        print('%06d' % count, program.sticks)
        count += 1
        time.sleep(.0001)  # yield to main thread


if __name__ == "__main__":

    program = Joystick()

    thread = Thread(target=run_thread, args=(program,))
    thread.start()

    program.run()
