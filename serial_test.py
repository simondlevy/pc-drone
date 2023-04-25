#!/usr/bin/python3

'''
Serial_test.py

Sends commands to Arduino Uno via serial port to control a drone
using the nRF24L01 wireless boards.

The arrow keys control elevator and aileron (forward/reverse and left/right)
and the w,s keys control throttle, and the a,d, keys control the rudder (yaw)

Copyright (c) 2023 perrytsao, Simon D. Levy

MIT License
'''
import serial, time, kbhit

class SerialTester:

    def __init__(self):

        self.throttle = 1000
        self.aileron = 1500
        self.elevator = 1500
        self.rudder = 1500 # yaw, rotates the drone

        self.tg = 10
        self.ag = 50
        self.eg = 50
        self.rg = 50

        self.kb = kbhit.KBHit()

        self.arduino = None


    def begin(self):

        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.01)

        time.sleep(1) #give the connection a second to settle


    def step(self):

        data = self.arduino.readline()

        if data:

            #String responses from Arduino Uno are prefaced with [AU]
            print('[AU]: '+data)
            
        if self.kb.kbhit():

            key = ord(self.kb.getch())

            if key == 27: #ESC
                print('[PC]: ESC exiting')
                return

            elif key == 13: #Enter
                #select()
                print('[PC]: Enter')

            elif key == 119: #w
                self.throttle+=self.tg

            elif key == 97: #a
                self.rudder-=self.rg         

            elif key == 115: #s
                self.throttle-=self.tg

            elif key == 100: #d
                self.rudder+=self.rg

            elif key == 224: #Special keys (arrows, f keys, ins, del, etc.)
                key = ord(self.kb.getch())
                if key == 80: #Down arrow
                    self.elevator-=eg
                elif key == 72: #Up arrow
                    self.elevator+=eg
                elif key == 77: #right arroww
                    self.aileron+=ag
                elif key == 75: #left arrow
                    self.aileron-=ag               
            
            command='%i,%i,%i,%i'% (self.throttle, self.aileron, self.elevator, self.rudder)
            # string commands to the Arduino are prefaced with  [PC]           
            print('[PC]: '+command)
            self.arduino.write(command+'\n')

    def close(self):

        # close the connection
        self.arduino.close()

        # re-open the serial port which will also reset the Arduino Uno and
        # this forces the quadcopter to power off when the radio loses conection. 
        self.arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)
        self.arduino.close()
        # close it again so it can be reopened the next time it is run.  

        self.kb.set_normal_term()

def main():


    tester = SerialTester()

    try:

        tester.begin()

        while True:

            try:
                tester.step()

            except KeyboardInterrupt:

                break
            
    finally:

        tester.close()


main()
