'''
PID control constants MultiSim version of PC-Drone

Copyright (c) 2023 perrystao, Simon D. Levy

MIT License
'''

Kpx = 1
Kix = .03
Kdx = 10
Kx = -1

Kpy = Kpx
Kiy = Kix
Kdy = Kdx
Ky = -1*Kx

Kpz = 2
Kiz = .1
Kdz = 20
Kz = -1

Kpt = 30
Kit = 1
Kdt = 10
Kt = 2

#############################
# Position Gains
Ksz = .1
Ksx = .01
Ksy = .01
Kst = .01

# Velocity Filter gains
#Fx = 0.5
#Fy = 0.5
#Fz = 0.5
#Ft = 0.5

#############################

THROTTLE_MID = 1350
