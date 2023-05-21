'''
PID control constants and other parameters

Copyright (c) 2023 perrystao, Simon D. Levy

MIT License
'''

Kpx = 0 # 1
Kix = 0 # .03
Kdx = 0 # 10
Kx = 0 # 1
Kxwindup = 0 # 200000

Kpy = Kpx
Kiy = Kix
Kdy = Kdx
Ky = Kx
Kywindup = Kxwindup

Kpz = 2
Kiz = .1
Kdz = 20
Kz = 1
Kzwindup = 10000

Kpt = 30
Kit = 1
Kdt = 10
Kt = 2

Z_TARGET = 15

# min=1000, max=2000
THROTTLE_MID = 1500
ROLL_MID = 1500
PITCH_MID = 1500
YAW_MID = 1500


