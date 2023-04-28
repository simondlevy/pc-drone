'''
PID control constants and other parameters

Copyright (c) 2023 perrystao, Simon D. Levy

MIT License
'''

Kpx = 1
Kix = .03
Kdx = 10
Kx = 1
Kxwindup = 200000

Kpy = Kpx
Kiy = Kix
Kdy = Kdx
Ky = Kx
Kywindup = Kxwindup

Kpz = 2
Kiz = .1
Kzwindup = 10

Kpt = 30
Kit = 1
Kdt = 10
Kt = 2

# min=1000, max=2000
THROTTLE_MID = 1500 # 1350

Z_TARGET = 15
