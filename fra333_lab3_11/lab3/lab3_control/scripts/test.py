#!/usr/bin/python3
from enum import Enum
class commy(Enum):
    none = 0
    rotationccw = 1
    rotationcw = 2
    forward = 3
    backward = 4
    up = 5
    down = 6
    left = 7 # [x,y,0] x [0,0,1]
    right = 8