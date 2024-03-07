import math
import numpy as np
from numpy import cos, sin 


def Rx(alpha):
    R = np.matrix([[1, 0, 0],
                   [0, cos(alpha), -sin(alpha)],
                   [0, sin(alpha), cos(alpha)]], dtype=float)
    return R

def Ry(beta):
    R = np.matrix([[cos(beta), 0, sin(beta)],
                   [0, 1, 0],     
                   [-sin(beta), 0, cos(beta)]], dtype=float)
    return R

def Rz(gamma):
    R = np.matrix([[cos(gamma), -sin(gamma), 0],
                  [sin(gamma), cos(gamma), 0],
                  [0, 0, 1]], dtype=float)
    return R


def rotationMatrix(p, r, y):
    # pitch = math.radians(p)
    # roll = math.radians(r)
    # yaw = math.radians(y)

    R = Rz(y) @ Ry(p) @ Rx(r)
    return R


# m = rotationMatrix(0.0, 0.0, math.radians(90.0))
# x = np.array([[1], [0], [0]])
# x_res = np.matmul(m, x, out=None)
# print(m)








