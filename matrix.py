import math
import numpy as np
from numpy import cos, sin 

def rotation2d(theta):
        return np.array([[np.cos(theta), -np.sin(theta)], 
                         [np.sin(theta),  np.cos(theta)]])


def rotationMatrix(pitch, roll, yaw):
	return np.array([[np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.cos(roll), -np.sin(roll)],
						[np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(pitch),
						  	np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(pitch),
							np.sin(pitch)*np.cos(roll)],
						  [np.cos(yaw)*np.sin(roll)*np.cos(pitch) + np.sin(yaw)*np.sin(pitch),
							np.sin(yaw)*np.cos(pitch)*np.sin(roll) - np.cos(yaw)*np.sin(pitch),
							np.cos(pitch)*np.cos(roll)]])



m = rotationMatrix(0.0, 0.0, math.radians(90.0))
x = np.array([[1], [0], [0]])
x_res = np.matmul(m, x, out=None)
print(x_res)








