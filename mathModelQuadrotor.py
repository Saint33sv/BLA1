import yaml
import numpy as np
from matrix import *
from message import *

class MatModelQuadrotor:

    def __init__(self, paramsQuadrotor):
        self.paramsQuadrotor = paramsQuadrotor
        self.angularVelocity = np.zeros(3, dtype=float)
        self.g = 9.81
        
    def calculateStateVector(self, lastStateVector, rotorsAngularVelocity):
        funcright = self.functionRight(lastStateVector, rotorsAngularVelocity)
        funcrightfirst = np.zeros(6, dtype=float)
        funcrightsecond = np.zeros(6, dtype=float)

        funcrightfirst[0] = lastStateVector.VelX        
        funcrightfirst[1] = lastStateVector.VelY
        funcrightfirst[2] = lastStateVector.VelZ
        funcrightfirst[3] = lastStateVector.PitchRate
        funcrightfirst[4] = lastStateVector.RollRate
        funcrightfirst[5] = lastStateVector.YawRate
        funcrightsecond[0] = lastStateVector.X
        funcrightsecond[1] = lastStateVector.Y
        funcrightsecond[2] = lastStateVector.Z
        funcrightsecond[3] = lastStateVector.Pitch
        funcrightsecond[4] = lastStateVector.Roll
        funcrightsecond[5] = lastStateVector.YawRate

        funcrightfirst += funcright * self.paramsQuadrotor['simulationStep']
        funcrightsecond += funcrightfirst * self.paramsQuadrotor['simulationStep']

        lastStateVector.VelX = funcrightfirst[0]  
        lastStateVector.VelY = funcrightfirst[1]
        lastStateVector.VelZ = funcrightfirst[2]
        lastStateVector.PitchRate = funcrightfirst[3]
        lastStateVector.RollRate = funcrightfirst[4] 
        lastStateVector.YawRate = funcrightfirst[5]
        lastStateVector.X = funcrightsecond[0]
        lastStateVector.Y = funcrightsecond[1]
        lastStateVector.Z = funcrightsecond[2]
        lastStateVector.Pitch = funcrightsecond[3] 
        lastStateVector.Roll = funcrightsecond[4] 
        lastStateVector.Yaw = funcrightsecond[5]

        lastStateVector.Pitch = self.reset(lastStateVector.Pitch) 
        lastStateVector.Roll = self.reset(lastStateVector.Roll) 
        lastStateVector.Yaw = self.reset(lastStateVector.Yaw)

        # lastStateVector.PitchRate = self.reset(lastStateVector.PitchRate) 
        # lastStateVector.RollRate = self.reset(lastStateVector.RollRate) 
        # lastStateVector.YawRate = self.reset(lastStateVector.YawRate)
# 


        return lastStateVector
        

    def functionRight(self, lastStateVector, rotorsAngularVelocity):
        self.angularVelocity[0] = lastStateVector.PitchRate
        self.angularVelocity[1] = lastStateVector.RollRate
        self.angularVelocity[2] = lastStateVector.YawRate

        momentsThrustRotors = np.zeros(3, dtype=float)
        normalizeVector = np.array([[0], [0], [1]], dtype=float)
        inertialTensor = np.matrix([[self.paramsQuadrotor['Ixx'], 0, 0],
                                    [0, self.paramsQuadrotor['Iyy'], 0],
                                    [0, 0, self.paramsQuadrotor['Izz']]], dtype=float)
        sumRotorsAngularVelocity = 0
        
        for i in range(self.paramsQuadrotor['numberOfRotors']):
            sumRotorsAngularVelocity += rotorsAngularVelocity[i][0]**2
        print("rotorsAngularVelocity ", rotorsAngularVelocity)
        momentsThrustRotors[0] = (self.paramsQuadrotor['lengthOfFlyerArms'] * self.paramsQuadrotor['motorThrustCoef']) * \
            (rotorsAngularVelocity[0][0]**2 - rotorsAngularVelocity[2][0]**2)
        momentsThrustRotors[1] = (self.paramsQuadrotor['lengthOfFlyerArms'] * self.paramsQuadrotor['motorThrustCoef']) * \
            (rotorsAngularVelocity[3][0]**2 - rotorsAngularVelocity[1][0]**2)
        momentsThrustRotors[2] = self.paramsQuadrotor['motorResistCoef'] * \
            (rotorsAngularVelocity[3][0]**2 + rotorsAngularVelocity[1][0]**2 - \
                rotorsAngularVelocity[0][0]**2 - rotorsAngularVelocity[2][0]**2)
        
        
        Pi = self.paramsQuadrotor['motorThrustCoef']*sumRotorsAngularVelocity
              
        acceleration = (1/self.paramsQuadrotor['mass']) * (rotationMatrix(lastStateVector.Pitch,
                                                                       lastStateVector.Roll,
                                                                       lastStateVector.Yaw) @ \
            (Pi*normalizeVector)) - self.g*normalizeVector
        
        
        angularAcceleration = inertialTensor.I * (momentsThrustRotors - np.cross(self.angularVelocity,
                                                        inertialTensor @ self.angularVelocity)).T
        

        
        result = np.zeros(6, dtype=float)
        for i in range(3):
            result[i] = acceleration[i]
            result[i+3] = angularAcceleration[i]

        return result

    def reset(self, anngularValue):
        result = anngularValue % 6.28318530718
        return result  
