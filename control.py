import math
import numpy as np
from matrix import *


class PID:
    def __init__(self, desiredValue, kp, ki, kd):
        self.desiredValue = desiredValue
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.errorPast = 0.0
        self.integral = 0.0

    def run(self, currentValue, dt):
        # print("Текущее значение-- ", currentValue)
        # print("Целевое значение---- ", self.desiredValue)
        # print("Коефициенты ---", self.kp, self.ki, self.kd, sep=', ')
        self.error = self.desiredValue - currentValue
        self.integral += self.error * dt
        value = self.kp * self.error + self.ki * self.integral + self.kd * ((self.error - self.errorPast) / dt)
        print(f"{self.kp} * {self.error} + {self.ki} * {self.integral} + {self.kd} * (({self.error} - {self.errorPast}) / {dt}) = {value}")
        self.errorPast = self.error
        return value


class ControlSystem:
    def __init__(self):
        # Переменные для Контуров управления угловой скоростью БЛА
        self.angaccelX = 0.0
        self.angaccelY = 0.0
        self.angaccelZ = 0.0
        self.angVelX = 0.0
        self.angVelY = 0.0
        self.angVelZ = 0.0
        self.avk_p_x = 100.0 
        self.avk_p_y = 100.0 
        self.avk_p_z = 1.0 
        self.avk_i_x = 0.0 
        self.avk_i_y = 0.0 
        self.avk_i_z = 0.0 
        self.avk_d_x = 0.0 
        self.avk_d_y = 0.0 
        self.avk_d_z = 0.0
        self.PIDangVelX = PID(self.angVelX, self.avk_p_x, self.avk_i_x, self.avk_d_x)
        self.PIDangVelY = PID(self.angVelY, self.avk_p_y, self.avk_i_y, self.avk_d_y)
        self.PIDangVelZ = PID(self.angVelZ, self.avk_p_z, self.avk_i_z, self.avk_d_z)

        # Переменные для Контуров управления угловым положением БЛА 
        self.angPosX = 0.0
        self.angPosY = 0.0
        self.angPosZ = 3.0
        self.apk_p_x = 100.0 
        self.apk_p_y = 100.0 
        self.apk_p_z = 10.0 
        self.apk_i_x = 0.0 
        self.apk_i_y = 0.0 
        self.apk_i_z = 0.0 
        self.apk_d_x = 30.0
        self.apk_d_y = 30.0 
        self.apk_d_z = 0.0
        self.PIDangPosX = PID(self.angPosX, self.apk_p_x, self.apk_i_x, self.apk_d_x)
        self.PIDangPosY = PID(self.angPosY, self.apk_p_y, self.apk_i_y, self.apk_d_y)
        self.PIDangPosZ = PID(self.angPosZ, self.apk_p_z, self.apk_i_z, self.apk_d_z)

        # Переменные для Контуров управления пространственным положением БЛА
        self.desPositionX = 0.0
        self.desPositionY = 0.0
        self.desPositionZ = 0.0
        self.posdesZ = 0.0
        self.pk_p_x = 0.3
        self.pk_p_y = 0.25
        self.pk_p_z = 400
        self.pk_i_x = 0.001
        self.pk_i_y = 0.001
        self.pk_i_z = 50
        self.pk_d_x = 0.47
        self.pk_d_y = 0.47
        self.pk_d_z = 300
        self.PIDPosX = PID(self.desPositionX, self.pk_p_x, self.pk_i_x, self.pk_d_x)
        self.PIDPosY = PID(self.desPositionY, self.pk_p_y, self.pk_i_y, self.pk_d_y)
        self.PIDPosZ = PID(self.desPositionZ, self.pk_p_z, self.pk_i_z, self.pk_d_z)

        self.maxAngAccel = 25.0
        self.maxAngVelosity = 1.0
        self.maxAngPosition = 0.5
        self.limitRotorVelositi = 2631

    def sed_desired_posotion(self, X, Y, Z):
        self.desPositionX = X
        self.desPositionY = Y
        self.desPositionZ = Z

    def PID_angularVelositi(self, cur_ang_vel_x, cur_ang_vel_y, cur_ang_vel_z, dt):
        print('---------------Контур угловых скоростей--------------')
        self.PIDangVelX.desiredValue = self.angVelX
        self.PIDangVelY.desiredValue = self.angVelY
        self.PIDangVelZ.desiredValue = self.angVelZ
        self.angaccelX = self.PIDangVelX.run(cur_ang_vel_x, dt)
        self.angaccelY = self.PIDangVelY.run(cur_ang_vel_y, dt)
        self.angaccelZ = self.PIDangVelZ.run(cur_ang_vel_z, dt)
        self.angaccelX = self.saturation(self.angaccelX, self.maxAngAccel)
        self.angaccelY = self.saturation(self.angaccelY, self.maxAngAccel)
        self.angaccelZ = self.saturation(self.angaccelZ, self.maxAngAccel)
        
    def PID_angularPosition(self, cur_ang_pos_x, cur_ang_pos_y, cur_ang_pos_z, dt):
        print('---------------Контур угловых положений--------------')
        self.PIDangPosX.desiredValue = self.angPosX
        self.PIDangPosY.desiredValue = self.angPosY
        self.PIDangPosZ.desiredValue = self.angPosZ
        self.angVelX = self.PIDangPosX.run(cur_ang_pos_x, dt)
        self.angVelY = self.PIDangPosY.run(cur_ang_pos_y, dt)
        self.angVelZ = self.PIDangPosZ.run(cur_ang_pos_z, dt)
        self.angVelX = self.saturation(self.angVelX, self.maxAngVelosity)
        self.angVelY = self.saturation(self.angVelY, self.maxAngVelosity)
        self.angVelZ = self.saturation(self.angVelZ, self.maxAngVelosity)

    def PID_Position(self, cur_pos_x, cur_pos_y, cur_pos_z, cur_ang_pos_z, dt):
        print('---------------Контур положений в СК--------------')
        self.PIDPosX.desiredValue = self.desPositionX
        self.PIDPosY.desiredValue = self.desPositionY
        self.PIDPosZ.desiredValue = self.desPositionZ        
        self.angPosX = self.PIDPosX.run(cur_pos_x, dt)
        self.angPosY = self.PIDPosY.run(cur_pos_y, dt)
        self.posdesZ = self.PIDPosZ.run(cur_pos_z, dt)
        self.angPosX = self.saturation(self.angPosX, self.maxAngPosition)
        self.angPosY = self.saturation(self.angPosY, self.maxAngPosition)
        
        target_pitch_roll = rotation2d(cur_ang_pos_z).T @ np.array([[self.angPosY], [self.angPosX]])
        self.angPosX = target_pitch_roll[0]
        self.angPosY = target_pitch_roll[1]
                
    def mixer(self):
        print(self.posdesZ, self.angaccelX, self.angaccelY, self.angaccelZ, sep='\t')
        m1 = self.posdesZ + self.angaccelX - self.angaccelZ
        m2 = self.posdesZ - self.angaccelY + self.angaccelZ
        m3 = self.posdesZ - self.angaccelX - self.angaccelZ
        m4 = self.posdesZ + self.angaccelY + self.angaccelZ
        m1 = self.saturation_velosity(m1, self.limitRotorVelositi)
        m2 = self.saturation_velosity(m2, self.limitRotorVelositi)
        m3 = self.saturation_velosity(m3, self.limitRotorVelositi)
        m4 = self.saturation_velosity(m4, self.limitRotorVelositi)
        result = np.array([[m1],
                           [m2],
                           [m3],
                           [m4]], dtype=float)
        print("MIXER ----- ", result)
        return result
    
    def saturation(self, inputVal, controlLimit):
        # Звено насыщения ограничивает размер входного параметра
        # На выходе метода,абсолютное значение не может быть больше
        # заданного предела controlLimit
        if inputVal > controlLimit:
            inputVal = controlLimit
        elif inputVal < -controlLimit:
            inputVal = -controlLimit

        return inputVal

    
    def saturation_velosity(self, inputVal, controlLimit):
        if inputVal > controlLimit:
            inputVal = controlLimit
        elif inputVal < 0:
            inputVal = 0

        return inputVal

# pid = PID(100, 0, 0, 0)
# res = pid.run(0, 0.01, 20)
# print(res)