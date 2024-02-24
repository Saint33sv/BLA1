import math
import numpy as np
from matrix import *


class ControlSystem:
    def __init__(self):
        # Переменные для Контуров управления угловой скоростью БЛА
        self.angaccelX = 0.0
        self.angaccelY = 0.0
        self.angaccelZ = 0.0
        self.angVelX = 0.0
        self.angVelY = 0.0
        self.angVelZ = 0.0
        self.angvelX_error = 0.0
        self.angvelY_error = 0.0
        self.angvelZ_error = 0.0
        self.angvelX_errorPast = 0.0
        self.angvelY_errorPast = 0.0
        self.angvelZ_errorPast = 0.0
        self.integralAngVelX = 0.0
        self.integralAngVelY = 0.0
        self.integralAngVelZ = 0.0
        self.avk_p_x = 100 
        self.avk_p_y = 100 
        self.avk_p_z = 1 
        self.avk_i_x = 0.0 
        self.avk_i_y = 0.0 
        self.avk_i_z = 0.0 
        self.avk_d_x = 0.0 
        self.avk_d_y = 0.0 
        self.avk_d_z = 0.0

        # Переменные для Контуров управления угловым положением БЛА 
        self.angPosX = 0.0
        self.angPosY = 0.0
        self.angPosZ = 0.0
        self.angposX_error = 0.0
        self.angposY_error = 0.0
        self.angposZ_error = 0.0
        self.angposX_errorPast = 0.0
        self.angposY_errorPast = 0.0
        self.angposZ_errorPast = 0.0
        self.integralAngPosX = 0.0
        self.integralAngPosY = 0.0
        self.integralAngPosZ = 0.0
        self.apk_p_x = 100 
        self.apk_p_y = 100 
        self.apk_p_z = 10 
        self.apk_i_x = 0.0 
        self.apk_i_y = 0.0 
        self.apk_i_z = 0.0 
        self.apk_d_x = 30
        self.apk_d_y = 30 
        self.apk_d_z = 0.0

        # Переменные для Контуров управления пространственным положением БЛА
        self.posdesZ = 0.0
        self.desPositionX = 0.0
        self.desPositionY = 0.0
        self.desPositionZ = 0.0
        self.positionX_error = 0.0
        self.positionY_error = 0.0
        self.positionZ_error = 0.0
        self.positionX_errorPast = 0.0
        self.positionY_errorPast = 0.0
        self.positionZ_errorPast = 0.0
        self.integralPosX = 0.0
        self.integralPosY = 0.0
        self.integralPosZ = 0.0
        self.pk_p_x = 0.3
        self.pk_p_y = 0.25
        self.pk_p_z = 400
        self.pk_i_x = 0.001
        self.pk_i_y = 0.001
        self.pk_i_z = 50 
        self.pk_d_x = 0.47
        self.pk_d_y = 0.47
        self.pk_d_z = 300

        self.maxAngAccel = 25
        self.maxAngVelosity = 1
        self.maxAngPosition = 0.5
        self.limitRotorVelositi = 2631

    def sed_desired_posotion(self, X, Y, Z):
        self.desPositionX = X
        self.desPositionY = Y
        self.desPositionZ = Z

    def PID_angularVelositi(self, cur_ang_vel_x, cur_ang_vel_y, cur_ang_vel_z, dt):
        # Вычислим функцию ошибки
        self.angvelX_error = self.angVelX - cur_ang_vel_x
        self.angvelY_error = self.angVelY - cur_ang_vel_y
        self.angvelZ_error = self.angVelZ - cur_ang_vel_z
        # Вычисляем интеграл ошибки
        self.integralAngVelX += self.angvelX_error * dt
        self.integralAngVelY += self.angvelY_error * dt
        self.integralAngVelZ += self.angvelZ_error * dt
        # Получим рассчетную управляющее угловое ускорение двигателей при помощи ПИД регулятора
        self.angaccelX = self.avk_p_x * self.angvelX_error + self.avk_i_x * self.integralAngVelX + self.avk_d_x * ((self.angvelX_error - self.angvelX_errorPast) / dt)
        self.angaccelY = self.avk_p_y * self.angvelY_error + self.avk_i_y * self.integralAngVelY + self.avk_d_y * ((self.angvelY_error - self.angvelY_errorPast) / dt)
        self.angaccelZ = self.avk_p_z * self.angvelZ_error + self.avk_i_z * self.integralAngVelZ + self.avk_d_z * ((self.angvelZ_error - self.angvelZ_errorPast) / dt)
        # Установим предыдущую ошибку для использования в дальнейших итерациях
        self.angvelX_errorPast = self.angvelX_error
        self.angvelY_errorPast = self.angvelY_error
        self.angvelZ_errorPast = self.angvelZ_error
        # Вызовем звено насыщения для ограничения максимального управляющего воздействия
        self.angaccelX = self.saturation(self.angaccelX, self.maxAngAccel)
        self.angaccelY = self.saturation(self.angaccelY, self.maxAngAccel)
        self.angaccelZ = self.saturation(self.angaccelZ, self.maxAngAccel)

    def PID_angularPosition(self, cur_ang_pos_x, cur_ang_pos_y, cur_ang_pos_z, dt):
        # Вычислим функцию ошибки
        self.angposX_error = self.angPosX - cur_ang_pos_x
        self.angposY_error = self.angPosY - cur_ang_pos_y
        self.angposZ_error = self.angPosZ - cur_ang_pos_z
        # Вычисляем интеграл ошибки
        self.integralAngPosX += self.angposX_error * dt
        self.integralAngPosY += self.angposY_error * dt
        self.integralAngPosZ += self.angposZ_error * dt
        # Получим рассчетную управляющее угловую скорость двигателей при помощи ПИД регулятора
        self.angVelX = self.apk_p_x * self.angposX_error + self.apk_i_x * self.integralAngPosX + self.apk_d_x * ((self.angposX_error - self.angposX_errorPast) / dt)
        self.angVelY = self.apk_p_y * self.angposY_error + self.apk_i_y * self.integralAngPosY + self.apk_d_y * ((self.angposY_error - self.angposY_errorPast) / dt)
        self.angVelZ = self.apk_p_z * self.angposZ_error + self.apk_i_z * self.integralAngPosZ + self.apk_d_z * ((self.angposZ_error - self.angposZ_errorPast) / dt)
        # Установим предыдущую ошибку для использования в дальнейших итерациях
        self.angposX_errorPast = self.angposX_error
        self.angposY_errorPast = self.angposY_error
        self.angposZ_errorPast = self.angposZ_error
        # Вызовем звено насыщения для ограничения максимального управляющего воздействия
        self.angVelX = self.saturation(self.angVelX, self.maxAngVelosity)
        self.angVelY = self.saturation(self.angVelY, self.maxAngVelosity)
        self.angVelZ = self.saturation(self.angVelZ, self.maxAngVelosity)   

    def PID_Position(self, cur_pos_x, cur_pos_y, cur_pos_z, cur_ang_pos_z, dt):
        # Вычислим функцию ошибки
        self.positionX_error = self.desPositionX - cur_pos_x
        self.positionY_error = self.desPositionY - cur_pos_y
        self.positionZ_error = self.desPositionZ - cur_pos_z
        # Вычисляем интеграл ошибки
        self.integralPosX += self.positionX_error * dt
        self.integralPosY += self.positionY_error * dt
        self.integralPosZ += self.positionZ_error * dt
        # Получим рассчетную управляющее угловое положение двигателей при помощи ПИД регулятора
        self.angPosX = self.pk_p_x * self.positionX_error + self.pk_i_x * self.integralPosX + self.pk_d_x * ((self.positionX_error - self.positionX_errorPast) / dt)
        self.angPosY = self.pk_p_y * self.positionY_error + self.pk_i_y * self.integralPosY + self.pk_d_y * ((self.positionY_error - self.positionY_errorPast) / dt)
        self.posdesZ = self.pk_p_z * self.positionZ_error + self.pk_i_z * self.integralPosZ + self.pk_d_z * ((self.positionZ_error - self.positionZ_errorPast) / dt)
        # Установим предыдущую ошибку для использования в дальнейших итерациях
        self.positionX_errorPast = self.positionX_error
        self.positionY_errorPast = self.positionY_error
        self.positionZ_errorPast = self.positionZ_error
        R = rotationMatrix(0, 0, math.degrees(cur_ang_pos_z))
        xy = np.array([[self.angPosX], [self.angPosY], [0]])
        res_xy = R * xy
        self.angPosX = res_xy[0]
        self.angPosY = res_xy[1]
        # Вызовем звено насыщения для ограничения максимального управляющего воздействия
        self.angPosX = self.saturation(self.angPosX, self.maxAngPosition)
        self.angPosY = self.saturation(self.angPosY, self.maxAngPosition)
        
    def mixer(self):
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