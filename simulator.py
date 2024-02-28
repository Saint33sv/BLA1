import socket
import struct
import time as t


class Simulator:
    def __init__(self, Tend, dt, mathModel, controlModel, stateVector):
        self.Tend = Tend
        self.dt = dt
        self.mathModel = mathModel
        self.controlModel = controlModel
        self.stateVector = stateVector

    def run(self):
        host = '127.0.0.1'
        port = 12346
        addr = (host, port)
        

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        time = 0
        print(self.controlModel.angPosZ)
        while (time <= self.Tend):

            self.stateVector.timeStamp = time
            data = bytearray(struct.pack("ddddddddddddd",
                                        self.stateVector.X,
                                        self.stateVector.Y,
                                        self.stateVector.Z,
                                        self.stateVector.VelX,
                                        self.stateVector.VelY,
                                        self.stateVector.VelZ,
                                        self.stateVector.Pitch,
                                        self.stateVector.Roll,
                                        self.stateVector.Yaw,
                                        self.stateVector.PitchRate,
                                        self.stateVector.RollRate,
                                        self.stateVector.YawRate,
                                        self.stateVector.timeStamp
                                        ))
            udp_socket.sendto(data, addr)
            
            print("Положение ЛА в стартовой СК",
                 self.stateVector.X,
                 self.stateVector.Y,
                 self.stateVector.Z,
                 'Скорость ЛА в стартовой СК',
                 self.stateVector.VelX,
                 self.stateVector.VelY,
                 self.stateVector.VelZ,
                 'Угловое положение ЛА',
                 self.stateVector.Pitch,
                 self.stateVector.Roll,
                 self.stateVector.Yaw,
                 'Угловая скорость ЛА',
                 self.stateVector.PitchRate,
                 self.stateVector.RollRate,
                 self.stateVector.YawRate,
                 'Метка времени симуляции',
                 self.stateVector.timeStamp, sep='\n')
            
            self.controlModel.PID_Position(self.stateVector.X, self.stateVector.Y, self.stateVector.Z, self.stateVector.Yaw, self.dt)
            self.controlModel.PID_angularPosition(self.stateVector.Pitch, self.stateVector.Roll, self.stateVector.Yaw, self.dt)
            self.controlModel.PID_angularVelositi(self.stateVector.PitchRate, self.stateVector.RollRate, self.stateVector.YawRate, self.dt)
            rotorsAngularVelositis = self.controlModel.mixer()
            self.stateVector = self.mathModel.calculateStateVector(self.stateVector, rotorsAngularVelositis)
            
            time += self.dt
            t.sleep(0.01)




        udp_socket.close()
