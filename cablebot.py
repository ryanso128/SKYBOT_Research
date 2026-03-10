import numpy as np
import matplotlib.pyplot as plt
import time
import math
import struct
import serial

#Check that ardunio is uncommented, motor and EE initial positions are correct

class StepperMotor():
    def __init__(self, motor_base, anchor_EE, EE_base, pulley_di, stepsPerRevolution):
        self.motor_base = motor_base
        self.EE_base = EE_base
        self.anchor_EE = anchor_EE
        self.anchor_base = anchor_EE + EE_base
        self.pul_circ = 2*math.pi*pulley_di
        self.stepsPerRevolution = stepsPerRevolution
        self.string_length = self.calculateStringLength(EE_base)
    def calculateStringLength(self, des_EE_base):
        des_anchor_base = des_EE_base +  self.anchor_EE
        string_base = des_anchor_base - self.motor_base
        des_string_length = np.linalg.norm(string_base)
        return des_string_length
    def calculateMotorSteps(self, EE_des):
        des_length = self.calculateStringLength(EE_des)
        string_disp = des_length - self.string_length
        print(string_disp)
        steps =  int(string_disp*self.stepsPerRevolution/self.pul_circ)
        self.string_length = des_length
        return steps 
    
class Cablebot():
    def __init__(self, motors, EE_pos, arduino=None):
       self.arduino = arduino
       self.motors = motors
       self.EE_pos = EE_pos
    def moveMotors(self, steps):
        self.arduino.write(struct.pack('<3h', *steps))
    def flatcircleTrajectory(self, increments, center, radius):
        circle_points = []
        for i in range(1,increments+1):
            point = np.zeros(3)
            angle =  i*360/increments
            x = center[0] + radius*np.cos(np.deg2rad(angle))
            y = center[1] + radius*np.sin(np.deg2rad(angle))
            point[0] = x
            point[1] = y
            point[2] = 0
            circle_points.append(point)
        return circle_points
    def lineTrajectory(self, increments, des_EE_pos):
        line_points = []
        for i in range(1,increments+1):
            point = np.zeros(3)
            for j in range(0,3):
                point[j] = self.EE_pos[j] + i*(des_EE_pos[j] - self.EE_pos[j])/increments
            line_points.append(point)
        return line_points
    def plotTrajectory(self, point_array):
        colors = ['r', 'b', 'g']
        fig, ax = plt.subplots()
        for motor in self.motors:
            ax.scatter(motor.motor_base[0],motor.motor_base[1], c='k')  
        for point in point_array:
            ax.scatter(point[0], point[1], c='k')
            for i in range(0,3):
                ax.plot(np.array([self.motors[i].motor_base[0], point[0]]), np.array([self.motors[i].motor_base[1], point[1]]), colors[i])  
        plt.show()

#Motor Pulley Diameters
pul_dia1 = pul_dia2 = pul_dia3 =  .05
#Steps per Revolution
stepsPerRevolution = 200
#Motor Positions XY in base frame
motor1_base = np.array([2.48, 13.11, 0])
motor2_base = np.array([21.18,22.63,0])
motor3_base = np.array([20.98, 2.55,0])
#EE Anchor Positions in EE Frame, origin at center of EE
anchor1_EE = np.array([-.087,-.05,0])
anchor2_EE = np.array([0,.1,0])
anchor3_EE = np.array([.087,-.05,0])
#EE Initial Position in base frame
EE_init_base = np.array([24.5/2,25.5/2,0])

#Define motor objects
motor1 = StepperMotor(motor1_base, anchor1_EE, EE_init_base, pul_dia1, stepsPerRevolution)
motor2 = StepperMotor(motor2_base, anchor2_EE, EE_init_base, pul_dia2, stepsPerRevolution)
motor3 = StepperMotor(motor3_base, anchor3_EE, EE_init_base, pul_dia3, stepsPerRevolution)
motors = [motor1, motor2, motor3]

#Define Cablebot
arduino = serial.Serial(port = 'COM3', baudrate = 9600, timeout=0, parity=serial.PARITY_NONE, rtscts=0)
time.sleep(2)
cablebot = Cablebot(motors, EE_init_base, arduino=arduino)

#Overall Trajectory: Move to circle start point, move in circle
circle_center = np.array([15, 13.4, 0])
path2 = cablebot.flatcircleTrajectory(50, circle_center, 4)
path1 = cablebot.lineTrajectory(50, path2[0])
full_path = path1 + path2

#Plot Trajectory
cablebot.plotTrajectory(full_path)
input("Move the End Effector? Enter to continue")

'''
instead of doing one step at a time, do all calculations first then move motorss
'''

for path_point in full_path:
    steps1 = motor1.calculateMotorSteps(path_point)
    steps2 = motor2.calculateMotorSteps(path_point)
    steps3 = motor3.calculateMotorSteps(path_point)
    steps = [steps1, steps2, steps3]
    #Handshake Protocal
    call = arduino.readline().decode().strip()
    if call == "nextset":
        cablebot.moveMotors(steps)
    else:
        print("No Communication...")
        arduino.close()
        exit()

arduino.close()