import numpy as np
import math
import spidev
import RPi.GPIO as GPIO
import time

class inverseKinematics():
    def __init__(self,coor_x,coor_y,theta3):
        "Input the coordinate of then end-effector and the angle of its pose"
        self.coor_x = coor_x
        self.coor_y = coor_y
        self.theta3 = theta3

        "Arm properties, angle in radian, length in mm"
        self.a1 = 10
        self.a2 = 10
        self.d3 = 10

    def getRotationMatrix(self):
        "Calculate the rotation matrix from "
        theta3 = np.radians(self.theta3)
        self.r =np.array ([[np.cos(theta3), - np.sin(theta3), 0], \
                           [np.sin(theta3), np.cos(theta3), 0], [0, 0, 1]])

    def getWristCenter(self):
        "Calculate the coordinate of wristCenter, the point before the end-effector"
        a = np.array([self.coor_x,self.coor_y,0])
        b = np.dot(self.d3,self.r[:,0])
        oc = a.T - b
        self.ox = oc[0]
        self.oy = oc[1]

    def getAngles(self):
        self.getRotationMatrix()
        self.getWristCenter()

        "Calculate the cosine of theta2 using law of cosines"
        c2 = (-self.ox**2 - self.oy**2 + self.a1**2 + self.a2**2)/(2*self.a1*self.a2)
        theta2_neg = np.arctan2(-np.sqrt(1-c2**2),c2);

        "Calculate theta1 using a pair of inverse tangents"
        theta1 = np.arctan2(self.oy,self.ox) - np.arctan2(self.a2*np.sin(theta2_neg),self.a1\
                                                          +self.a2*np.cos(theta2_neg))

        self.jointAngle_UpArm = theta1
        self.jointAngle_LowArm = theta2_neg

        return np.array([self.jointAngle_UpArm,self.jointAngle_LowArm])

    def getLengthOfArm(self):
        self.getAngles()

        upperTril1 = 71.59
        lowerTril1 = 65
        lowerTril2 = 289.41
        l1 = upperTril1

        "Calculate the upper triangle"
        alpha = self.jointAngle_UpArm + math.pi/2

        omega = max(np.arccos((228.97**2+71.59**2-292**2)/(2*228.97*71.59)),math.pi/2)

        "Calculate the lower triangle"
        l2 = np.sqrt(lowerTril2**2+lowerTril1**2)
        beta = np.arctan2(lowerTril1,lowerTril2)

        psi = 2*math.pi - alpha - beta - omega
        arm_l3 = np.sqrt(l1**2+l2**2-2*l1*l2*np.cos(psi))

        return arm_l3

    def getLengthOfElbow(self):
        self.getAngles()

        upperTril1 = 60.18
        lowerTril1 = 60.37
        lowerTril2 = 228.38
        leftTril1 = 16.4
        l1 = upperTril1
        sideTril1 = 34.5

        "Calculate the upper triangle"
        alpha = self.jointAngle_UpArm + math.pi

        omega = max(np.arccos((142.87**2+60.18**2-193**2)/(2*142.87*60.18)),math.pi/2)

        "Calculate the left lower triangle"
        gamma = np.arctan2(leftTril1,lowerTril2)

        "Calculate the lower triangle"
        l2 = np.sqrt(lowerTril2**2+lowerTril1**2)
        beta = np.arctan2(lowerTril1,lowerTril2)

        psi = 2*math.pi - alpha - beta - omega - gamma
        elbow_l3 = np.sqrt(l1**2+l2**2-2*l1*l2*np.cos(psi))

        "Calculate the side triangle"
        l4 = np.sqrt(elbow_l3**2-sideTril1**2)
        return l4

    def getPulse(self,dist):
        "get the rounds of motor"
        rounds = dist/4
        pulse = rounds*6400
        return pulse

if __name__=='__main__':

    preArm = 319.41
    preElbow = 257.86
    preZ = 0
    arm_arr = np.array([1])
    elbow_arr = np.array([1])
    base_arr = np.array([1])
    i = 0

    # Communication with spidev
    spi = spidev.SpiDev()
    spi.open(0,0)

    # Init GPIO for waiting buffer
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(29, GPIO.IN)
    GPIO.setup(31, GPIO.IN)
    GPIO.setup(33, GPIO.IN)
    GPIO.setup(35, GPIO.IN)

    trigger = GPIO.input(29)
    sensor_base = GPIO.input(31)
    sensor_arm = GPIO.input(33)
    sensor_elbow = GPIO.input(35)
    sensor_list = [sensor_base, sensor_arm, sensor_elbow]

    # Find origin
    while sum(sensor_list) != 0:
        if trigger == 0:
            tmp = np.multiply(sensor_list,[10,10,10,0,0])
            tmp = tmp.tolist()
            spi.xfer(tmp)
            trigger = GPIO.input(29)
            sensor_base = GPIO.input(31)
            sensor_arm = GPIO.input(33)
            sensor_elbow = GPIO.input(35)
            sensor_list = [sensor_base, sensor_arm, sensor_elbow]
        elif trigger == 1:
        
            trigger = GPIO.input(29)


    print("please input the coor_x:")
    a = float(input())
    print("please input the coor_y:")
    b = float(input())
    print("please input the coor_z:")
    z = float(input())
    print("please input the theta3:")
    c = float(input())
    inv = inverseKinematics(a,b,c)

    "get the length of arm and elbow"
    arm = inv.getLengthOfArm()
    elbow = inv.getLengthOfElbow()
    base = z

    dist_arm = arm - preArm
    dist_elbow = elbow - preElbow
    dist_z = z - preZ

    pulse_arm = inv.getPulse(dist_arm)
    pulse_elbow = inv.getPulse(dist_elbow)
    pulse_base = inv.getPulse(dist_z)

    '''
    while i <100:
        print("please input the coor_x:")
        a = float(input())
        print("please input the coor_y:")
        b = float(input())
        print("please input the coor_z:")
        z = float(input())
        print("please input the theta3:")
        c = float(input())
        inv = inverseKinematics(a,b,c)

        "get the length of arm and elbow"
        arm = inv.getLengthOfArm()
        elbow = inv.getLengthOfElbow()
        base = z

        dist_arm = arm - preArm
        dist_elbow = elbow - preElbow
        dist_z = z - preZ

        pulse_arm = inv.getPulse(dist_arm)
        pulse_elbow = inv.getPulse(dist_elbow)
        pulse_base = inv.getPulse(dist_z)

        arr = np.zeros([int(max(abs(round(pulse_arm/10)),abs(round(pulse_elbow/10)),abs(round(pulse_base/10)))),5])

        for i in range(1,int(abs(round(pulse_arm/10)))):
            if pulse_arm >= 0:
                arr[i,0] = 1
            else:
                arr[i,0] = -1

        for j in range(0,int(abs(round(pulse_elbow/10)))):
            if pulse_elbow >= 0:
                arr[j,0] = 1
            else:
                arr[j,0] = -1

        for k in range(0,int(abs(round(pulse_base/10)))):
            if pulse_base >= 0:
                arr[k,0] = 1
            else:
                arr[k,0] = -1

        preArm = arm
        preElbow = elbow
        preZ = z

        i = i +1
        '''

    # Automatically wait buffer to clean and then continue
    i = 1
    arm_steps = pulse_arm / 100
    elbow_steps = pulse_elbow / 100
    base_steps = pulse_base / 100
    tmp = [arm_steps, elbow_steps, base_steps]
    sort(tmp)

    trigger = GPIO.input(29)
    while i <= base_steps:
        if trigger == 0:
            spi.xfer([1,1,1,1,1])
            trigger = GPIO.input(29)
            i = i + 1
        else:
            trigger = GPIO.input(29)

    print(steps)
    print(arr)
    print pulse_arm, pulse_elbow, pulse_base
