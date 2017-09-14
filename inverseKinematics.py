import numpy as np
import math
from Tkinter import *
from getch import getch
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

class KeyControl:
    def __init__(self):
        # this dict keeps track of keys that have been pressed but not
        # released
        self.pressed = {}

        self._create_ui()

    def start(self):
        self._animate()
        self.root.mainloop()

    def _create_ui(self):
        self.root = Tk()
        self.m1 = Motion()
        self.m2 = Motion()
        self.m3 = Motion()
        self._set_bindings()

    def _animate(self):
        if self.pressed["d"]: self.m1.move_left()
        if self.pressed["k"]: self.m1.move_right()
        if self.pressed["w"]: self.m2.move_up2()
        if self.pressed["s"]: self.m2.move_down2()
        if self.pressed["o"]: self.m3.move_up3()
        if self.pressed["l"]: self.m3.move_down3()
        self.root.after(10, self._animate)

    def _set_bindings(self):
        for char in ["d","k","w","s","o", "l"]:
            self.root.bind("<KeyPress-%s>" % char, self._pressed)
            self.root.bind("<KeyRelease-%s>" % char, self._released)
            self.pressed[char] = False

    def _pressed(self, event):
        self.pressed[event.char] = True

    def _released(self, event):
        self.pressed[event.char] = False

class Motion:
    def __init__(self):
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

    def move_left(self):
        if trigger == 0:
            spi.xfer([10,0,0,0,0])
            trigger = GPIO.input(29)
        else:
            trigger = GPIO.input(29)

    def move_right(self):
        if trigger == 0:
            spi.xfer([-10,0,0,0,0])
            trigger = GPIO.input(29)
        else:
            trigger = GPIO.input(29)

    def move_up2(self):
        if trigger == 0:
            spi.xfer([0,10,0,0,0])
            trigger = GPIO.input(29)
        else:
            trigger = GPIO.input(29)

    def move_down2(self):
        if trigger == 0:
            spi.xfer([0,-10,0,0,0])
            trigger = GPIO.input(29)
        else:
            trigger = GPIO.input(29)

    def move_up3(self):
        if trigger == 0:
            spi.xfer([0,0,10,0,0])
            trigger = GPIO.input(29)
        else:
            trigger = GPIO.input(29)

    def move_down3(self):
        if trigger == 0:
            spi.xfer([0,0,-10,0,0])
            trigger = GPIO.input(29)
        else:
            trigger = GPIO.input(29)

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
    sensor_list = [sensor_base, sensor_arm, sensor_elbow,1,0]
    tmp = [15,-15,-15,0,0]

    '''
    # Find origin
    while sum(tmp) != 0:
        if trigger == 0:
            # print(trigger)
            tmp = np.multiply(sensor_list,[15,-15,-15,0,0])
            tmp = tmp.tolist()
            print(tmp)
            spi.xfer(tmp)
            #time.sleep(0.01)
            trigger = GPIO.input(29)
            sensor_base = GPIO.input(31)
            sensor_arm = GPIO.input(33)
            sensor_elbow = GPIO.input(35)
            sensor_list = [sensor_base, sensor_arm, sensor_elbow,1,0]
        elif trigger == 1:
            trigger = GPIO.input(29)
    '''

    #k = KeyControl()
    #k.start()

    '''
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

    # Automatically wait buffer to clean and then continue
    i = 1
    direct = [1,1,1,0,0]
    arm_steps = pulse_arm / 100
    elbow_steps = pulse_elbow / 100
    base_steps = pulse_base / 100

    if arm_steps < 0:
		direct[2] = -1
		arm_steps = np.abs(arm_steps)

    if elbow_steps < 0:
		direct[1] = -1
		elbow_steps = np.abs(elbow_steps)

    if base_steps < 0:
		direct[0] = -1
		base_steps = np.abs(base_steps)
    print direct

    tmp = [arm_steps, elbow_steps,base_steps]
    tmp.sort()
    idx_arm = tmp.index(arm_steps)
    idx_elbow = tmp.index(elbow_steps)
    idx_base = tmp.index(base_steps)
    print pulse_arm, pulse_elbow, pulse_base
    print tmp
    '''

    filename = raw_input('Please Enter a filename: ')
    if filename == 'log.txt':
        f = open(filename, 'w')

        trigger = GPIO.input(29)
        while True:
            key = ord(getch())
            if key == 27:
                f.write('27\n')
                break
            elif key == 100:
                if trigger == 0:
                    spi.xfer([10,0,0,0,0])
                    f.write('100\n')
                    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 107:
                if trigger == 0:
		    spi.xfer([-10,0,0,0,0])
                    f.write('107\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 119:
                if trigger == 0:
		    spi.xfer([0,10,0,0,0])
                    f.write('119\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 115:
                if trigger == 0:
		    spi.xfer([0,-10,0,0,0])
                    f.write('115\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 111:
                if trigger == 0:
		    spi.xfer([0,0,10,0,0])
                    f.write('111\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 108:
                if trigger == 0:
		    spi.xfer([0,0,-10,0,0])
                    f.write('108\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 114:
                if trigger == 0:
		    spi.xfer([0,0,0,10,0])
                    f.write('114\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)
            elif key == 102:
                if trigger == 0:
		    spi.xfer([0,0,0,-10,0])
                    f.write('102\n')
		    trigger = GPIO.input(29)
		else:
		    trigger = GPIO.input(29)

        f.close()

    elif filename == 'a':
        f = open('log.txt', 'r')
        trigger = GPIO.input(29)
        tmp_ = f.readlines()
        i = 0
        while 1:
			if trigger == 0:
				key = tmp_[i]
				key = int(key)
				print(key)
				if key == 27:
					break
				elif key == 100:
					spi.xfer([10,0,0,0,0])
					print('pass')
					trigger = GPIO.input(29)
				elif key == 107:
					spi.xfer([-10,0,0,0,0])
					trigger = GPIO.input(29)
				elif key == 119:
					spi.xfer([0,10,0,0,0])
					print('pass 119')
					trigger = GPIO.input(29)
				elif key == 115:
					spi.xfer([0,-10,0,0,0])
					trigger = GPIO.input(29)
				elif key == 111:
					spi.xfer([0,0,10,0,0])
					print('pass 111')
					trigger = GPIO.input(29)
				elif key == 108:
					spi.xfer([0,0,-10,0,0])
					trigger = GPIO.input(29)
				elif key == 114:
					spi.xfer([0,0,0,-10,0])
					trigger = GPIO.input(29)
				elif key == 102:
					spi.xfer([0,0,0,10,0])
					trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)



    '''
    try:
         f_r = open('log.txt', 'r')
    except:
        f = open(filename,'w')
        #for last in f_r:pass

        f.write('B' + str(bs) + 'E' + str(elb) + 'A' + str(ar) + '\n')
        f.close()
    else:
        f = open('log.txt','a')
        f.write('B' + str(bs) + 'E' + str(elb) + 'A' + str(ar) + '\n')

        f.close()
        f_r.close()



    arm_steps = ar/4*6400/100
    elbow_steps = elb/4*6400/100
    base_steps = bs/4*6400/100

    tmp = [arm_steps, elbow_steps, base_steps]
    tmp.sort()
    idx_arm = tmp.index(arm_steps)
    idx_elbow = tmp.index(elbow_steps)
    idx_base = tmp.index(base_steps)
    print tmp

    trigger = GPIO.input(29)
    if idx_arm == 0 and idx_elbow == 1 and idx_base == 2:
		while i <= tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,10,0,0])
				tmp2 = tmp2.tolist()
				print(tmp2)
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,0,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[2]-tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,0,0,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)


    if idx_arm == 0 and idx_elbow == 2 and idx_base == 1:
		while i <= tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,10,0,0])
				tmp2 = tmp2.tolist()
                print(tmp2)
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,0,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[2]-tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[0,10,0,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)



    if idx_arm == 1 and idx_elbow == 0 and idx_base == 2:
		while i <= tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,10,0,0])
				tmp2 = tmp2.tolist()
                print(tmp2)
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,0,10,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[2]-tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,0,0,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)



    if idx_arm == 1 and idx_elbow == 2 and idx_base == 0:
		while i <= tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,10,0,0])
				tmp2 = tmp2.tolist()
                print(tmp2)
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[1]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[0,10,10,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[2]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[0,10,0,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)



    if idx_arm == 2 and idx_elbow == 1 and idx_base == 0:
		while i <= tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,10,0,0])
				tmp2 = tmp2.tolist()
                print(tmp2)
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[0,10,10,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[2]-tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[0,0,10,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)


    if idx_arm == 2 and idx_elbow == 0 and idx_base == 1:
		while i <= tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,10,10,0,0])
				tmp2 = tmp2.tolist()
                print(tmp2)
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[10,0,10,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)

		i = 1
		while i <= tmp[2]-tmp[1]-tmp[0]:
			if trigger == 0:
				tmp2 = np.multiply(direct,[0,10,10,0,0])
				tmp2 = tmp2.tolist()
				spi.xfer(tmp2)
				trigger = GPIO.input(29)
				i = i + 1
			else:
				trigger = GPIO.input(29)
        '''
