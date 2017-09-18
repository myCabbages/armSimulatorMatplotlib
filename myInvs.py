if __name__ == '__main__':
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
