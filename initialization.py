import RPi.GPIO as GPIO
import spidev

if __name__ == '__main__':
    # Initialize GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(29, GPIO.IN)
    GPIO.setup(31, GPIO.IN)
    
    # Initialize spidev
    spi = spidev.SpiDev()
    spi.open(0,0)
 
    trigger = GPIO.input(29)
    trig_sensor = GPIO.input(31)

    while 1:
        if trigger == 0:
            if trig_sensor == 1:
                spi.xfer([10,10,10,10,10])
                trigger = GPIO.input(29)
                trig_sensor = GPIO.input(31)
            elif trig_sensor == 0:
                break
        elif trigger == 1:
            trigger = GPIO.input(29)
            


