import RPi.GPIO as GPIO
import spidev

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29,GPIO.IN)
spi = spidev.SpiDev()
spi.open(0,0)

trigger = GPIO.input(29)
i = 1
angle = input('input an angle: ')
steps = int(float(angle)/1.8*32.0/100.0)
print steps

while i <= steps:
    if trigger == 0:
        spi.xfer([0,0,0,25,0])
        trigger = GPIO.input(29)
        i = i + 1
    else:
        trigger = GPIO.input(29)

