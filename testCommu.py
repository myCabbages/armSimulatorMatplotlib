import spidev
import RPi.GPIO as GPIO
spi = spidev.SpiDev()
spi.open(0,0)
# Init GPIO for waiting buffer
GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.IN)
trigger = GPIO.input(29)

# Automatically wait buffer to clean and then continue
steps=3

i = 0
while i <= steps:
	# Automatically print trigger 
	if trigger == 0:
		spi.xfer([10,10,10,10,10])
		# Auspi.xfer([-10,10,10,10,20])
		
		trigger = GPIO.input(29)
		i = i + 1
	else:
		trigger = GPIO.input(29)
		# Automatically print trigger 
   
