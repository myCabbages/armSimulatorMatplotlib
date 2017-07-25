import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.IN)
print(GPIO.input(29))
