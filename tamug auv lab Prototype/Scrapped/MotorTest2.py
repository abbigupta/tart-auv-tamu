import RPi.GPIO as GPIO
from time import sleep


ledpin = 35
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,6000)
pi_pwm.start(0)

pi_pwm.ChangeDutyCycle(50)
sleep(50)
for duty in range(50,101,1):
	pi_pwm.ChangeDutyCycle(duty)
	sleep(0.01)
sleep(0.5)

for duty in range(100,50,-1):
	pi_pwm.ChangeDutyCycle(duty)
	sleep(0.01)
sleep(0.5)
