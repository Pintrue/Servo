import RPi.GPIO as GPIO
import time
from math import pi

GPIO.setmode(GPIO.BOARD)
servo = 7
GPIO.setup(servo, GPIO.OUT)
pwm = GPIO.PWM(servo, 50)

# servo 0: left/90->2.8, right/-90->11.3, mid/0->7.1
left, right, mid = 2.8, 11.3, 7.1

pwm.start(mid)
m = (left - right) / 180
b = mid
# m = (right - left) / 180.0	# linear relation between y/angle and x/DC: y = mx+b
# b = left

try:
	while(1):
		for desired_pos in range(0, 91):
#			DC = 43.0/900.0*desired_pos+2.7 #2.0 / 45.0 * desired_pos + 3
			DC = m * desired_pos + b
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
		for desired_pos in range(90, -91, -1):
#			DC = 43.0/900.0*desired_pos+2.7
			DC = m * desired_pos + b
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
		for desired_pos in range(-90, 1):
			DC = m * desired_pos + b
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
except KeyboardInterrupt:
	print "\nUser input KeyboardInterrput"

finally:
	print "Exiting gracefully..."
	pwm.stop()
	GPIO.cleanup()
