import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
servo = 13
GPIO.setup(servo, GPIO.OUT)
pwm = GPIO.PWM(servo, 50)

left, right, mid = 2.2, 4.0, 3.1
pwm.start(right)

# servo 2: left->2.2, right->4.0, mid->3.1

m = (right - left) / 90.0	# linear relation between y/angle and x/DC: y = mx+b
#m = 1.0 / 50.0

try:
	while(1):
		for desired_pos in range(0, -91, -1):
#			DC = 43.0/900.0*desired_pos+2.7 #2.0 / 45.0 * desired_pos + 3
			DC = m * desired_pos + right
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
		for desired_pos in range(-90, 1):
#			DC = 43.0/900.0*desired_pos+2.7
			DC = m * desired_pos + right
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
except KeyboardInterrupt:
	print "\nUser input KeyboardInterrput"

finally:
	print "Exiting gracefully..."
	pwm.stop()
	GPIO.cleanup()
