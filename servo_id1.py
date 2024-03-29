import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
servo = 11
GPIO.setup(servo, GPIO.OUT)
pwm = GPIO.PWM(servo, 50)
pwm.start(8.2)

# servo 0: left->2.8, right->11.3, mid->7.1

left, right, mid = 8.2, 2.2, 5.2
m = (left - right) / 120.0	# linear relation between y/angle and x/DC: y = mx+b


try:
	while(1):
		for desired_pos in range(0, -121, -1):
#			DC = 43.0/900.0*desired_pos+2.7 #2.0 / 45.0 * desired_pos + 3
			DC = m * desired_pos + left
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
		for desired_pos in range(-120, 1):
#			DC = 43.0/900.0*desired_pos+2.7
			DC = m * desired_pos + left
			pwm.ChangeDutyCycle(DC)
			time.sleep(0.04)
except KeyboardInterrupt:
	print "\nUser input KeyboardInterrput"

finally:
	print "Exiting gracefully..."
	pwm.stop()
	GPIO.cleanup()
