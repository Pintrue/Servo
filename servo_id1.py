import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
servo = 7
GPIO.setup(servo, GPIO.OUT)
pwm = GPIO.PWM(servo, 50)
pwm.start(7)

# servo 0: left->2.8, right->11.3, mid->7.1

left, right, mid = 2.8, 11.3, 7.1
m = (11.3 - 2.8) / 180.0	# linear relation between y/angle and x/DC: y = mx+b


try:
	while(1):
		for desired_pos in range(0, 180):
#			DC = 43.0/900.0*desired_pos+2.7 #2.0 / 45.0 * desired_pos + 3
			DC = m * desired_pos + left
			pwm.ChangeDutyCycle(DC)
			time.sleep(.04)
		for desired_pos in range(180, 0, -1):
			DC = 43.0/900.0*desired_pos+2.7
			pwm.ChangeDutyCycle(DC)
			time.sleep(.04)
except KeyboardInterrupt:
	print "\nUser input KeyboardInterrput"

finally:
	print "Exiting gracefully..."
	pwm.stop()
	GPIO.cleanup()
