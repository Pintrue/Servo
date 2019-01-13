import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
servo = 7
GPIO.setup(servo, GPIO.OUT)
pwm = GPIO.PWM(servo, 50)
pwm.start(7)

try:
	while(1):
		for desired_pos in range(0, 180):
			DC = 43.0/900.0*desired_pos+2.7 #2.0 / 45.0 * desired_pos + 3
			pwm.ChangeDutyCycle(DC)
			time.sleep(.01)
		for desired_pos in range(180, 0, -1):
			DC = 43.0/900.0*desired_pos+2.7
			pwm.ChangeDutyCycle(DC)
			time.sleep(.01)
except KeyboardInterrupt:
	print "\nUser input KeyboardInterrput"

finally:
	print "Exiting gracefully..."
	pwm.stop()
	GPIO.cleanup()
