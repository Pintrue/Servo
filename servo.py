import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo0, servo1, servo2 = 7, 11, 13
GPIO.setup(servo, GPIO.OUT)
pwm = GPIO.PWM(servo, 50)
pwm.start(7)

try:
	while True:
		desired_pos = input("Desired angle of the servo ")
		DC = 43.0/900.0*desired_pos+2.7 #2.0 / 45.0 * desired_pos + 3
		pwm.ChangeDutyCycle(DC)

except KeyboardInterrupt:
	print "\nExiting gracefully..."
	pwm.stop()
	GPIO.cleanup()
