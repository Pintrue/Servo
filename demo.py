import RPi.GPIO as GPIO
import time
from math import pi

clipped_angle = 5

GPIO.setmode(GPIO.BOARD)

# base servo
servo0 = 7
GPIO.setup(servo0, GPIO.OUT)
pwm0 = GPIO.PWM(servo0, 50)

left0, right0, mid0 = 2.8, 11.3, 7.1
pwm0.start(mid0)
m0 = (left0 - right0) / 180.0
b0 = mid0

# shoulder servo
servo1 = 11
GPIO.setup(servo1, GPIO.OUT)
pwm1 = GPIO.PWM(servo1, 50)

left1, right1, mid1 = 7.9, 2.2, 5.05
pwm1.start(left1)
m1 = (right1 - left1) / 120.0
b1 = left1

# forearm servo
servo2 = 13
GPIO.setup(servo2, GPIO.OUT)
pwm2 = GPIO.PWM(servo2, 50)

left2, right2, mid2 = 3.0, 7.3, 5.15
pwm2.start(right2)
m2 = (right2 - left2) / 90.0
b2 = right2


try:

	ja0 = input("Joint angle of BASE SERVO?\n\t")
	while ja0 < -90 or ja0 > 90:
		ja0 = input("Illegal value for BASE SERVO. PLZ try again\n\t")

	ja1 = input("Joint angle of SHOULDER SERVO?\n\t")
        while ja1 < 0 or ja1 > 120:
                ja1 = input("Illegal value for SHOULDER SERVO. PLZ try again\n\t")

	ja2 = input("Joint angle of FOREARM SERVO?\n\t")
        while ja2 < -90 or ja0 > 0:
                ja0 = input("Illegal value for FOREARM SERVO. PLZ try again\n\t")

		
	to_ja0 = 0
	while ja0 != 0:
		delta0 = min(clipped_angle, max(-clipped_angle, ja0))
		to_ja0 += delta0
		DC0 = m0 * to_ja0 + b0
		pwm0.ChangeDutyCycle(DC0)
		time.sleep(0.5)
		ja0 -= delta0

		
	to_ja1 = 0
	while ja1 != 0:
		delta1 = min(clipped_angle, max(-clipped_angle, ja1))
		to_ja1 += delta1
		DC1 = m1 * to_ja1 + b1
		pwm1.ChangeDutyCycle(DC1)
		time.sleep(0.5)
		ja1 -= delta1

		
	to_ja2 = 0
	while ja2 != 0:
		delta2 = min(clipped_angle, max(-clipped_angle, ja2))
		to_ja2 += delta2
		DC2 = m2 * to_ja2 + b2
		pwm2.ChangeDutyCycle(DC2)
		time.sleep(0.5)
		ja2 -= delta2


except KeyboardInterrupt:
	print "\nUser input KeyboardInterrupt"

finally:
	print "Exiting gracefully..."
	pwm0.stop()
	pwm1.stop()
	pwm2.stop()
	GPIO.cleanup()

