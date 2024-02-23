#Test for working with the servo motor (sweep through the angles)

#import libraries
import RPi.GPIO as GPIO
import time

#set GPIO numbering mode (left-right;top-down numbering starting from 1)
GPIO.setmode(GPIO.BOARD)

#Set pin 32 as an output pin
servo_pin = 32 #pan servo (Working out issues - NOTE: should use SERVO class instead)
GPIO.setup(servo_pin, GPIO.OUT)

#set PWM to 50Hz (servo expects 50Hz PWM to read from: 20 ms per pulse)
servo = GPIO.PWM(servo_pin, 50)

#begin the servo motors in off state
servo.start(0)
time.sleep(1)

#start at 0 degrees
servo.ChangeDutyCycle(1.5/20.0)
time.sleep(0.5)
servo.ChangeDutyCycle(0)
time.sleep(0.5)

#Sweep through 0.5 to 2.4 ms pulse
for i in range(5, 25):
   pulse_ms = i/10
   duty_cycle = (pulse_ms/20.0)*100.0
   servo.ChangeDutyCycle(duty_cycle)
   time.sleep(0.5)

#Clean up things at the end
servo.stop()
GPIO.cleanup()

