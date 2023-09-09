from Claw import Claw

#Test the Claw and Servo class together
import time
pin = 11
claw = Claw(servo_pin=pin)
claw.set_percent_open(0)
time.sleep(2)
claw.set_percent_open(50)
time.sleep(2)
claw.set_percent_open(100)
time.sleep(2)
claw.set_percent_open(50)
time.sleep(2)
claw.set_percent_open(0)
time.sleep(2)