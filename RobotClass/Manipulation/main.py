from Claw import Claw

#Test the Claw and Servo class together
import time
servo_pin = 11
right_limit_switch_pin = 13
left_limit_switch_pin = 15
claw = Claw(
    servo_pin = servo_pin,
    right_limit_switch_pin = right_limit_switch_pin, 
    left_limit_switch_pin = left_limit_switch_pin
)

claw.capture_object()
print("Captured!")
time.sleep(3)
claw.release_object()
print("Released!")
time.sleep(3)