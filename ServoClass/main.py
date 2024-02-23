import time
from Servo import Servo

servo1 = Servo(pin=33, use_hardware_PWM=True, reset_servo_position=True)
servo1.set_angle_deg(0)
servo = Servo(pin=32, use_hardware_PWM=True, reset_servo_position=True)
servo.set_angle_deg(-90)
time.sleep(2)
servo.set_angle_deg(90)
time.sleep(2)
servo.set_angle_deg(0)
time.sleep(2)