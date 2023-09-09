import time
from Servo import Servo

servo = Servo(pin=32, use_hardware_PWM=True, reset_servo_position=True)
servo.set_angle_deg(-90)
time.sleep(2)
servo.set_angle_deg(90)
time.sleep(2)
servo.set_angle_deg(0)
time.sleep(2)