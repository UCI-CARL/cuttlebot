#Test for working with the pan-tilt unit

#import libraries
import RPi.GPIO as GPIO
import time
import random

#set GPIO numbering mode (left-right;top-down numbering starting from 1)
GPIO.setmode(GPIO.BOARD)

#Set pin 11 as an output pin
bottom_servo_pin = 11
top_servo_pin = 13
GPIO.setup(bottom_servo_pin, GPIO.OUT)
GPIO.setup(top_servo_pin, GPIO.OUT)

#set PWM to 50Hz (servo expects 50Hz PWM to read from: 20 ms per pulse)
bottom_servo = GPIO.PWM(bottom_servo_pin, 50)
top_servo = GPIO.PWM(top_servo_pin, 50)

#begin the servo motors in off state
bottom_servo.start(0)
top_servo.start(0)
time.sleep(1)

#function to move the servos on the pan-tilt unit
absolute_max_angle = 90
current_position = None


def move_to_pos(angle_vector, current_position):
    ''' 
    Params:
        angle_vector: a vector of size 2, with first entry the angle to sweep for the bottom servo, and second entry for the top servo
    '''
    if(abs(angle_vector[0]) > absolute_max_angle or abs(angle_vector[1]) > absolute_max_angle):
        raise(Exception("out of bounds error for angle"))
    #get the number of ms in the pulse (20ms or 50Hz) for the angle
    bot_pulse_ms = 1 + (angle_vector[0]+absolute_max_angle)/(2*absolute_max_angle)
    top_pulse_ms = 1 + (angle_vector[1]+absolute_max_angle)/(2*absolute_max_angle)
    #Get the duty cycle for the specified PWM
    bot_duty_cycle = (bot_pulse_ms/20.0)*100
    top_duty_cycle = (top_pulse_ms/20.0)*100
    #Now set the PWM so that the servo motors can move to their specified degree
    bottom_servo.ChangeDutyCycle(bot_duty_cycle)
    top_servo.ChangeDutyCycle(top_duty_cycle)
    #calculate the amount of time to move to the spots
    if(current_position == None):
        positional_difference = (2*absolute_max_angle, 2*absolute_max_angle)
    else:
        positional_difference = [abs(angle_vector[i] - current_position[i]) for i in range(len(angle_vector))]
    w = 400.0 #degrees/sec (estimated from the load)
    c = 0.025 #constant wait time to account for information being sent (5ms + 1 clock cycle at 50Hz = 20ms)
    movement_time = [p/w + c for p in positional_difference]
    #wait for the servo to move to the spots
    print(f"{current_position} -> {angle_vector} = {positional_difference}, {movement_time}")
    if(movement_time[0] > movement_time[1]): #bottom servo will need more time
        time.sleep(movement_time[1])
        top_servo.ChangeDutyCycle(0)
        time.sleep(movement_time[0] - movement_time[1])
        bottom_servo.ChangeDutyCycle(0)
    elif(movement_time[1] > movement_time[0]): #top servo will need more time
        time.sleep(movement_time[0])
        bottom_servo.ChangeDutyCycle(0)
        time.sleep(movement_time[1] - movement_time[0])
        top_servo.ChangeDutyCycle(0)
    else: #the servos will take the same time to move
        time.sleep(movement_time[0])
        top_servo.ChangeDutyCycle(0)
        bottom_servo.ChangeDutyCycle(0)
    current_position = angle_vector

#move and stop to each position
print("=====================")
print("reseting position")
move_to_pos((0, 0), current_position=None) # start
time.sleep(1)
current_position = (0, 0)

print("=====================")
print("progressively quick movements through the angles")
wait_times = [1, 0.5, 0]
servo_angles = [-45, 0, 45]
for wait_time in wait_times:
   for bot_angle in servo_angles:
      for top_angle in servo_angles:
         pos = (bot_angle, top_angle)
         move_to_pos(pos, current_position)
         current_position = pos
         time.sleep(wait_time)

print("=====================")
print("resetting position")
move_to_pos((0, 0), current_position)
current_position = (0, 0)
time.sleep(1)

print("=====================")
print("Sweep through all integer degrees as fast as possible")
for  elevation in range(-45, 45, 15):
   for azimuth in range(-45, 45, 15):
      move_to_pos((azimuth, elevation), current_position)
      current_position = (azimuth, elevation)

print("=====================")
print("resetting position")
move_to_pos((0, 0), current_position)
current_position = (0, 0)
time.sleep(1)

print("=====================")
print("Perform the 2 extreme actions (diagonal and cross diagonal)")
move_to_pos((45, 45), current_position)
current_position = (45, 45)
time.sleep(1)
pos_order_diagonal = [(45,45), (-45,-45)]
for pos in pos_order_diagonal:
   move_to_pos(pos, current_position)
   current_position = pos
time.sleep(2)

move_to_pos((-45, 45), current_position)
current_position = (-45, 45)
time.sleep(1)
pos_order_off_diagonal = [(-45,45), (45,-45)]
for pos in pos_order_off_diagonal:
   move_to_pos(pos, current_position)
   current_position = pos
time.sleep(2)

print("=====================")
print("resetting position")
move_to_pos((0, 0), current_position)
current_position = (0, 0)
time.sleep(1)

print("=====================")
print("move to 25 random locations")
for i in range(0, 25):
   rand_pos = [15*random.randrange(-absolute_max_angle/15, absolute_max_angle/15) for i in range(2)]
   move_to_pos(rand_pos, current_position)
   current_position = rand_pos

print("=====================")
print("resetting position")
move_to_pos((0, 0), current_position)
current_position = (0, 0)

#Clean up things at the end
bottom_servo.stop()
top_servo.stop()
GPIO.cleanup()

