#import libraries
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt

#set GPIO numbering mode (left-right;top-down numbering starting from 1)
GPIO.setmode(GPIO.BOARD)

#Set pin 11 as an output pin
servo_pin = 11
GPIO.setup(servo_pin, GPIO.OUT)

#set PWM to 50Hz (servo expects 50Hz PWM to read from: 20 ms per pulse)
pwm = GPIO.PWM(servo_pin, 50)

#begin the servo motors in off state
pwm.start(7.5)
time.sleep(5)
pwm.ChangeDutyCycle(2.5)
time.sleep(1)

val = 2.5
target = 7.5
prev_error = target-val
d = 0

x=list()
y=list()
for iter in range(10000):
    error = target-val
    p = 0.0005*(error)
    i = 0.005*(error - prev_error)
    prev_error = error
    d += 0.000005*(error)
    val += p + i + d
    if(val < 2.5):
        val = 2.5
    elif(val > 12.5):
        val = 12.5
    
    pwm.ChangeDutyCycle(val)
    time.sleep(0.0001)
    if(iter%50 == 0):
        print(val)
        x.append(iter)
        y.append(val)

print("Bye!")
pwm.stop()
GPIO.cleanup()
plt.scatter(x, y, c='r')
plt.show()