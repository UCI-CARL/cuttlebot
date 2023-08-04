#Install by following: https://pypi.org/project/rpi-hardware-pwm/
from rpi_hardware_pwm import HardwarePWM
import time
import matplotlib.pyplot as plt

pwm = HardwarePWM(pwm_channel=1, hz=50)
pwm.start(7.5)
time.sleep(5)
pwm.change_duty_cycle(2.5)
time.sleep(1)

val = 2.5
target = 7.5
prev_error = target-val
d = 0
x = list()
y = list()
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
    
    pwm.change_duty_cycle(val)
    if(iter%50 == 0):
        print(val)
        x.append(iter)
        y.append(val)
    iter += 1

print("bye!")
pwm.stop()
plt.scatter(x, y, c='r')
plt.show()
