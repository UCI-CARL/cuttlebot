import time
from LimitSwitch import LimitSwitch

limit_switch = LimitSwitch(pin=13, use_pullup_config=True)

start = time.time()
while(time.time() - start < 10):
    if(limit_switch.is_pressed()):
        print("Goteem!!!")
    else:
        print("Womp Womp :(")
    time.sleep(0.05)