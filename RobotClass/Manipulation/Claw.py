import time
from Manipulation.LimitSwitch import LimitSwitch
from Servo import Servo

class Claw():
    #Claw class constructor
    def __init__(self, servo_pin, right_limit_switch_pin, left_limit_switch_pin):
        #Initialize the object data members
        self.percent_open = None
        self.open_bound_deg = -90
        self.close_bound_deg = 0
        #Instantiate the composed objects
        self.right_limit_switch = LimitSwitch(pin=right_limit_switch_pin, use_pullup_config=True)
        self.left_limit_switch = LimitSwitch(pin=left_limit_switch_pin, use_pullup_config=True)
        self.servo = Servo(pin=servo_pin, reset_servo_position=False)

    #Set how open the claw should be
    def set_percent_open(self, pct_open):
        #Check to avoid recomputation
        if(pct_open == self.percent_open):
            return
        #Boundary Check
        if(pct_open < 0):
            pct_open = 0
        elif(pct_open > 100):
            pct_open = 100
        #Compute the angle to move the claw's servo
        angle = self.close_bound_deg + (self.open_bound_deg - self.close_bound_deg)*(pct_open/100.0)
        self.servo.set_angle_deg(angle)
        self.percent_open = pct_open

    #Get the current open percentage that the claw is set to be at
    def get_percent_open(self):
        return(self.percent_open)
    
    #return if an object is captured by looking at the state of the limit switches
    def is_object_captured(self):
        return(self.right_limit_switch.is_pressed() and self.left_limit_switch.is_pressed())
    
    #return is an object is fully released (left and right limit switch are not pressed)
    def is_object_fully_released(self):
        return((not self.right_limit_switch.is_pressed()) and (not self.left_limit_switch.is_pressed()))
    
    #Open the claw and slowly close it until an object is detected or the closing boundary is reached
    #WILL NEED TO FIX LATER TO INCORPORATE CONCURRENCY
    def capture_object(self):
        #open the claw (start at 100%)
        open_percent = 100
        #increment down, slowly closing the claw
        while(open_percent >= 0):
            #Set new claw state
            self.set_percent_open(open_percent)
            #if object is detected to be captured, then return to execute next task
            if(self.is_object_captured()):
                break
            #sleep for short amount of time and decrease percent openess
            time.sleep(0.75)
            open_percent -= 1

    def release_object(self):
        #start with the claw at its current state
        open_percent = self.percent_open
        #increment down, slowly closing the claw
        while(open_percent <= 100):
            #Set new claw state
            self.set_percent_open(open_percent)
            #if object is detected to be captured, then return to execute next task
            if(self.is_object_fully_released()):
                break
            #sleep for short amount of time and increment percent openess
            time.sleep(0.75)
            open_percent += 1