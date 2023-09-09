from Servo import Servo

class Claw():
    #Claw class constructor
    def __init__(self, servo_pin):
        self.percent_open = None
        self.open_bound_deg = -90
        self.close_bound_deg = 0
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