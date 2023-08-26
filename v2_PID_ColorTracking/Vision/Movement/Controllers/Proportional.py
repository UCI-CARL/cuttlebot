import numpy as np

#Controller to directly move point to center via equation
class Proportional_2D():
    #class constructor
    def __init__(self, cam_FOV_width, cam_width, cam_height, hysteresis_rad=0.05, target_value_proportion=0.1):
        #save passed parameters
        self.cam_FOV_width = cam_FOV_width
        self.cam_FOV_height = cam_FOV_width*(cam_height/cam_width)
        self.cam_width = cam_width
        self.cam_height = cam_height
        self.hysteresis_rad = hysteresis_rad
        self.target_value_proportion = target_value_proportion
        #define the proportionality constant for tan([theta_x, theta_y]) proportional to [x,y]
        self.proportionality_constant = ((2.0*np.arctan(self.cam_FOV_width/2.0)) / (self.cam_width), (2.0*np.arctan(self.cam_FOV_height/2.0)) / (self.cam_width))
        #variables for the input reference and the current output (y)
        self.ref = np.array((0.0,0.0))
        self.y = np.array((0.0,0.0))
        #save the current PWM of the servos
        self.PWM_duty_cycles = np.array((7.5,7.5))

    #reset saved parameters
    def reset(self):
        self.ref = np.array((0.0,0.0))
        self.y = np.array((0.0,0.0))
        self.PWM_duty_cycles = np.array((7.5,7.5))

    def set_reference(self, ref):
        self.ref = ref

    def update_output(self, y):
        self.y = y

    #update the PWM of the 2 given servos
    def update_servo_PWMs(self, servo1, servo2):
        #Apply the equation to move to the new PWM values
        error = self.ref - self.y
        relative_theta = np.arctan(self.proportionality_constant*error)
        #if point is in center boundary, then no motion is necessary
        for i in range(len(relative_theta)):
            if(np.abs(relative_theta[i]) < self.hysteresis_rad):
                relative_theta[i] = 0.0
        #compute an update the new PWM duty cycle
        relative_PWM = 3.18310*relative_theta
        self.PWM_duty_cycles += self.target_value_proportion*relative_PWM
        #check to ensure the duty cycles are within the valid range (2.5, 12.5)
        servos = [servo1, servo2]
        for i in range(len(self.PWM_duty_cycles)):
            #case for PWM too low
            if(self.PWM_duty_cycles[i] < 2.5):
                servos[i].change_duty_cycle(2.5)
                self.PWM_duty_cycles[i] = 2.5
            #case for PWM too high
            elif(self.PWM_duty_cycles[i] > 12.5):
                servos[i].change_duty_cycle(12.5)
                self.PWM_duty_cycles[i] = 12.5
            #case for valid PWM signal
            else:
                servos[i].change_duty_cycle(self.PWM_duty_cycles[i])