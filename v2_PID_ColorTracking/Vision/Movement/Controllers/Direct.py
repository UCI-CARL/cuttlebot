import numpy as np

#Controller to directly move point to center via equation
class Direct_2D():
    #class constructor
    def __init__(self, cam_FOV, cam_dim, center_degrees = 5):
        #save passed parameters
        self.cam_FOV = cam_FOV
        self.cam_dim = np.array(cam_dim)
        self.center_degrees = center_degrees
        #define the proportionality constant for tan([theta_x, theta_y]) proportional to [x,y]
        self.proportionality_constant = (2.0*np.tan(self.cam_FOV/2.0)) / (self.cam_dim)
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
        self.y = np.arctan(self.proportionality_constant*y)

    #update the PWM of the 2 given servos
    def update_servo_PWMs(self, servo1, servo2):
        #Apply the equation to move to the new PWM values
        error = self.ref - self.y
        relative_theta = np.arctan(self.proportionality_constant*error)
        #if point is in center boundary, then no motion is necessary
        for i in range(len(relative_theta)):
            if(np.abs(relative_theta[i]) < self.center_degrees):
                relative_theta[i] = 0.0
        #compute an update the new PWM duty cycle
        relative_PWM = 7.5 + 0.05556*relative_theta
        self.PWM_duty_cycles += relative_PWM
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