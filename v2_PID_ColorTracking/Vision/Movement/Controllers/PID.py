#import needed libraries
import numpy as np

#PID controller class for 2D vector objects
class PID_2D():
    def __init__(self, k_p, k_i, k_d):
        #gain paramters for the P, I, and D paths respectively
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        #variables for the input reference and the current output (y)
        self.ref = np.array((0,0))
        self.y = np.array((0,0))
        #variables related to the error (prev for derivatives and total for integrals)
        self.error = np.array((0,0))
        self.error_prev = np.array((0,0))
        self.error_total = np.array((0,0))
        #variable for servo PWM values
        self.PWM_duty_cycles = np.array((7.5,7.5))

    def reset(self):
        #reset output
        self.y = np.array((0,0))
        #reset errors
        self.error = np.array((0,0))
        self.error_prev = np.array((0,0))
        self.error_total = np.array((0,0))
        #reset variable for servo PWM values
        self.PWM_duty_cycles = np.array((7.5,7.5))

    #function for setting the reference value and resetting all associated variables
    def set_reference(self, ref):
        #set the reference value
        self.ref = ref

    #update the output to the newly measured output
    def update_output(self, y):
        #set the current output
        self.y = y
        #update the current error to the previous error
        self.error_prev = self.error
        #update the current error using the provided output
        self.error = self.ref - self.y
        #update the total error to incorpoate the new error
        self.error_total += self.error

    #update the PWM of the 2 given servos
    def update_servo_PWMs(self, servo1, servo2):
        #Apply the gains to the error (P), error rate (I), and total error (D)
        p = self.k_p*(self.error)
        i = self.k_i*(self.error - self.error_prev)
        d = self.k_d*(self.error_total)
        #get the sum of the PID values (this will be the PWM)
        self.PWM_duty_cycles += p + i + d
        #check to ensure the duty cycles are within the valid range (2.5, 12.5)
        servos = [servo1, servo2]
        for i in range(len(self.PWM_duty_cycles)):
            #case for PWM too low
            if(self.PWM_duty_cycles[i] < 2.5):
                servos[i].change_duty_cycle(2.5)
            #case for PWM too high
            elif(self.PWM_duty_cycles[i] > 12.5):
                servos[i].change_duty_cycle(12.5)
            #case for valid PWM signal
            else:
                servos[i].change_duty_cycle(self.PWM_duty_cycles[i])