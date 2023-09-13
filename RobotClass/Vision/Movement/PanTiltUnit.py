#import needed libraries
import time
import numpy as np
from rpi_hardware_pwm import HardwarePWM
from Vision.Movement.Controllers.PID import PID_2D
from Vision.Movement.Controllers.Proportional import Proportional_2D
from Vision.Movement.Controllers.Direct import Direct_2D

#Class for moving the pan-tile-unit
class PanTiltUnit():
    #Class constructor 
    def __init__(self):
        #Instantiate the servo for pwm channel 0 (GPIO12/Pin32) and start the duty cycle at 7.5% (0 degrees)
        self.pan_servo = HardwarePWM(pwm_channel=0, hz=50)
        self.pan_servo.start(7.5)
        #Instantiate the servo for pwm channel 1 (GPIO13/Pin33) and start the duty cycle at 7.5% (0 degrees)
        self.tilt_servo = HardwarePWM(pwm_channel=1, hz=50)
        self.tilt_servo.start(7.5)
        self.servo_PWM_duty_cycles = np.array((7.5,7.5))
        #Wait 1 second to give the servos time to move to their center positions
        time.sleep(1)

        #Instantiate the controller object
        self.controller = PID_2D(k_p=0.005, k_i=0.00005, k_d=0.00005)
        #self.controller = Proportional_2D(cam_FOV_width=95, cam_width=320, cam_height=240, hysteresis_rad=0.05, target_value_proportion=0.1)
        #self.controller = Direct_2D(cam_FOV_width=95, cam_width=320, cam_height=240, hysteresis_rad=0.05)
        #For tracking problems, set the reference to (0,0)
        self.controller.set_reference((0.0, 0.0))

    #update the pan-tilt-unit with the new output point and move the servos to center the given point
    def update(self, output_point):
        #update the output of the PID controller
        self.controller.update_output(output_point)
        self.controller.update_servo_PWMs(self.pan_servo, self.tilt_servo)
        self.servo_PWM_duty_cycles = self.controller.PWM_duty_cycles
    
    #Set the PWMs for the servos directly
    def set_servo_PWM_duty_cycles(self, pan_PWM, tilt_PWM):
        self.pan_servo.change_duty_cycle(pan_PWM)
        self.tilt_servo.change_duty_cycle(tilt_PWM)
        self.servo_PWM_duty_cycles = np.array((pan_PWM,tilt_PWM))
        self.controller.PWM_duty_cycles = self.servo_PWM_duty_cycles
