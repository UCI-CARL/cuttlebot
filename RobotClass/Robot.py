#The robot class (for now) will be composed of the RVR+, Vision module, and claw (...adding soon)
import time
import numpy as np
import sphero_sdk as sphero
from Manipulation.Claw import Claw
from Vision.Perception import Perception

class Robot():
    #class constructor
    def __init__(self):
        #Instantiate the modules on the robot alongside the robot itself
        #First the rvr
        self.rvr = sphero.SpheroRvrObserver()
        self.rvr.wake()
        #give the rvr time to wake up
        time.sleep(2)
        #Reset the YAW on the rvr
        self.rvr.reset_yaw()
        #Now the claw (NOT USING THE CLAW FOR NOW)
        ########self.claw = Claw(servo_pin=-1, right_limit_switch_pin=-1, left_limit_switch_pin=-1)
        #Last, the vision module
        self.vision = Perception()

    def face_color(self):
        self.vision.camera.set_color_filter(120, precision=15)
        while(1):
            mask = self.vision.camera.get_color_mask()
            #If there are less than 10 active pixels
            if(np.sum(mask/255) < 10):
                #recenter eyes
                cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556
                proportion = 0.3
                robot_proportion_angle_deg = proportion*cur_pan_angle
                pan_compensation_PWM = 7.5 + 0.055556*(-robot_proportion_angle_deg)
                new_pan_PWM = self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0] + pan_compensation_PWM
                self.vision.pan_tilt_unit.set_servo_PWM_duty_cycles(new_pan_PWM, self.vision.pan_tilt_unit.controller.PWM_duty_cycles[1])
                #self.rvr.reset_yaw()
                #self.rvr.reset_locator_x_and_y()
                #self.rvr.drive_to_position_si(
                #    linear_speed = 0.5,
                #    yaw_angle = robot_proportion_angle_deg,
                #    x = 0,
                #    y = 0,
                #    flags = 0
                #)
                #skip vision processing
                if(np.abs(robot_proportion_angle_deg) < 7.5):
                    self.rvr.drive_tank_si_units(
                        left_velocity = 0,
                        right_velocity = 0
                    )
                else:
                    self.rvr.drive_tank_si_units(
                        left_velocity = -robot_proportion_angle_deg/90.0,
                        right_velocity = robot_proportion_angle_deg/90.0
                    )
                continue
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            self.vision.pan_tilt_unit.update(avg_point)
            #Now move the robot according to the current angle of the pan unit
            cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556
            proportion = 0.3
            robot_proportion_angle_deg = proportion*cur_pan_angle
            #pan_compensation_PWM = 7.5 + 0.055556*(-robot_proportion_angle_deg)
            #new_pan_PWM = self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]+pan_compensation_PWM
            #self.vision.pan_tilt_unit.set_servo_PWM_duty_cycles(new_pan_PWM, self.vision.pan_tilt_unit.controller.PWM_duty_cycles[1])
            #self.rvr.reset_yaw()
            #self.rvr.reset_locator_x_and_y()
            #self.rvr.drive_to_position_si(
            #    linear_speed = 0.5,
            #    yaw_angle = robot_proportion_angle_deg,
            #    x = 0,
            #    y = 0,
            #    flags = 0
            #)
            if(np.abs(robot_proportion_angle_deg) < 5.0):
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )
            else:
                self.rvr.drive_tank_si_units(
                    left_velocity = -robot_proportion_angle_deg/90.0,
                    right_velocity = robot_proportion_angle_deg/90.0
                )

    #class destructor
    def __del__(self):
        #close the rvr
        self.rvr.close()