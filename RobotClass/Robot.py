#The robot class (for now) will be composed of the RVR+, Vision module, and claw (...adding soon)
import time
import numpy as np
import sphero_sdk as sphero
import cv2
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
        #Now the claw (pins 11 and 13 for the limit switches are not currently in use)
        self.claw = Claw(servo_pin=7, right_limit_switch_pin=11, left_limit_switch_pin=13)
        #Last, the vision module
        self.vision = Perception()

    def do_nothing(self):
        self.rvr.sensor_control.sensors["Locator"][0].enable_streaming_service("Locator")
        print(self.rvr.sensor_control.sensors["Locator"][0].enabled_streaming_services_by_id)
        rate_ms = 50 # min is 33 ms
        self.rvr.sensor_control.start(rate_ms) #SOMTHING WRONG WITH START METHOD!
        print(self.rvr.sensor_control.enabled_sensors)
        print("=========================")
        #for i in range(50):
            #print(self.rvr.sensor_control.sensors["Locator"][0].streaming_services_configuration)
            #time.sleep(0.1)
        self.rvr.sensor_control.stop()
        self.rvr.sensor_control.sensors["Locator"][0].disable_all_streaming_services()
        print("Closed!")
        for i in range(5):
            img = self.vision.camera.get_image()
            cv2.imshow('image',img)
            cv2.imwrite(f"/home/cuttlebot/cuttlebot/RobotClass/Vision/Sight/CalibrationSet_Cam0/calibration_img{i+10}.png", img)

            # waitKey() waits for a key press to close the window and 0 specifies indefinite loop
            cv2.waitKey(0)
            # cv2.destroyAllWindows() simply destroys all the windows we created.
            cv2.destroyAllWindows()

    def __align_via_body_movement(self):
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0
        #control loop
        while(1):
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if(np.sum(mask/255) < 20):
                #slow to a stop if nothing is found
                print("looking...")
                driving_left_velocity *= 0.95
                driving_right_velocity *= 0.95
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            #Now move the robot proportional to the relative point of the largest object (P controller)
            max_speed = 0.25
            if(np.abs(rel_point[0]) <= 1):
                #if object is centered, the return from procedure
                print("aligned!")
                driving_left_velocity = 0
                driving_right_velocity = 0
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                return
            elif(rel_point[0] > 0):
                #X-position of object to left of center -> turn right
                turning_speed = np.abs(rel_point[0]/self.vision.camera.width)*max_speed
                driving_left_velocity = turning_speed
                driving_right_velocity = -turning_speed
            else:
                #X-position of object to right of center -> turn left
                turning_speed = np.abs(rel_point[0]/self.vision.camera.width)*max_speed
                driving_left_velocity = -turning_speed
                driving_right_velocity = turning_speed
            #after computing the new velocity of the wheels, set the values to the rvr
            self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )

    def align_with_object_at_distance():
        #Look for red object
        self.vision.camera.set_color_filter(0, precision=15)
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0
        prev_width = None
        prev_
        #reset IMU values
        self.rvr.reset_yaw()
        time.sleep(0.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(0.1)
        #control loop
        while(1):
            #Give some waiting time before stating each loop
            time.sleep(0.1)
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            print(f"mask total active pixels = {np.sum(mask/255)}")
            if(np.sum(mask/255) < 20):
                #stop the robot
                ########driving_left_velocity *= 0.75
                ########driving_right_velocity *= 0.75
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            p_width = largest_contour_bounding_box[2]
            #print(f"Width_1={largest_contour_bounding_box[2]}")
            #Initiallize the previous width value to compare
            if(prev_width == None):
                p_width = prev_width
                movement = 0
                continue
            #get the depth from the previous and current width values
            obj_depth = self.vision.get_temporal_difference_object_depth(
                p_length1=prev_width, 
                p_length2=p_width, 
                distance_moved_radially_inward_m=movement
            )
            print(f"distance={obj_depth}")
            #open claw dependent on distance
            #
            #
            #
            #adjust velocity and move forward
            max_speed = 1 #m/s
            velocity_change = 0.2 #max_speed*(2/(1+np.exp(obj_depth/2)) - 1)
            driving_left_velocity = velocity_change
            driving_right_velocity = velocity_change
            self.rvr.drive_tank_si_units(
                left_velocity = driving_left_velocity,
                right_velocity = driving_right_velocity
            )

            
             


    def get_object_depth_info_forward_to_back(self):
        #Look for red object
        self.vision.camera.set_color_filter(0, precision=15)
        #face the object
        self.__align_via_body_movement()
        #take 3 pictures and get the average bounding box pixel width of all detected one
        p_width_1 = 0
        img_found = 0
        for i in range(5):
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if(np.sum(mask/255) < 20):
                #wait and try again
                time.sleep(0.1)
                print("Width_1=None")
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            img_found += 1
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            p_width_1 += largest_contour_bounding_box[2]
            print(f"Width_1={largest_contour_bounding_box[2]}")
        #get the average width or specify no width for no object being found
        if(img_found != 0):
            print(f"total_width_1={p_width_1}")
            p_width_1 /= img_found
            print(f"img_found={img_found}")
            print(f"avg_width_1={p_width_1}")
        else:
            p_width_1 = None
        #move radially inward by m meters
        movement = 0.1 #m
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle = 0 if (movement>0) else 180,
            x=0,
            y=movement,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2.5)
        #align body once more with object
        self.__align_via_body_movement()
        #get the average width of 3 pictures for the object at the new position
        p_width_2 = 0
        img_found = 0
        for i in range(5):
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if(np.sum(mask/255) < 20):
                #wait and try again
                time.sleep(0.1)
                print("Width_2=None")
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            img_found += 1
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            p_width_2 += largest_contour_bounding_box[2]
            print(f"Width_2={largest_contour_bounding_box[2]}")
        #get the average width or specify no width for no object being found
        if(img_found != 0):
            print(f"total_width_2={p_width_2}")
            p_width_2 /= img_found
            print(f"img_found={img_found}")
            print(f"avg_width_2={p_width_2}")
        else:
            p_width_2 = None
        #Ensure that the width of the object was found in both cases
        if(p_width_1==None or p_width_2==None):
            print(f"Images not found! width1={p_width_1}, width2={p_width_2}")
            return None
        #Use the two average widths of the two positions to get an estimation of depth
        obj_depth = self.vision.get_temporal_difference_object_depth(
            p_length1=p_width_1, 
            p_length2=p_width_2, 
            distance_moved_radially_inward=movement
        )
        print(obj_depth)


    def get_object_depth_info_side_to_side(self):
        #####Can also perform this task below for each contour in the image#####
        #Look for red object
        self.vision.camera.set_color_filter(0, precision=15)
        #set cameras to 0, 0 angle
        self.vision.pan_tilt_unit.set_servo_angles(0, 0)
        #face largest object with color of interest
        self.__align_via_body_movement()
        #turn camera 90deg either left or right (turning left right now, but can be dependent on environmental constraints)
        self.vision.pan_tilt_unit.set_servo_angles(90, 0)
        #turn RVR in opposite direction 90 degrees
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle=-90,
            x=0,
            y=0,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2.5)
        #readjust rvr to center camera on largest object up
        self.__align_via_body_movement()
        #save centroid for reference
        print("mid SNAP!")###############
        #Move forward/back and compare the center-point of the object when the side first reaches the end of the frame OR when a limit of +/- 0.15m
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle=0,
            x=0,
            y=0.15,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2)
        #take picture after moving forward 0.15m
        print("front SNAP!")################
        #move back 0.3m
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle=0,
            x=0,
            y=0.15,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2)
        #take picture after being 0.15m back from reference position
        print("back SNAP!") ##############
        #use trig to estimate the depth information (2 results: mid w/ back & mid w/ front comparison)
        ##################################### depth_front_estimate = 
        ##################################### depth_back_estimate = 
        #take the average depth of the 2 computations
        depth = (depth_front_estimate + depth_back_estimate)/2
        #return the robot to the center, directly facing the object
        
        #print the estimated depth from the camera to the object
        print("Last statment!")

    def work_that_claw(self):
        while(1):
            #open claw
            self.claw.set_percent_open(100)
            time.sleep(2.5)
            #close the claw
            self.claw.set_percent_open(0)
            time.sleep(2.5)


    def move_to_color(self):
        #Look for red object
        self.vision.camera.set_color_filter(0, precision=15)
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0
        #control loop
        while(1):
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if(np.sum(mask/255) < 20):
                #stop the robot
                driving_left_velocity *= 0.75
                driving_right_velocity *= 0.75
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            self.vision.pan_tilt_unit.update(rel_point)
            #Now move the robot according to the current angle of the pan unit
            cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556 #!!!need to add servo class to PTU so we can get the servo angle
            proportion = 0.3
            robot_proportion_angle_deg = proportion*cur_pan_angle
            #Filter out small robot movements
            if(np.abs(robot_proportion_angle_deg) < 1.5):
                driving_left_velocity = 0
                driving_right_velocity = 0
            #if the movement is big enough, then move the robot
            else:
                driving_left_velocity = -robot_proportion_angle_deg/90.0
                driving_right_velocity = robot_proportion_angle_deg/90.0
            #in addition to turning the robot, add an offset to move it forward toward the object of intetest
            velocity_limit = 0.50 #m/s
            optimal_object_width = 125
            driving_left_velocity += velocity_limit*(1.0 - largest_contour_bounding_box[2]/optimal_object_width)
            driving_right_velocity += velocity_limit*(1.0 - largest_contour_bounding_box[2]/optimal_object_width)
            #after computing the new velocity of the wheels, set the values to the rvr
            self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )


    #face the robot to a color of interest
    def face_color(self):
        self.vision.camera.set_color_filter(120, precision=15)
        while(1):
            mask = self.vision.camera.get_color_mask()
            #If there are less than 10 active pixels
            if(np.sum(mask/255) < 10):
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=10 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            self.vision.pan_tilt_unit.update(avg_point)
            #Now move the robot according to the current angle of the pan unit
            cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556
            proportion = 0.4
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

            #Filter out small robot movements
            if(np.abs(robot_proportion_angle_deg) < 5.0):
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )
            #if the movement is big enough, then move the robot
            else:
                self.rvr.drive_tank_si_units(
                    left_velocity = -robot_proportion_angle_deg/90.0,
                    right_velocity = robot_proportion_angle_deg/90.0
                )

    #class destructor
    def __del__(self):
        #close the rvr
        self.rvr.close()