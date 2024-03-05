#import needed libraries
import numpy as np
import cv2
from Vision.Sight.Camera import Camera
from Vision.Movement.PanTiltUnit import PanTiltUnit

#Perception class for using sensation and movement to perform perceptual abilities
class Perception():
    #Vision class constructor
    def __init__(self):
        #Intantiate the camera and pan tilt unit objects
        self.camera = Camera(camera_ID=None)#self.camera = Camera(camera_ID=0) #NEED TO SOLVE ISSUES CAMERA CALIBRATION
        self.pan_tilt_unit = PanTiltUnit()

    #Determine if the defined colors passed to the function are in view of the camera
    def get_colors_in_view(self, color_dict, precision=15):
        #initiallize a list to store the colored objects that appear in the image
        colors_in_view = list()
        #get the current view of the robot
        image = self.camera.get_image()
        #loop through each color in the color dictionary
        for color in color_dict.keys():
            #set the color filter and get the mask in the picture
            self.camera.set_color_filter(color_dict[color], precision)
            color_mask = self.camera.color_filter.get_color_filter_mask(image)
            #condition 1: the mask must have possibly found an object (more than 20 active pixels)
            if(np.sum(color_mask/255) < 20):
                continue
            #get the width of the bounding box of the largest countour
            largest_contour = self.get_largest_contour(color_mask)
            largest_contour_bounding_box = self.get_contour_bounding_box(largest_contour)
            pixel_width = largest_contour_bounding_box[2]
            #condition 2: the object with have a width no less than 10 pixels (reduces chance of contour being random noise)
            if(pixel_width >= 10):
                colors_in_view.append(color)
        #return the list of colors in view
        return(colors_in_view)
    

    #Method to track a certain color in the camera's view
    def track_color(self, hue, precision=15):
        self.camera.set_color_filter(hue, precision)
        while(1):
            #Get the current view of the camera (feedback for the controller guiding the movement)
            mask = self.camera.get_color_mask()
            #Check to ensure a valid image was recieved (at least 10 or more pixels in the specified color range)
            if(np.sum(mask) < 10):
                cv2.imshow("Camera Image", self.camera.get_image())
                #Press Esc to stop program
                if(cv2.waitKey(1)&0xFF == 27):
                    break
                continue
            #Get the average point of the mask (relative to the center of the camera)
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            avg_point = self.get_avg_mask_point(mask, relative_point=mask_center)
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            self.pan_tilt_unit.update(avg_point)
            #Option to present the image of the camera
            image = cv2.circle(self.camera.get_image(), tuple([int(i) for i in avg_point+mask_center]), radius=5, color=(0,0,255), thickness=3)
            cv2.imshow("Camera Image", image)
            #Press Esc to stop program
            if(cv2.waitKey(1)&0xFF == 27):
                break
        #destroy all open cv windows upon completion of the program
        cv2.destroyAllWindows()

    #Helper function to get the average pixel of a given mask
    def get_avg_mask_point(self, mask, relative_point=np.array([0,0])):
        #Get the dimensions of the mask
        (num_rows, num_cols) = mask.shape
        #Get the weights for the rows and columns
        row_weights = np.array(range(num_rows))
        column_weights = np.array(range(num_cols))
        #Get the multipliers for the rows and columns (i.e., how many mask points are in each row/column)
        row_multipliers = np.sum(mask, axis=1)
        column_multipliers = np.sum(mask, axis=0)
        #Compute the average row and column based on the weights and multipliers
        avg_row = np.sum(row_weights*row_multipliers)/np.sum(row_multipliers)
        avg_column = np.sum(column_weights*column_multipliers)/np.sum(column_multipliers)
        #Get the average point (x,y) for the mask
        avg_point = (avg_column, avg_row)
        #Then compute the average point relative to some pixel
        avg_point -= relative_point
        return(avg_point)

    #return the absolute point with measurementsthat are relative to the relative point
    def get_relative_position(self, absolute_point, relative_point=np.array([0,0])):
        #point will be the absolute point minus the relative point (relative point is the reference)
        return(absolute_point - relative_point)

    #return the largest contour from a given mask
    #modified from: https://stackoverflow.com/questions/44588279/find-and-draw-the-largest-contour-in-opencv-on-a-specific-color-python
    def get_largest_contour(self, mask):
        #first blur the image so that there are no errors with getting the contour
        blurred_mask = cv2.blur(mask, (15,15))
        #then get the threshold above 50% brightness
        ret, thresh = cv2.threshold(blurred_mask, 127, 255, 0)
        #next, find all the contours in the image
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #last, if a contour exist, find the largest one
        if(len(contours) > 0):
            return(max(contours, key = cv2.contourArea))
        #if no contours found, return none
        return(None)

    #return the bounding box for the specificed contour
    def get_contour_bounding_box(self, contour):
        #return bounding box in x,y,w,h form
        return(cv2.boundingRect(contour))

    #get the centroid for the bounding box
    def get_contour_bounding_box_centroid(self, contour_bounding_box):
        #compute the centroids in the x and y direction
        x_center = np.round(contour_bounding_box[0] + contour_bounding_box[2]/2)
        y_center = np.round(contour_bounding_box[1] + contour_bounding_box[3]/2)
        return(np.array([x_center, y_center]))

    #get the depth of an object from two different temporal potitions
    def get_temporal_difference_object_depth(self, p_length1, p_length2, distance_moved_radially_inward):
        #ref1_angle = p_length1/self.camera.get_max_dimension_pxl(axis_focus_dwh) * self.camera.get_FOV_rad(axis_focus_dwh)/2
        #ref2_angle = p_length2/self.camera.get_max_dimension_pxl(axis_focus_dwh) * self.camera.get_FOV_rad(axis_focus_dwh)/2
        #numerator = distance_moved_radially_inward_m * np.tan(ref1_angle)
        #denominator = np.tan(ref2_angle) - np.tan(ref1_angle)
        #depth_m = numerator / denominator
        depth = distance_moved_radially_inward*(p_length1/(p_length2 - p_length1))
        return(depth)
