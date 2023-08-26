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
        self.camera = Camera()
        self.pan_tilt_unit = PanTiltUnit()

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
            mask_center = (np.array(mask.shape)-1)/2
            avg_point = self.__get_avg_mask_point(mask, relative_pixel=mask_center)
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            self.pan_tilt_unit.update(avg_point)
            #Option to present the image of the camera
            image = cv2.circle(self.camera.get_image(), avg_point+mask_center, radius=5, color=(0,0,255), thickness=3)
            cv2.imshow("Camera Image", image)
            #Press Esc to stop program
            if(cv2.waitKey(1)&0xFF == 27):
                break
        #destroy all open cv windows upon completion of the program
        cv2.destroyAllWindows()

    #Helper function to get the average pixel of a given mask
    def __get_avg_mask_point(self, mask, relative_pixel=(0,0)):
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
        #Get the average point for the mask
        avg_point = (avg_row, avg_column)
        #Then compute the average point relative to some pixel
        avg_point -= relative_pixel
        return(avg_point)