from Robot import Robot

robot = Robot()

#robot.do_nothing() #do nothing

#robot.test_move_and_grab()

#robot.camera_mode() #show robot camera POV
#robot.test_claw()
#robot.get_new_calibration_images(camera_ID=0) #take calibration images for the specified camera
#robot.get_object_depth_info_forward_to_back() #take pic, move forward, take pic, calculate distance
robot.move_to_color() #follow a red object, maintaining a distance proportional to the size of the object
#robot.face_color()