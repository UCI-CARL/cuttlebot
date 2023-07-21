#Working with sphero rvr

#import necessary libraries
import os
import sys
import time

#obtain the global path to the sphero sdk directory
current_file_path = os.path.dirname(__file__)
sdk_local_path = os.path.join(current_file_path, "sphero-sdk-raspberrypi-python")
sdk_absolute_path = os.path.abspath(sdk_local_path)
#set the sphero sdk directory to PATH
sys.path.append(sdk_absolute_path)

#import the sphero library
import sphero_sdk as sphero

#instantiate the robot
rvr = sphero.SpheroRvrObserver()

#main function
def main():
    try:
        #wake up the rvr
        rvr.wake()
        #give some time to wake up and set robot
        time.sleep(10)

        #reset yaw
        rvr.reset_yaw()

        #drive forward
        rvr.drive_with_heading(speed=128, heading=0, flags=sphero.DriveFlagsBitmask.none.value)
        #drive forward for 1 second
        time.sleep(1)
        #Stop moving
        rvr.drive_stop()
        #stop for 1 second
        time.sleep(1)

        #drive backward
        rvr.drive_with_heading(speed=128, heading=0, flags=sphero.DriveFlagsBitmask.drive_reve>
        #drive backward for 1 second
        time.sleep(1)
        #Stop moving
        rvr.drive_stop()
        #stop for 1 second
        time.sleep(1)

        #turn around and drive
        rvr.drive_with_heading(speed=128, heading=180, flags=sphero.DriveFlagsBitmask.none.val>
        #turn around and drive for 2 second
        time.sleep(2)
        #Stop moving
        rvr.drive_stop()
        #stop for 1 second
        time.sleep(1)

        #turn to 90 degrees and start driving
        rvr.drive_with_heading(speed=64, heading=270, flags=sphero.DriveFlagsBitmask.none.valu>
        #turn turn and drive for 3 seconds
        time.sleep(3)
        #Stop moving
        rvr.drive_stop()
        #stop for 1 second
        time.sleep(1)

        #drive backward
        rvr.drive_with_heading(speed=128, heading=270, flags=sphero.DriveFlagsBitmask.drive_re>
        #drive backward for 1 second
        time.sleep(1)
        #Stop moving
        rvr.drive_stop()
        #stop for 1 second
        time.sleep(1)

    #stop on keyboard interrupt
    except KeyboardInterrupt:
        print("\nProgram terminated with keyboard interrupt.")

    #Last, close the rvr object
    finally:
        rvr.close()

#run main function if file executed directly
if(__name__ == "__main__"):
   main()