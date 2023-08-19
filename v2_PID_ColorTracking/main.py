#Import needed libraries
import Vision.Perception

#Main function
def main():
    #instantiate vision object
    vision = Vision.Perception()
    #use vision object to track color
    vision.track_color(0, precision=15)

#Condition to call main only when file is directly excuted
if(__name__ == "__main__"):
    main()