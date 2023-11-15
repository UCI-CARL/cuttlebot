import time
import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

class Servo():
    #Initiallize the class
    def __init__(self, pin, use_hardware_PWM=False, reset_servo_position=True, max_turn_angle=90):
        #Store class variables
        self.angle = None
        self.timestamp = None
        self.servo = None
        self.pin = pin
        self.use_hardware_pwm = use_hardware_PWM
        self.max_turn_angle = max_turn_angle

        #Check for Hardware PWM
        if(use_hardware_PWM):
            #set the channel for the PWM
            channel = None
            if(pin == 32):
                channel = 0
            elif(pin == 33):
                channel = 1
            else:
                raise Exception("Only Pins 32 or 33 can be used for hardware PWM")
            #Instantiate the servo for the specified pwm channel (0=GPIO12/Pin32, 1=GPIO13/Pin33) at 50Hz
            self.servo = HardwarePWM(pwm_channel=channel, hz=50)
        #Case for Software PWM
        else:
            #Set board for the GPIO
            GPIO.setmode(GPIO.BOARD)
            #Set up the specified pin
            GPIO.setup(pin, GPIO.OUT)
            #Instantiate the servo at the specified pin at 50Hz
            self.servo = GPIO.PWM(pin, 50)

        #Start the servo, but at 0 duty cycle
        self.servo.start(0)
        #reset the servo to its center (0 degrees) if wanted
        if(reset_servo_position):
            self.set_angle_deg(0)
    

    #set the angle of the servo motor and obtain a timestamp for when it was set
    def set_angle_deg(self, angle):
        #Check for errors in the angle bounds and cut them off to the specified max
        if(angle > self.max_turn_angle):
            angle = self.max_turn_angle
        elif(angle < -self.max_turn_angle):
            angle = -self.max_turn_angle
        #Check for change in angle to avoid extra computation 
        if(angle == self.angle):
            return
        #PWM_duty_cycle_pct = 7.5 + (10/180)*angle
        duty_cycle_pct = 7.5 + 0.055556*angle
        #Set the duty cycle for the servo
        if(self.use_hardware_pwm):
            self.servo.change_duty_cycle(duty_cycle_pct)
        else:
            self.servo.ChangeDutyCycle(duty_cycle_pct)
        #Update the object data members
        self.angle = angle
        self.timestamp = time.time()

    #get the angle that the servo was set to
    def get_angle_deg(self):
        return(self.angle)

    #get the timestamp for when the servo was last set
    def get_timestamp_sec(self):
        return(self.timestamp)

    #get the time that has passed from the last timestamp
    def get_time_passed_from_last_timestamp_sec(self):
        return(time.time() - self.timestamp)
    
    #Destructor For servo to stop the PWM upon destruction
    def __del__(self):
        self.servo.stop()
        if(not self.use_hardware_pwm):
            GPIO.cleanup(self.pin)