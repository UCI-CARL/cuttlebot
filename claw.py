#Test for working with the claw
###################
#ADC stuff for the claw not working - need to use the IC's library
###################

#import libraries
import RPi.GPIO as GPIO
import spidev
import time

#set GPIO numbering mode (left-right;top-down numbering starting from 1)
GPIO.setmode(GPIO.BOARD)

#Set pin 11 as an output pin
servo_pin = 11
led_pin = 13
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(led_pin, GPIO.OUT)

#set PWM to 50Hz (servo expects 50Hz PWM to read from: 20 ms per pulse)
servo = GPIO.PWM(servo_pin, 50)

#Create a spidev object for the ADC
spi = spidev.SpiDev()

#Now open the spidev object for the adc
delay_ms = 0
channel = 0
spi.open(delay_ms, channel)

#create a function to read from the openned spi object
#code referenced from: https://pimylifeup.com/raspberry-pi-adc/
def read_adc(adc_channel):
   if(adc_channel > 7 or adc_channel < 0):
      return(-1)
   r = spi.xfer2([1, 8+adc_channel << 4, 0])
   data = ((r[1] & 3) << 8) + r[2]
   return(data)

#start running the PWM (with pulse off)
servo.start(0)

#set the duty cycle for the PWM (0.5ms = 0deg = 2.5% and 2.5ms = 180deg = 12.5%)
try:
   prev_current = None
   angle = 45
   while(True):
      #linear relationship between angle and duty cycle
      duty_cycle = 2.5 + angle/18
      servo.ChangeDutyCycle(duty_cycle)
      for i in range(200):
         time.sleep(0.01)
         rate_of_change = 0
         measured_current = (5.0 - 5.0*(read_adc(adc_channel=0)/1024.0))/1.5
         if(prev_current != None):
            #rate of change in amps per second
            rate_of_change = (measured_current - prev_current)/0.01
         prev_current = measured_current
         print(rate_of_change)
         if(rate_of_change > 0.01):
            print("YES YES YES")
            GPIO.output(led_pin, GPIO.HIGH)
            time.sleep(3)
            GPIO.output(led_pin, GPIO.LOW)
      if(angle == 45):
         angle = 90
      else:
         angle = 45
finally:
   #Clean up things at the end
   servo.stop()
   GPIO.cleanup()
