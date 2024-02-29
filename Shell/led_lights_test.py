# libraries
import time
import board
import neopixel

# setting up the led
led = neopixel.NeoPixel(board.D12, 70, pixel_order=neopixel.GRB, brightness=0.1)

x = 0
wave_frequency = 0.2
blue = (98, 144, 220)
black = (39, 39, 41) #not really black but this is for the blue/black wave

home = (138, 43, 226)

row_1 = 0
row_2 = 10
row_3 = 20
row_4 = 30
row_5 = 40
row_6 = 50
row_7 = 60
row_8 = 70

# keyboard input to determine what to do
command = input("Pick either 1 (hypnotize) or 2 (camouflage): ")

def hypnotize():
	led.fill(blue)
	for x in range(row_1, row_2):
		led[x] = black
		
	time.sleep(wave_frequency)
	
	led.fill(blue)
	for x in range(row_2, row_3):
		led[x] = black
		
	time.sleep(wave_frequency)
	
	led.fill(blue)
	for x in range(row_3, row_4):
		led[x] = black
		
	time.sleep(wave_frequency)
	
	led.fill(blue)
	for x in range(row_4, row_5):
		led[x] = black
	
	time.sleep(wave_frequency)
	
	led.fill(blue)
	for x in range(row_5, row_6):
		led[x] = black
	
	time.sleep(wave_frequency)
	
	led.fill(blue)
	for x in range(row_6, row_7):
		led[x] = black
	
	time.sleep(wave_frequency)
	
	led.fill(blue)
	for x in range(row_7, row_8):
		led[x] = black
		
def camouflage():
	led.fill(home)
	time.sleep(10)


# set the color of the entire led strip
while True:
	#led.fill((98, 144, 220))
	
	if command == '1':
		hypnotize()
		
	elif command == '2':
		R = int(input("Pick R value: "))
		G = int(input("Pick G value: "))
		B = int(input("Pick B value: "))
		home = (R, G, B)
		camouflage()
		
	command = input("Pick either 1 (hypnotize) or 2 (camouflage): ")
	
	


time.sleep(1)



"""
# simple version of doing a wave
while True:
 while x < 16:
 	led[x] = (98, 144, 220)
 	led[x+1] = (0, 0, 0)
 	
 	x = x + 1
 	time.sleep(0.05)
 	
 while x > 3:
 	led[x] = (98, 144, 220)
 	led[x-1] = (0, 0, 0)
 	
 	x = x - 1
 	time.sleep(0.05)
 	
 while x < 16:
 	led[x] = (98, 144, 220)
 	led[x+1] = (0, 0, 0)
 	
 	x = x + 1
 	time.sleep(0.05)
 	
 while x > 3:
 	led[x] = (98, 144, 220)
 	led[x-3] = (0, 0, 0)
 	
 	x = x - 1
 	time.sleep(0.05)
"""

"""
x = 0
is_towards_end = True

while True:
	led.fill((0, 0, 0))
	led[x] = (0, 0, 0)
	time.sleep(0.05)
	
	if(is_towards_end):
		x += 1
		if(x == 16):
			is_towards_end = False
	else:
		x -= 1
		if(x == -1):
			is_towards_end = True
"""		
	
	
time.sleep(3)

# turn off all the leds
led.fill((0, 0, 0))