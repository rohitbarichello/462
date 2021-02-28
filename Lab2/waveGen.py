import time
import math
import RPi.GPIO as GPIO
import Adafruit_MCP4725
import signal

dac = Adafruit_MCP4725.MCP4725()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
BUTTON = 21

def handleButtonPress(inputpin):
	firstLoop = True

	while True:
		if firstLoop:
			print("\n***Complete these fields to generate a waveform***")
			firstLoop = False
		else:
			print("\n***Try again with numbers in range***")
		
		shape = input("Shape(Enter 1 for square, 2 for triangle, and 3 for sin): ")
		frequency = float(input("Frequency(Max 20 Hz): "))
		max_output_v = float(input("Maximum output voltage(Max 5 V): "))
		
		if (shape == 1 or shape == 2 or shape == 3) and (0 < frequency <= 20) and (0 < max_output_v <= 5): 
			firstLoop = True
			if shape == 1:
				square_wave(frequency, max_output_v)
			elif shape == 2:
				triangle_wave(frequency, max_output_v)
			elif shape == 3:
				sine_wave(frequency, max_output_v)

def sine_wave(FREQUENCY, MAX_OUTPUT_V):
	t = 0.0
	tStep = 0.05
	
	while True:
		if GPIO.input(BUTTON):
			break
		#voltage = (4096)*((MAX_OUTPUT_V / 5.0) * math.sin((FREQUENCY)*t))
		voltage = (2048)*(1.0 + (MAX_OUTPUT_V / 5.0) * math.sin((FREQUENCY)*t))
		dac.set_voltage(int(voltage))
		
		t += tStep
		time.sleep(0.0005)
		
def square_wave(FREQUENCY, MAX_OUTPUT_V):
	while True:
		if GPIO.input(BUTTON):
			break
		dac.set_voltage(int(4096*(MAX_OUTPUT_V / 5.0)))
		time.sleep((1/FREQUENCY) / 2)
		dac.set_voltage(-int(4096*(MAX_OUTPUT_V / 5.0)))
		time.sleep((1/FREQUENCY) / 2)
				
def triangle_wave(FREQUENCY, MAX_OUTPUT_V):
	t = 0.0
	tStep = 0.05
	m = (1/FREQUENCY) / 2
	
	while True:
		if GPIO.input(BUTTON):
			break
		voltage = 4096 * (((MAX_OUTPUT_V / m) * (m - abs(t % (2*m) - m)) ) / 5.0)
		dac.set_voltage(int(voltage))
		
		t += tStep
		time.sleep(0.0005)
		
def CtrlC_handler(signum, frame):
    print("\n\nSeeya")
    exit()
		
def main():
	print("Use CTRL+C to exit. Press the button to choose a waveform")
	GPIO.add_event_detect(BUTTON, GPIO.FALLING, handleButtonPress)
	signal.signal(signal.SIGINT, CtrlC_handler)
	
	time.sleep(100000)



if __name__ == "__main__":
    main() 
