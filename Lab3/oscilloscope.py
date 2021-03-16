import time as t
from gpiozero import MCP3008
import matplotlib.pyplot as plt


def plot(xdata, ydata):
    plt.plot(xdata, ydata)
    plt.show()
    
    
def frequency(zero_times):
	period = zero_times[3] - zero_times[2]
	
	return 1 / period
	
	
def identifyWaveForm(frequency, zero_indices, voltages):
	waveType = ''
	next_zero_index = int((zero_indices[3] + zero_indices[2]) / 2)
	peak_index = int((zero_indices[3] + next_zero_index) / 2)
	index_between_peak_and_zero2 = int((zero_indices[3] + peak_index) / 2)
	
	slope1 = (voltages[peak_index]/2) / (index_between_peak_and_zero2 - peak_index)
	slope2 = (voltages[index_between_peak_and_zero2]) / (zero_indices[3] - index_between_peak_and_zero2)
	slopeDiff = slope2 - slope1
		
	if abs(voltages[index_between_peak_and_zero2] - voltages[peak_index]) < 0.001:
		waveType = "Square wave"
	elif abs(slopeDiff) < 0.001:
		waveType = "Triangle wave"
	else:
		waveType = "Sin wave"
	
	
	print(f'{waveType} of frequency {frequency} Hz')
	
	
def main():
	wave = MCP3008(channel = 0, clock_pin=21, mosi_pin=20, miso_pin=16, select_pin=12)
	
	voltages = []
	times = []
	zero_times = []
	zero_indices = []

	elapsed_time = 0
	start_time = t.time()
	
	checking_for_zero = True

	while elapsed_time <= 5:
		elapsed_time = t.time() - start_time
		voltage = wave.value * 3.3

		times.append(elapsed_time)
		voltages.append(voltage)

		if checking_for_zero and voltage < 0.01:
			zero_times.append(elapsed_time)
			zero_indices.append(len(voltages) - 1)
			checking_for_zero = False
			
		if len(voltages) > 3 and not checking_for_zero and voltage > voltages[-2]:
			checking_for_zero = True
		
	plot(times, voltages)

	identifyWaveForm(round(frequency(zero_times), 3), zero_indices, voltages)
    
    
if __name__ == "__main__":
    main()
