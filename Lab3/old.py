import os
import signal
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import matplotlib.pyplot as plt

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

voltages = []
times = []
peak_times = []

def sixteen_bit_to_voltage(sixteen_bit_val):
    return sixteen_bit_val * (3.3 / 65535)
    

def plot(xdata, ydata):
    plt.plot(xdata, ydata)
    plt.show()


def main():
    start_time = time.time()
    total_time_passed = 0.0
    
    #while total_time_passed <= 0.1:
    for i in range(10):
        total_time_passed = time.time() - total_time_passed
        
        times.append(total_time_passed)
        voltages.append(sixteen_bit_to_voltage(chan0.value))
        
        print(chan0.value)
        
        time.sleep(0.01)
        
    plot(times, voltages)
    
    
if __name__ == "__main__":
    main()
