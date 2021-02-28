import os
import signal
import time as t
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

def sixteen_bit_to_voltage(sixteen_bit_val):
    return sixteen_bit_val * (3.3 / 65535)
    

def plot(xdata, ydata):
    plt.plot(xdata, ydata)
    plt.show()


def main():
    step = 0
    
    # take 20000 data points
    while True:
        if step > 20000:
            break
            
        times.append(step)
        voltages.append(sixteen_bit_to_voltage(chan0.value))
        
        step += 1
        
        #sample rate is approximately 10000 points per second
        t.sleep(0.0001)
        
    plot(times, voltages)
    
    
if __name__ == "__main__":
    main()
