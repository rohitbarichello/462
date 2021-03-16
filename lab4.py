import time as t
import signal
import matplotlib.pyplot as plt

import smbus			#import SMBus module of I2C
from time import sleep          #import

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

read_from_MPU = TRUE


def CtrlC_handler(signum, frame):
    read_from_MPU = False

				
def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
    
    
def plot(xdata, ydata, zdata, elapsed_time):
    plt.plot(xdata, elapsed_time)
    plt.plot(ydata, elapsed_time)
    plt.plot(zdata, elapsed_time)
    
    plt.show()
    
    
def main():
    MPU_Init()
    signal.signal(signal.SIGINT, CtrlC_handler)
    
    xdata = []
    ydata = []
    zdata = []

    countdown = 5

    print("Get ready to step")
    while countdown > 0:
	print(countdown - 1)
        sleep(1)
    
    elapsed_time = 0
    start_time = t.time()
	
    print (" Reading Data of Gyroscope and Accelerometer")
    print("Use CTRL+C to end data collection and plot collected data")

    while read_from_MPU:
	#Keep track of elapsed time
	elapsed_time = t.time() - start_time
	
	#Read data from MPU
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
	
	#prepare data for plotting     
	xdata.append(acc_x)
	ydata.append(acc_y)
	ydata.append(acc_z)
	elapsed_time.append(elapsed_time)

        #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
        sleep(0.1)
    
    plot(xdata, ydata, zdata, elapsed_time)
    
    
if __name__ == "__main__":
    main()










