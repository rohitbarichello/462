import time as t
import signal
import matplotlib.pyplot as plt
import numpy as np

import smbus  # import SMBus module of I2C
from time import sleep  # import

# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address


def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value


def plot(data, time):
    #plt.plot(timedata, xdata)
    #plt.plot(timedata, ydata)
    #plt.plot(timedata, zdata)
    plt.plot(data, time)

    plt.show()


def main():
    MPU_Init()

    xdata = []
    ydata = []
    zdata = []
    timedata = []

    # s is the vector sum of all three readings, x, y, and z
    sdata = []

    # capturing filtered s data
    window = []
    filteredSData = []
    filteredSDataTime = []

    # capturing smoothed data (using finite difference)
    smoothedSData = []
    smoothedSDataTime = []

    pointCount = 0
    countdown = 5

    # countdown to program execution
    print("Get ready to step")
    while countdown > 0:
        print(countdown)
        sleep(1)

        countdown = countdown - 1

    # prepare to capture elapsed time
    elapsed_time = 0
    start_time = t.time()

    # Start reading data
    print(" Reading Data of Gyroscope and Accelerometer")
    while elapsed_time <= 10:
        # Keep track of elapsed time
        elapsed_time = t.time() - start_time
        pointCount += 1

        # Read data from MPU
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        As = abs(Ax) + abs(Ay) + abs(Az)

        # prepare data for plotting
        xdata.append(Ax)
        ydata.append(Ay)
        zdata.append(Az)
        sdata.append(As)
        window.append(As)
        timedata.append(elapsed_time)

        # filter data by taking only the max acceleration vector sum of each 10 point window
        if pointCount % 10 == 0:
            filteredSData.append(max(window))
            filteredSDataTime.append(elapsed_time)
            window = []

            # take the finite difference of each filtered point to smooth the data
            if len(filteredSData) > 3:
                smoothedSData.append(
                    (filteredSData[-1] + filteredData[-3]) / 2)
                smoothedSDataTime.append(elapsed_time)

        # print collected data
        #print ("\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)

        sleep(0.001)

    # plot collected data
    plot(smoothedSData, smoothedSDataTime)


if __name__ == "__main__":
    main()
