import RPi.GPIO as GPIO
import time
import signal


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


GPIO.setup(19, GPIO.OUT)

LOW_TIME = 338.3E-6
TOTAL_PERIOD = 20.074E-3
HIGH_TIME_LOW = 550E-6
HIGH_TIME_HIGH = 1040E-6
UNUSED_CHANNEL_TIME = 1000E-6


def CtrlC_handler(signum, frame):
    exit()    
 
    
def main():
    print("Use CTRL+C to kill signal")
    signal.signal(signal.SIGINT, CtrlC_handler)

    CYCLE_START_TIME = TOTAL_PERIOD - 9*LOW_TIME - 4*UNUSED_CHANNEL_TIME - 2*HIGH_TIME_HIGH - 2*HIGH_TIME_LOW
    print(CYCLE_START_TIME * 1000)

    while True:
        # starting pulse
        GPIO.output(19, 1)
        time.sleep(CYCLE_START_TIME)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 1: Roll
        GPIO.output(19, 1)
        time.sleep(HIGH_TIME_HIGH)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 2: Pitch
        GPIO.output(19, 1)
        time.sleep(HIGH_TIME_LOW)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 3: Throttle
        GPIO.output(19, 1)
        time.sleep(HIGH_TIME_HIGH)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 4: Yaw
        GPIO.output(19, 1)
        time.sleep(HIGH_TIME_LOW)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 5: Unused
        GPIO.output(19, 1)
        time.sleep(UNUSED_CHANNEL_TIME)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 6: Unused
        GPIO.output(19, 1)
        time.sleep(UNUSED_CHANNEL_TIME)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 7: Unused
        GPIO.output(19, 1)
        time.sleep(UNUSED_CHANNEL_TIME)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)

        # Pulse 8: Unused
        GPIO.output(19, 1)
        time.sleep(UNUSED_CHANNEL_TIME)
        GPIO.output(19, 0)
        time.sleep(LOW_TIME)
    

if __name__ == "__main__":
    main()

