import RPi.GPIO as GPIO
import time
import signal


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


GPIO.setup(23, GPIO.OUT)

LOW = 338.3E-6
TOTAL_PERIOD = 20.074E-3
UNUSED_CHANNEL = 1000E-6

ROLL = 1000E-6
PITCH = 1000E-6
THROTTLE = 400E-6
YAW = 1000E-6


def CtrlC_handler(signum, frame):
    exit()    
 
    
def main():
    print("Use CTRL+C to kill signal")
    signal.signal(signal.SIGINT, CtrlC_handler)
    FIRST = TOTAL_PERIOD - 9*LOW - 4*UNUSED_CHANNEL - ROLL - PITCH - THROTTLE - YAW
    print(FIRST)

    while True:
        # starting pulse
        GPIO.output(23, 1)
        time.sleep(FIRST)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 1: Roll
        GPIO.output(23, 1)
        time.sleep(ROLL)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 2: Pitch
        GPIO.output(23, 1)
        time.sleep(PITCH)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 3: Throttle
        GPIO.output(23, 1)
        time.sleep(THROTTLE)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 4: Yaw
        GPIO.output(23, 1)
        time.sleep(YAW)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 5: Unused
        GPIO.output(23, 1)
        time.sleep(UNUSED_CHANNEL)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 6: Unused
        GPIO.output(23, 1)
        time.sleep(UNUSED_CHANNEL)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 7: Unused
        GPIO.output(23, 1)
        time.sleep(UNUSED_CHANNEL)
        GPIO.output(23, 0)
        time.sleep(LOW)

        # Pulse 8: Unused
        GPIO.output(23, 1)
        time.sleep(UNUSED_CHANNEL)
        GPIO.output(23, 0)
        time.sleep(LOW)
    

if __name__ == "__main__":
    main()

