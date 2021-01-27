import RPi.GPIO as GPIO
import time
import signal


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Switch
GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Traffic Light 1
GPIO.setup(4, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

# Traffic Light 2
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

# Panel
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

SWITCH = 19

BLUE_1 = 4
GREEN_1 = 17
RED_1 = 27

BLUE_2 = 5
GREEN_2 = 6
RED_2 = 13

PANEL_SEGS = {
    1:25,
    2:24,
    3:12,
    4:23,
    5:16,
    6:21,
    7:20
    }


def CtrlC_handler(signum, frame):
    GPIO.output(RED_1, 0)
    GPIO.output(GREEN_1, 0)
    GPIO.output(BLUE_1, 0)
    GPIO.output(RED_2, 0)
    GPIO.output(GREEN_2, 0)
    GPIO.output(BLUE_2, 0)
    GPIO.output(PANEL_SEGS[1], 0)
    GPIO.output(PANEL_SEGS[2], 0)
    GPIO.output(PANEL_SEGS[3], 0)
    GPIO.output(PANEL_SEGS[4], 0)
    GPIO.output(PANEL_SEGS[5], 0)
    GPIO.output(PANEL_SEGS[6], 0)
    GPIO.output(PANEL_SEGS[7], 0)
    exit()
    
def panelNumChange(num):
    GPIO.output(PANEL_SEGS[1], 0)
    GPIO.output(PANEL_SEGS[2], 0)
    GPIO.output(PANEL_SEGS[3], 0)
    GPIO.output(PANEL_SEGS[4], 0)
    GPIO.output(PANEL_SEGS[5], 0)
    GPIO.output(PANEL_SEGS[6], 0)
    GPIO.output(PANEL_SEGS[7], 0)
                                      
    if num == 1:
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[6], 1)
    elif num == 2:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[5], 1)
        GPIO.output(PANEL_SEGS[7], 1)
    elif num == 3:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[6], 1)
        GPIO.output(PANEL_SEGS[7], 1)
    elif num == 4:
        GPIO.output(PANEL_SEGS[2], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[6], 1)
    elif num == 5:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[2], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[6], 1)
        GPIO.output(PANEL_SEGS[7], 1)
    elif num == 6:
        GPIO.output(PANEL_SEGS[2], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[5], 1)
        GPIO.output(PANEL_SEGS[6], 1)
        GPIO.output(PANEL_SEGS[7], 1)
    elif num == 7:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[6], 1)
    elif num == 8:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[2], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[5], 1)
        GPIO.output(PANEL_SEGS[6], 1)
        GPIO.output(PANEL_SEGS[7], 1)
    elif num == 9:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[2], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[4], 1)
        GPIO.output(PANEL_SEGS[6], 1)
    elif num == 0:
        GPIO.output(PANEL_SEGS[1], 1)
        GPIO.output(PANEL_SEGS[2], 1)
        GPIO.output(PANEL_SEGS[3], 1)
        GPIO.output(PANEL_SEGS[5], 1)
        GPIO.output(PANEL_SEGS[6], 1)
        GPIO.output(PANEL_SEGS[7], 1)
        

def main():
    signal.signal(signal.SIGINT, CtrlC_handler)
    firstLoop = True
    
    while True:
        if firstLoop:
            GPIO.output(RED_1, 0)
            
        GPIO.output(GREEN_1, 0)
        GPIO.output(BLUE_1, 0)
        GPIO.output(RED_2, 0)
        GPIO.output(GREEN_2, 0)
        GPIO.output(BLUE_2, 0)
        GPIO.output(PANEL_SEGS[1], 0)
        GPIO.output(PANEL_SEGS[2], 0)
        GPIO.output(PANEL_SEGS[3], 0)
        GPIO.output(PANEL_SEGS[4], 0)
        GPIO.output(PANEL_SEGS[5], 0)
        GPIO.output(PANEL_SEGS[6], 0)
        GPIO.output(PANEL_SEGS[7], 0)
        time.sleep(.01)
            
        # 1 - Button not yet pressed. Traffic Light 2 is Green
        GPIO.output(GREEN_2, 1)
        
        # implementing 20 second limit
        if not firstLoop:
            time.sleep(7)
        else:
            firstLoop = False
            
        while not GPIO.input(SWITCH):
            pass
            
        # 2 - Button pressed. Traffic Light 2 blinks blue 3 times then turns red
        GPIO.output(GREEN_2, 0)
        time.sleep(.01)
        for i in range(3):
            GPIO.output(BLUE_2, 1)
            time.sleep(0.1)
            GPIO.output(BLUE_2, 0)
            time.sleep(1)
            
        GPIO.output(RED_2, 1)
        
        # 3 - Traffic Light 2 turns red. Traffic Light 1 turns green then countdown
        # from 9 to 0 begins in seconds
        GPIO.output(GREEN_1, 1)
        GPIO.output(RED_1, 0)
        for i in reversed(range(10)):
            # 4 - Countdown reaches 4. Traffic Light 1 flashes blue until time 0
            if i == 0:
                GPIO.output(GREEN_1, 0)
                time.sleep(.01)
                
                GPIO.output(RED_1, 1)
                GPIO.output(GREEN_2, 1)
                panelNumChange(i)
                time.sleep(1)
            elif i <= 4:
                print
                GPIO.output(GREEN_1, 0)
                time.sleep(.01)
                
                panelNumChange(i)
                
                GPIO.output(BLUE_1, 1)
                time.sleep(0.1)
                GPIO.output(BLUE_1, 0)
                time.sleep(1)
            else:
                panelNumChange(i)
                time.sleep(1)
        
        
        # 5 - Countdown reaches 0. Traffic Light 1 turns ref and Traffic Light 2 turns green
        GPIO.output(RED_1, 1)
        GPIO.output(GREEN_2, 1)
    

if __name__ == "__main__":
    main()
