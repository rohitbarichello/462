import RPi.GPIO as GPIO
import time
import signal
from cv2 import cv2
import numpy as np
import math
import threading
from queue import Queue


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(23, GPIO.OUT)

TOTAL_PERIOD = 20.074E-3

LOW = 338.3E-6
UNUSED_CHANNEL = 950E-6
YAW = 950E-6
PITCH = 950E-6

ASCENT_HAND_RATIO = 12
DESCENT_HAND_RATIO = 5


def initiate_descent(CURRENT_THROTTLE):
    print("Descending")

    FINAL_THROTTLE = CURRENT_THROTTLE

    roll_q.put(950E-6)

    step = (CURRENT_THROTTLE - 400E-6 ) / 200
    
    for i in range(200):    
        FINAL_THROTTLE -= step

        throttle_q.put(FINAL_THROTTLE)
        
        time.sleep(0.01)

    throttle_q.put(0)

    print("Seeya")
    exit()

def camera(roll_q, throttle_q):
    print("Camera thread")
    capture = cv2.VideoCapture(0)

    ROLL = 950E-6
    THROTTLE = 400E-6

    airborne = False

    while True:   
        try:   
            # each video frame stored in "frame". The frame is 640x480 px
            ret, frame = capture.read()

            frame = cv2.flip(frame, 1)

            # define region of interest. interested in the box specified
            # first set is y bounds second set is x bounds
            x = 200
            y = 100
            x1 = x
            x2 = 640 - x
            y1 = y
            y2 = 480 - y
            
            roi=frame[y1:y2, x1:x2]
            
            # draw a green rectangle in region of interest
            # first set is x coordinate and second set is y coordinate
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),0)    

            # convert region of interest, roi, to hsv color space, which is useful for skin color tracking
            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # define range of skin color in HSV. lower_skin is lightest skin value and upper_skin is darkest
            skin_lower_bound = np.array([0,20,70], dtype=np.uint8) # Base: 0, 20, 70
            skin_upper_bound = np.array([20,255,255], dtype=np.uint8) # Base: 20, 255, 255
            
            # extract all pixels that are skin color
            # inRange() returns an array based on the source array, where any pixel in the range is given a 0
            # and any pixel out of the range is given a 225
            mask = cv2.inRange(roi_hsv, skin_lower_bound, skin_upper_bound)
            
            # extrapolate the hand to fill dark spots within
            # We're essentially dilating each pixel to cover the black space in the image. this is done "iterations" times
            # kernel is a 3x3 array of 1's or data type uint8
            kernel = np.ones((2, 2),np.uint8) 
            mask = cv2.dilate(mask, kernel, iterations = 3) 
            
            mask = cv2.GaussianBlur(mask,(5,5),100) 

            # return image contours
            contours, hierarchies = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                contour = max(contours, key = lambda x: cv2.contourArea(x))

                epsilon = 0.0005*cv2.arcLength(contour,True)
                approx= cv2.approxPolyDP(contour,epsilon,True)
                cv2.drawContours(roi, [contour], 0, (0,255,0), 1)
                
                # create a convex hull around the hand.
                hull = cv2.convexHull(contour)
                cv2.drawContours(roi, [hull], 0, (0,255,0), 3)
                
                # get area of hull and contour
                hull_area = cv2.contourArea(hull)
                contour_area = cv2.contourArea(contour)

                # find the centroid of the convex hull. this will be tracked to map to ppms
                hull_moments = cv2.moments(hull)
                centroid_x = int(hull_moments["m10"] / hull_moments["m00"])
                centroid_y = int(hull_moments["m01"] / hull_moments["m00"])
                cv2.circle(roi, (centroid_x, centroid_y), 5, (255, 0, 0))
                            
                cv2.imshow('mask',mask)
                cv2.imshow('frame',frame)

                # this is the percentage of the region of interest that's covered by the hand
                hand_to_roi_ratio = (contour_area / 67200) * 100 

                # if the flight controller is airborne and the hand is removed from the rio, land the quadcopter
                # otherwise, this is the beginning of program execution, so set airborne to true
                if airborne:
                    if hand_to_roi_ratio < DESCENT_HAND_RATIO:
                        initiate_descent(THROTTLE)
                else:                   
                    if hand_to_roi_ratio > ASCENT_HAND_RATIO:
                        airborne = True

                # Start ppm calculations
                centroid_x_trans = centroid_x - 160
                centroid_y_trans = -1 * (centroid_y - 120)

                percent_deviation_x = centroid_x_trans / 120
                percent_deviation_y = centroid_y_trans / 160

                x_factor = (1 + percent_deviation_x)
                y_factor = (1 + percent_deviation_y)

                # the hardcoded numbers, 950 and 400 are the pules widths neccessary to signal to the filght controller
                # that roll and throttle are idle, i.e the drone shouldn't move up or horizontally.
                # we multiply those values by the centroid's deviation from the center of the region of interest
                # to decide by how much we should signal the drone to move up down left or right.
                ROLL = 950E-6 * x_factor
                THROTTLE = 400E-6 * y_factor

                # set ppm parameters
                roll_q.put(ROLL)
                throttle_q.put(THROTTLE)
                    
                # exit image recognition if "d" is entered from the keyboard
                if cv2.waitKey(20) & 0xFF == ord('d'):
                    break

        except ValueError as e:
            print(e)
        

    cv2.destroyAllWindows()
    capture.release()    


# function to send ppm signals to quad via gpio
def ppm(roll_q, throttle_q): 
    print("ppm thread")

    ROLL = 950E-6
    THROTTLE = 400E-6

    while True:
        try:
            ROLL = roll_q.get(block=False)
        except:
            pass

        try:
            THROTTLE = throttle_q.get(block=False)
        except:
            pass
        

        FIRST = TOTAL_PERIOD - 9*LOW - 4*UNUSED_CHANNEL - ROLL - PITCH - THROTTLE - YAW
        # print(FIRST)

        try:
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

        except:
            # print("Error")
            pass


if __name__ == "__main__":
    roll_q = Queue()
    throttle_q = Queue()

    ppm_thread = threading.Thread(target=ppm, args=(roll_q, throttle_q))
    camera_thread = threading.Thread(target=camera, args=(roll_q, throttle_q))

    ppm_thread.start()
    camera_thread.start()

    ppm_thread.join()
    camera_thread.join()


        

