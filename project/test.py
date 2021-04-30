import RPi.GPIO as GPIO
import time
import signal
from cv2 import cv2
import numpy as np
import math


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(23, GPIO.OUT)

TOTAL_PERIOD = 20.074E-3

LOW = 338.3E-6
UNUSED_CHANNEL = 1000E-6
YAW = 1000E-6
PITCH = 1000E-6

ASCENT_HAND_RATIO = 12
DESCENT_HAND_RATIO = 1

font = cv2.FONT_HERSHEY_SIMPLEX


def send_ppm(FIRST, ROLL, PITCH, THROTTLE, YAW):
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

def initiate_ascent():
    print('Ascending')

    # ROLL = 1000E-6
    # THROTTLE = 400E-6

    # MAX_THROTTLE = 900E-6
    # step = (MAX_THROTTLE - THROTTLE ) / 200
    
    # for i in range(200):
    #     if i % 100 == 0:
    #         print(i / 100)

    #     THROTTLE += step

    #     FIRST = TOTAL_PERIOD - 9*LOW - 4*UNUSED_CHANNEL - ROLL - PITCH - THROTTLE - YAW
        
    #     send_ppm(FIRST, ROLL, PITCH, THROTTLE, YAW)
        
    #     time.sleep(0.01)

    # return THROTTLE

def initiate_descent(CURRENT_THROTTLE):
    print("Descending")

    # ROLL = 1000E-6
    # THROTTLE = CURRENT_THROTTLE

    # step = (CURRENT_THROTTLE - 400E-6 ) / 400
    
    # for i in range(400):
    #     if i % 100 == 0:
    #         print(i / 100)
            
    #     THROTTLE -= step

    #     FIRST = TOTAL_PERIOD - 9*LOW - 4*UNUSED_CHANNEL - ROLL - PITCH - THROTTLE - YAW
        
    #     send_ppm(FIRST, ROLL, PITCH, THROTTLE, YAW)
        
    #     time.sleep(0.01)

    print("Seeya")
    exit()

def main(AIRBORNE, THROTTLE):
    capture = cv2.VideoCapture(0)

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
            skin_lower_bound = np.array([10,51,102], dtype=np.uint8) # Original: 0, 20, 70
            skin_upper_bound = np.array([180,255,255], dtype=np.uint8) # Original: 20, 255, 255
            
            # extract all pixels that are skin color
            # inRange() returns an array based on the source array, where any pixel in the range is given a 0
            # and any pixel out of the range is given a 225
            mask = cv2.inRange(roi_hsv, skin_lower_bound, skin_upper_bound)
            
            # extrapolate the hand to fill dark spots within
            # We're essentially dilating each pixel to cover the black space in the image. this is done "iterations" times
            # kernel is a 3x3 array of 1's or data type uint8
            kernel = np.ones((2, 2),np.uint8) # original code used 3 for kernel dimension
            mask = cv2.dilate(mask, kernel, iterations = 3) # original code used 4 for iterations
            
            # blur the image. Original kernal for blur was 5
            mask = cv2.GaussianBlur(mask,(5,5),100) 

            # return contours, a list of coordinates in the image that make up the contour
            # blurring the image helps to reduce amount of contours. Idk if this is important yet. 
            contours, hierarchies = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            # we get many contours stored in the variable "contours". The one with the highest area is the hand contour
            if len(contours) > 0:
                contour = max(contours, key = lambda x: cv2.contourArea(x))

                # the contour will be imperfect. Best practice is to use approxPolyDP() to expand the contour boundary a bit
                # the function takes an epsilon and expands the contour by the epsilon amount. True indicates a closed contour
                # arclength() gets the perimeter of a contour. We use 0.0005 as a hardcoded percentage of the contour 
                # perimeter to expand the image by. 
                epsilon = 0.0005*cv2.arcLength(contour,True)
                approx= cv2.approxPolyDP(contour,epsilon,True)
                cv2.drawContours(roi, [contour], 0, (0,255,0), 1)
                # print(contour)
                
                # create a convex hull around the hand. A convex hull is a polygon that perfectly contains an object using straight lines.
                # it only has convex angles, no concave angles. It will create vertices at the fingertips when detecting hands.
                hull = cv2.convexHull(contour)
                cv2.drawContours(roi, [hull], 0, (0,255,0), 3)
                
                # get area of hull and contour
                hull_area = cv2.contourArea(hull)
                contour_area = cv2.contourArea(contour)

                # get centroid of hull
                hull_moments = cv2.moments(hull)
                centroid_x = int(hull_moments["m10"] / hull_moments["m00"])
                centroid_y = int(hull_moments["m01"] / hull_moments["m00"])
                cv2.circle(roi, (centroid_x, centroid_y), 5, (255, 0, 0))

                # code to see if majority hand is in roi
                hand_to_roi_ratio = (contour_area / 67200) * 100
                # if hand_to_roi_ratio > ASCENT_HAND_RATIO:
                #     print(hand_to_roi_ratio)

                if AIRBORNE:
                    if hand_to_roi_ratio < DESCENT_HAND_RATIO:
                        initiate_descent(THROTTLE)
                else:
                    if hand_to_roi_ratio > ASCENT_HAND_RATIO:
                        THROTTTLE = initiate_ascent()
                        print("Airborne")
                        AIRBORNE = True       
                            
                # # display video feeds
                cv2.imshow('mask',mask)
                cv2.imshow('frame',frame)

                # calculate pulse width times for 4 quadcopter channels
                centroid_x_trans = centroid_x - 160
                # print(f"x: {centroid_x_trans}")
                centroid_y_trans = centroid_y - 120

                percent_deviation_x = centroid_x_trans / 120
                # print(f"{percent_deviation_x}")
                percent_deviation_y = centroid_y_trans / 160

                x_factor = (1 + percent_deviation_x) if percent_deviation_x > 0 else (1 + percent_deviation_x)
                # print(x_factor)
                y_factor = (1 + percent_deviation_y) if percent_deviation_y > 0 else (1 + percent_deviation_y)

                ROLL = 1000E-6 * x_factor
                # print(ROLL)
                THROTTLE = 400E-6 * y_factor

                FIRST = TOTAL_PERIOD - 9*LOW - 4*UNUSED_CHANNEL - ROLL - PITCH - THROTTLE - YAW

                # send ppm signal
                send_ppm(FIRST, ROLL, PITCH, THROTTLE, YAW)
                
            # exit image recognition if "d" is entered from the keyboard
            if cv2.waitKey(20) & 0xFF == ord('d'):
                break

        except:
            initiate_descent(THROTTLE)
        

    cv2.destroyAllWindows()
    capture.release()    


if __name__ == "__main__":
    AIRBORNE = False
    THROTTLE = 884E-6
    main(AIRBORNE, THROTTLE)


        
# # find the percentage of the area of the convex hull not overlapping with the hand
# area_ratio = ((hull_area-contour_area) / contour_area) * 100

# # get hull defects. defects are essentially the valleys in a convex hull. since the hull is only convex angles, the defects are
# # essentially all the would-be concave angles. In a hand, these end up being the valleys between the fingers
# hull = cv2.convexHull(approx, returnPoints=False)
# defects = cv2.convexityDefects(approx, hull)

# # finding our relevant defects
# for i in range(defects.shape[0]):
#     start, end, defect, distance = defects[i,0]
#     start = tuple(approx[start][0])
#     end = tuple(approx[end][0])
#     defect = tuple(approx[defect][0])
                
#     # find area of triangle using herons formula
#     a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
#     b = math.sqrt((defect[0] - start[0])**2 + (defect[1] - start[1])**2)
#     c = math.sqrt((end[0] - defect[0])**2 + (end[1] - defect[1])**2)
#     s = (a+b+c)/2
#     triangle_area = math.sqrt(s*(s-a)*(s-b)*(s-c))

#     # find distance between the defect point and the convex hull
#     d = (2 * triangle_area) / a

#     # law of cosines to find angle formed by triangle formed by two hull vertices and a defect point
#     angle = math.acos((b**2 + c**2 - a**2)/(2*b*c))
    
#     # # ignore angles > 90 and ignore points very close to convex hull(they generally come due to noise)
#     if angle <= 120 and d > 20:
#         cv2.circle(roi, defect, 3, [0,0,255])