from cv2 import cv2
import numpy as np
import math
capture = cv2.VideoCapture(0)

firstError = True
     
while True:
        
    try: #error relating to finding the max contour
        
        # each video frame stored in "frame". The frame is 640x480 px
        ret, frame = capture.read()
        
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
        skin_lower_bound = np.array([0,20,70], dtype=np.uint8) # Original: 0, 20, 70
        skin_upper_bound = np.array([20,255,255], dtype=np.uint8) # Original: 20, 255, 255
        
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
        
            
        cv2.imshow('mask',mask)
        cv2.imshow('frame',frame)

    except:
        if firstError: 
            print("error")
            firstError = False
        
    if cv2.waitKey(20) & 0xFF == ord('d'):
        break
    


cv2.destroyAllWindows()
capture.release()    
    



