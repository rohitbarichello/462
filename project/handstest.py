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
        skin_lower_bound = np.array([0,55,142], dtype=np.uint8) # Original: 0, 20, 70
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
        contour = max(contours, key = lambda x: cv2.contourArea(x))

        # the contour will be imperfect. Best practice is to use approxPolyDP() to expand the contour boundary a bit
        # the function takes an epsilon and expands the contour by the epsilon amount. True indicates a closed contour
        # arclength() gets the perimeter of a contour. We use 0.0005 as a hardcoded percentage of the contour 
        # perimeter to expand the image by. 
        epsilon = 0.0005*cv2.arcLength(contour,True)
        approx= cv2.approxPolyDP(contour,epsilon,True)
        cv2.drawContours(frame, [contour], 0, (0,255,0), 1)
        
        # create a convex hull around the hand. A convex hull is a polygon that perfectly contains an object using straight lines.
        # it only has convex angles, no concave angles. It will create vertices at the fingertips when detecting hands.
        hull = cv2.convexHull(contour)
        cv2.drawContours(frame, [hull], 0, (0,255,0), 3)
        
        # get area of hull and contour
        hull_area = cv2.contourArea(hull)
        contour_area = cv2.contourArea(contour)
      
        # find the percentage of the area of the convex hull not overlapping with the hand
        area_ratio = ((hull_area-contour_area) / contour_area) * 100

        # get hull defects. defects are essentially the valleys in a convex hull. since the hull is only convex angles, the defects are
        # essentially all the would-be concave angles. In a hand, these end up being the valleys between the fingers
        hull = cv2.convexHull(contour, returnPoints=False)
        defects = cv2.convexityDefects(approx, hull)

        # cv2.drawContours(frame, [defects], 0, (0,255,0), 3)

            
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
    



