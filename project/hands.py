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
        skin_lower_bound = np.array([0,36,70], dtype=np.uint8) # Original: 0, 20, 70
        skin_upper_bound = np.array([27,255,255], dtype=np.uint8) # Original: 20, 255, 255
        
        # extract all pixels that are skin color
        # inRange() returns an array based on the source array, where any pixel in the range is given a 0
        # and any pixel out of the range is given a 225
        mask = cv2.inRange(roi_hsv, skin_lower_bound, skin_upper_bound)
        
        # extrapolate the hand to fill dark spots within
        # We're essentially dilating each pixel to cover the black space in the image. this is done "iterations" times
        # kernel is a 3x3 array of 1's or data type uint8
        kernel = np.ones((2, 2),np.uint8) # original code used 3 for kernel dimension
        mask = cv2.dilate(mask, kernel, iterations = 3) # original code used 4 for iterations
        
        #blur the image to smooth it
        mask = cv2.GaussianBlur(mask,(5,5),100) 
        
        #find contours
        contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        #find contour of max area(hand)
        contour = max(contours, key = lambda x: cv2.contourArea(x))
        
        #approx the contour a little
        epsilon = 0.0005*cv2.arcLength(contour,True)
        approx= cv2.approxPolyDP(contour,epsilon,True)
       
        #make convex hull around hand
        hull = cv2.convexHull(contour)
        
        #define area of hull and area of hand
        areahull = cv2.contourArea(hull)
        areacontour = cv2.contourArea(contour)
      
        #find the percentage of area not covered by hand in convex hull
        arearatio=((areahull-areacontour)/areacontour)*100
    
        #find the defects in convex hull with respect to hand
        hull = cv2.convexHull(approx, returnPoints=False)
        defects = cv2.convexityDefects(approx, hull)
        
        # l = no. of defects
        l=0
        
        #code for finding no. of defects due to fingers
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(approx[s][0])
            end = tuple(approx[e][0])
            far = tuple(approx[f][0])
            pt= (100,180)
                       
            # find length of all sides of triangle
            a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
            c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
            s = (a+b+c)/2
            ar = math.sqrt(s*(s-a)*(s-b)*(s-c))
            
            #distance between point and convex hull
            d=(2*ar)/a
            
            # apply cosine rule here
            angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
            
            # ignore angles > 90 and ignore points very close to convex hull(they generally come due to noise)
            if angle <= 90 and d>30:
                l += 1
                cv2.circle(roi, far, 3, [255,0,0], -1)
            
            #draw lines around hand
            cv2.line(roi,start, end, [0,255,0], 2)
            
        l+=1
        
        #print corresponding gestures which are in their ranges
        font = cv2.FONT_HERSHEY_SIMPLEX
        if l==1:
            if arearatio<17.5:
                cv2.putText(frame,'Thumbs Up',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    
            
        #show the windows
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
    



