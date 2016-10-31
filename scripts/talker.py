#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
# Import OpenCV
import cv2
# Import Numpy
import numpy as np
#ballsize 50.8mm
#camera
##pixel 3um*3um = 0.003 * 0.003 mm^2
##lens 6.35mm
##pixelsize/ballsize = lens/picsize
##pixelsize/ballsize = pixeldev/realdev
center = [320.0,240.0]
pixel = 0.003
lens = 3.5
ballsize = 50.8
lock = 0
#command = 50
lost_count = 0
pub = rospy.Publisher('servo', UInt16, queue_size=10)

def callback(data):
    global lock
    lock = data.data

def talker(angle):
    #global command
    global pub
    global lock
    
    #rate = rospy.Rate(0.5) # 10hz
    
    '''if command == 50:
	command = 80
    elif command == 80:
	command = 50
    '''	
    if angle < 0:
	angle = int(np.abs(angle)) + 180
    if lock == 0:
    	pub.publish(angle)
    #rate.sleep()

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("lock", UInt16, callback)
    camera_feed = cv2.VideoCapture(0)
    while (1):
        _, frame = camera_feed.read()
	#print frame
        # Convert the current frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the threshold for finding a blue object with hsv
        # lower_blue = np.array([20, 100, 100])
        # upper_blue = np.array([100, 255, 255])
        ### Define green or red
        lower_red = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        upper_red = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([179, 255, 255]))
    
        mask = cv2.addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0)
        # Create a binary image, where anything blue appears white and everything else is black
        # mask = cv2.inRange(hsv, lower_red, upper_red)
    
        # Get rid of background noise using erosion and fill in the holes using dilation and erode the final image on last time
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.erode(mask, element, iterations=2)
        mask = cv2.dilate(mask, element, iterations=2)
        mask = cv2.erode(mask, element)
    
        # Create Contours for all blue objects
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        maximumArea = 0
        bestContour = None
        #print contours
        for contour in contours:
            currentArea = cv2.contourArea(contour)
            if currentArea > maximumArea:
                bestContour = contour
                maximumArea = currentArea
                # Create a bounding box around the biggest blue object
	
        if bestContour is not None:
            x, y, w, h = cv2.boundingRect(bestContour)
	    if (w+h)/2 > 20:
                lost_count = 0
                ballcenter = [(x+w/2)*1.0, (y+h/2)*1.0]
	        cv2.circle(frame, (x+w/2, y+h/2), (w+h)/4, (0, 0, 255), 3)
                dis = lens*ballsize/(max([w,h])*pixel)
	    
	        #dev = np.sqrt((ballcenter[0]-center[0])*(ballcenter[0]-center[0]) + (ballcenter[1]-center[1])*(ballcenter[1]-center[1]))
	        dev = ballcenter[0] - center[0]	    
	        realdev = dev*ballsize/(max([w,h]))
	        #des_angle = np.floor(np.arcsin(realdev/dis)*180.0/np.pi)
	        des_angle = np.floor(np.arctan(dev*pixel/lens)*180.0/np.pi)
                cv2.putText(frame,str(dis),(x+w/2, y+h/2),cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 1)
	        try:
        	    #talker()
		    a_input = int(des_angle)
	            talker(a_input)
    	        except rospy.ROSInterruptException:
        	    pass
	    else:
		lost_count += 1
		if lost_count > 10:
		    a_input = 999
	            talker(a_input)
	else:
	    lost_count += 1
	    if lost_count > 10:
	        a_input = 999
	        talker(a_input)
        
	# Show the original camera feed with a bounding box overlayed
        cv2.imshow('frame', frame)
        # Show the contours in a seperate window
        #cv2.imshow('mask', mask)
        # Use this command to prevent freezes in the feed
        k = cv2.waitKey(5) & 0xFF
        # If escape is pressed close all windows
        if k == 27:
            break

    cv2.destroyAllWindows()
    
