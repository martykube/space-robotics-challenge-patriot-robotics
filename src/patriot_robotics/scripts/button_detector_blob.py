#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point 
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ButtonDetectorBlob:
    '''
    Subscribe to an image queue, process images, and publish ROI
    Subscribes:
     - image
    Publishes:
     - led_location msg/??
     - led_annotated sensor_msgs/Image
    '''

    def __init__(self):        
        rospy.init_node("button_detector_blob")
        self.bridge = CvBridge()
        self.image_publisher =  rospy.Publisher("button_annotated", Image, queue_size = 2)
        self.point_publisher =  rospy.Publisher("button_center", Point, queue_size = 2)
        rospy.Subscriber("multisense/camera/left/image_raw", Image, self.process_image)
        rospy.loginfo("Ready")     

    def process_image(self, image):
        '''
        Convert inbound image to OpenCV.
        Apply button detector
        Publish location if button is detected
        '''
	# convert image message to opencv image
        cv2_img_bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

	# get in hsv colors
        image_hsv = cv2.cvtColor(cv2_img_bgr, cv2.COLOR_BGR2HSV)

	# get only the specific red colors used in the button
        red_mask = cv2.inRange(image_hsv, (0, 125, 90), (1, 255, 255))

        image_with_contours = None
	
        # get contours around red button
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, 
                                                   cv2.CHAIN_APPROX_SIMPLE)

	# sort from largest contour to smallest
	contours = sorted(contours, key = cv2.contourArea, reverse = True)[:20]

        # if we find a contour, use it...
        if len(contours) > 0:

		# draw the bounding rectange         
		c = contours[0]

		x, y, w, h = cv2.boundingRect(c)
		cv2.rectangle(cv2_img_bgr, (x, y), (x+w, y+h), (255, 0, 0), 2)

		# convert the opencv image to a ROS message
                ros_img_contours = self.bridge.cv2_to_imgmsg(cv2_img_bgr, 
                                                             encoding="bgr8")

		# publish the image with a drawn bounding rectangle
                self.image_publisher.publish(ros_img_contours)

                # publish center of contour
                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # OpenCV switches x and y for points.  Be careful here.
                # We want:  origin top left, x right, y down
                point = Point(x=cx, y=cy, z=0.)
                self.point_publisher.publish(point)

          #      break


if __name__ == '__main__':
    button_detector = ButtonDetectorBlob()
    rospy.spin()
