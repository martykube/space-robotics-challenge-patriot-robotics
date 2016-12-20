#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped 
from std_msgs.msg import Empty


class Qual1ImageProc:
    '''
    Identify when LEDs turn on or off and publish image coordinates

    Subscribes:
    - image

    Publishes:
    - acquired When LED is detected
    - lost when no LED is detected    
    - on_off_annotated Debuging image for white screen detector
    - led_location LED image coordinates
    - led_annotated Debugging image for LED
    '''

    def __init__(self):        
        self.bridge = CvBridge()
        self.first_image = None

        # images to be processed
        rospy.Subscriber("image", Image, self.process_image)

        # LED detector
        self.acquired_publisher = rospy.Publisher("acquired", Empty, queue_size = 20)
        self.lost_publisher = rospy.Publisher("lost", Empty, queue_size = 20)
        self.image_publisher =  rospy.Publisher("led_annotated", Image, queue_size = 20)
        self.point_publisher =  rospy.Publisher("led_point", PointStamped, queue_size = 20)

    def process_image(self, image):
        '''
        Convert inbound image to OpenCV.
        Apply LED detector
        Publish location is LED is detected
        '''
        cv2_img_bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        image_hsv = cv2.cvtColor(cv2_img_bgr, cv2.COLOR_BGR2HSV)

        blue_mask = cv2.inRange(image_hsv, (119, 127, 127), (121, 255, 255))
        green_mask = cv2.inRange(image_hsv, (59, 127, 127), (61, 255, 255))
        red_mask = cv2.inRange(image_hsv, (0, 127, 127), (1, 255, 255))

        image_with_contours = None
        acquired = False

        for mask in (blue_mask, green_mask, red_mask):
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, 
                                                   cv2.CHAIN_APPROX_SIMPLE)

            # if we find a contour, use it...
            if len(contours) > 0:
                acquired = True

                # publish annotated image
                cv2.drawContours(cv2_img_bgr, contours, 0, (20, 255, 57), 3)
                ros_img_contours = self.bridge.cv2_to_imgmsg(cv2_img_bgr, 
                                                             encoding="bgr8")
                self.image_publisher.publish(ros_img_contours)

                # publish center of contour
                M = cv2.moments(contours[0])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # OpenCV switches x and y for points.  Be careful here.
                # We want:  origin top left, x right, y down
                point = PointStamped(header=image.header, point=Point(x=cx, y=cy, z=0.)) 
                self.point_publisher.publish(point)

                break

        if acquired:
            self.acquired_publisher.publish(Empty())
        else:
            self.lost_publisher.publish(Empty())


if __name__ == '__main__':
    rospy.init_node("qual1_image_proc")
    led_detector = Qual1ImageProc()
    rospy.loginfo("Ready")     
    rospy.spin()
    
