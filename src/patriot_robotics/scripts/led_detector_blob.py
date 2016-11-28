#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point 
from cv_bridge import CvBridge, CvBridgeError
import cv2


class LedDetectorBlob:
    '''
    Subscribe to an image queue, process images, and publish ROI
    Subscribes:
     - image
    Publishes:
     - led_location msg/??
     - led_annotated sensor_msgs/Image
    '''

    def __init__(self):        
        rospy.init_node("led_detector_blob")
        self.bridge = CvBridge()
        self.image_publisher =  rospy.Publisher("led_annotated", Image, queue_size = 2)
        self.point_publisher =  rospy.Publisher("led_point", Point, queue_size = 2)
        rospy.Subscriber("image", Image, self.process_image)
        rospy.loginfo("Ready")     

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
        for mask in (blue_mask, green_mask, red_mask):
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, 
                                                   cv2.CHAIN_APPROX_SIMPLE)

            # if we find a contour, use it...
            if len(contours) > 0:

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
                point = Point(x=cx, y=cy, z=0.)
                self.point_publisher.publish(point)

                break


if __name__ == '__main__':
    led_detector = LedDetectorBlob()
    rospy.spin()
