#!/usr/bin/env python 

'''
Read images from the filesystem and publish to ROS queue
'''
import cv2

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

import glob
import time

rospy.init_node("filesystem_camera")

images = []
bridge = CvBridge()
for filename in glob.glob('/home/marty/patriot_robotics/task1-dataset1/*.png'):
    cv2_img = cv2.imread(filename)
    ros_img = bridge.cv2_to_imgmsg(cv2_img, 'bgr8')
    images.append(ros_img)

rospy.loginfo("loaded %d images" % len(images))
publisher =  rospy.Publisher("image", Image, queue_size = 2)

start_time = time.time()
image_index = 0
image_duration_sec = 2


rate = rospy.Rate(5)
while not rospy.is_shutdown():

    publisher.publish(images[image_index])

    # change images once in a while
    current_time = time.time()
    if int(current_time - start_time) > image_duration_sec:
        start_time = current_time
        image_index = (image_index + 1) % len(images)

    rate.sleep()
