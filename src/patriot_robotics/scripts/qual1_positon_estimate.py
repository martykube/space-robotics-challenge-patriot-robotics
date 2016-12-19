#!/usr/bin/env python

import struct
import ctypes
import math

import rospy
import tf2_ros as tf
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point, PointStamped 
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from patriot_robotics.msg import PointStampedColorRGBA


class Qual1PositionEstimate:
    '''
    Translate LED image locations to 3D locations in R5 head frame

    Subscribes
    - led_image_location         PointStamped with the image (x, y) point and header
    - stereo_camera_pointcloud   Point cloud from the stereo_image_proc
    - tf

    Publishes
    - led_3D_location            The LED location in the R5 head frame

    '''

    def __init__(self):
        rospy.Subscriber("led_image_location", 
                         PointStamped, self.led_image_location)
        self.pointcloud = None
        rospy.Subscriber("stereo_camera_pointcloud", 
                         PointCloud2, self.stereo_camera_pointcloud)

        self.led_publisher = rospy.Publisher("led_3D_location", 
                                            PointStampedColorRGBA, queue_size = 20)

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.target_frame = 'head'


    def led_image_location(self, point_stamped):
        # read the point cloud at the image position
        u, v = int(point_stamped.point.x), int(point_stamped.point.y)
        rospy.loginfo('led_image_location (%f, %f)', u, v) 
        gen = pc2.read_points(self.pointcloud, uvs=((u, v), ))
        (x, y, z, rgb) = list(gen)[0]
        if math.isnan(z):
            # we don't have a point in the point cloud here
            rospy.loginfo('No point')
            return
        (r, g, b) = self.to_r_g_b(rgb)
        rospy.loginfo('raw (r, g, b) (%d, %d, %d)', r, g, b)
        # fixup values to be a pure red, green, or blue
        # set out of range to 0 and threshold at 127 to 0 or 255
        colors = []
        for channel in (r, g, b):
            if channel < 0 or channel > 255:
                channel = 0
            if channel > 127:
                channel = 255
            if channel < 127:
                channel = 0
            colors.append(channel)
        (r, g, b) = colors
        rospy.loginfo('Camera (x, y, z, r, g, b) (%f, %f, %f, %d, %d, %d)', x, y, z, r, g, b)
        
        #
        # translate frame from left_camera_optical_frame to head
        #

        # Coordinates!  This is my understanding...
        #
        # Camera optical frame coordinates are x right, y down, z ahead
        # Head frame coordinates are x ahead, y left, z up
        # It appears to me that tf handles this correctly.  
        #
        # The depth channel is:
        # A large z value in the camera optical frame.        
        # A large x value in the head frame.
        #
        # Also expect sign change on the image plane coordinates as the
        # camera is installed upside down. 
        #
        point_camera_frame = PointStamped(point=Point(x=x, y=x, z=z), 
                                          header=point_stamped.header)
        # check for transform
        if not self.tf_buffer.can_transform(self.target_frame, 
                                             point_camera_frame.header.frame_id, 
                                             point_camera_frame.header.stamp):
            rospy.loginfo('No transform')

        trans = self.tf_buffer.lookup_transform(
            self.target_frame,
            point_camera_frame.header.frame_id,
            point_camera_frame.header.stamp)
        
        point_head_frame = tf2_geometry_msgs.do_transform_point(
            point_camera_frame,
            trans)

        rospy.loginfo('Head (x, y, z, r, g, b) (%f, %f, %f, %d, %d, %d)', 
                      point_head_frame.point.x, point_head_frame.point.y, 
                      point_head_frame.point.z, 
                      r, g, b)
        # and publish
        point_to_publish = PointStampedColorRGBA(
            point_stamped=point_head_frame,
            color_rgba=ColorRGBA(r=r, g=g, b=b, a=1))

        self.led_publisher.publish(point_to_publish)


    def to_r_g_b(self, rgb):
        '''
        convert the PointCloud2 packed rgb into distinct ints in 0-255
        http://answers.ros.org/question/208834/read-colours-from-a-pointcloud2-python/
        http://wiki.ros.org/pcl/Overview

        x - the X Cartesian coordinate of a point (float32)
        y - the Y Cartesian coordinate of a point (float32)
        z - the Z Cartesian coordinate of a point (float32)
        
        rgb - the RGB (24-bit packed) color at a point (uint32) 
        '''
         # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,rgb)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x0000FF00)
        return (r, g, b)


    def stereo_camera_pointcloud(self, pointcloud2):
        '''
        Use the current point cloud, always.  If we start moving the robot 
        we might have to buffer pointclouds and match to image timestamps.
        '''
        self.pointcloud = pointcloud2


if __name__ == '__main__':
    rospy.init_node("qual1_position_estimate")
    led_positioner = Qual1PositionEstimate()
    rospy.loginfo("Ready")     
    rospy.spin()

