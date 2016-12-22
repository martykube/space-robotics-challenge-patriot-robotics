#!/usr/bin/env python

import struct
import ctypes
import math
import numpy as np
from scipy import stats

import rospy
import tf2_ros as tf
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point, PointStamped 
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from patriot_robotics.msg import PointStampedColorRGBA, ImageBlob


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
                         ImageBlob, self.led_image_location)
        self.pointcloud = None
        rospy.Subscriber("stereo_camera_pointcloud", 
                         PointCloud2, self.stereo_camera_pointcloud)

        self.led_publisher = rospy.Publisher("led_3D_location", 
                                            PointStampedColorRGBA, queue_size = 20)

        # location in camera optical frame
        self.led_publisher_camera = rospy.Publisher("led_3D_location_camera", 
                                                    PointStampedColorRGBA, queue_size = 20)
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.target_frame = 'head'

    def threshold_rgb(self, rgb_tuple):
        # fixup values to be a pure red, green, or blue
        # set out of range to 0 and threshold at 127 to 0 or 255
        colors = []
        for channel in rgb_tuple:
            if channel < 0 or channel > 255:
                channel = 0
            if channel > 127:
                channel = 255
            if channel < 127:
                channel = 0
            colors.append(channel)
        return colors

    def centroid_location(self, image_blob):
        '''
        Read the point cloud at the image cetroid
        '''
        u, v = int(image_blob.centroid.x), int(image_blob.centroid.y)
        rospy.loginfo('led_image_location (%f, %f)', u, v) 
        gen = pc2.read_points(self.pointcloud, uvs=((u, v), ))
        (x, y, z, rgb) = list(gen)[0]
        if math.isnan(z):
            # we don't have a point in the point cloud here
            return None
        (r, g, b) = self.to_r_g_b(rgb)
        rospy.loginfo('raw (r, g, b) (%d, %d, %d)', r, g, b)
        (r, g, b) = colors
        
        ret = (x, y, z, r, g, b) 
        rospy.loginfo(
            'Centroid in camera frame  (x, y, z, r, g, b) (%f, %f, %f, %d, %d, %d)' %
            ret)
        return ret

    def blob_location(self, image_blob):
        '''
        Read the cloud at all of the points in a bounding rectangle
        '''
        # make sure we have a point cloud
        if self.pointcloud is None:
            rospy.loginfo('No point cloud')
            return None

        # read the point cloud at the image position
        rect = image_blob.bounding_rectangle
        (x, y, width, height) = (rect.x_offset, rect.y_offset, rect.width, rect.height)
        # query for all points in rectangle
        uvs = []
        for dx in range(width):
            for dy in range(height):
                uvs.append((x + dx, y + dy))

        gen = pc2.read_points(self.pointcloud, uvs=uvs)
        points = list(gen)
        rospy.loginfo("Read %d points", len(points))
        
        valid_points = []
        for point in points:
            (x, y, z, rgb) = point
            if not math.isnan(z):
                (r, g, b) = self.to_r_g_b(rgb)
                (r, g, b) = self.threshold_rgb((r, g, b))
                valid_points.append((x, y, z, r, g, b))
        rospy.loginfo('Valid points %d', len(valid_points))
        
        valid_points = np.array(valid_points)

        # average spatial location
        location_mean = np.mean(valid_points[:, 0:3], axis=0)

        # most frequent color
        color_mode = stats.mode(valid_points[:, 3:6], axis=0)[0].flatten().astype(int)

        ret = (location_mean[0], location_mean[1], location_mean[2], 
               color_mode[0], color_mode[1], color_mode[2])
        rospy.loginfo('ret %s', ret)

        return ret

    def led_image_location(self, image_blob):
        
        loc = self.blob_location(image_blob)
        if loc is None:
            rospy.logwarn('Could not find average in point cloud')
            return
        (x, y, z, r, g, b) = loc  

        point_camera_frame = PointStamped(
            point=Point(x=x, y=y, z=z), 
            header=image_blob.header)

        # Publish points in camera optical frame for debugging
        self.led_publisher_camera.publish(
            PointStampedColorRGBA(
                point_stamped=point_camera_frame,
                color_rgba=ColorRGBA(r=r, g=g, b=b, a=1)))
        #
        # translate frame from left_camera_optical_frame to head
        #

        # Coordinates!  This is my understanding...
        #
        # Camera optical frame coordinates are x right, y down, z ahead
        # Head frame coordinates are x ahead, y left, z up
        #
        # The depth channel is:
        # A large z value in the camera optical frame.        
        # A large x value in the head frame.
        #
        # Also expect sign change on the image plane coordinates as the
        # camera is installed upside down. 
        #
        # check for transform
        if not self.tf_buffer.can_transform(self.target_frame, 
                                             point_camera_frame.header.frame_id, 
                                             point_camera_frame.header.stamp):
            rospy.logwarn('No transform')
            return

        trans = self.tf_buffer.lookup_transform(
            self.target_frame,
            point_camera_frame.header.frame_id,
            point_camera_frame.header.stamp)
        
        point_head_frame = tf2_geometry_msgs.do_transform_point(
            point_camera_frame,
            trans)

        # Collect point and color and publish
        color = ColorRGBA(r=r, g=g, b=b, a=1)
        point_to_publish = PointStampedColorRGBA(
            point_stamped=point_head_frame,
            color_rgba=color)

        rospy.loginfo('Head (x, y, z, r, g, b) (%f, %f, %f, %d, %d, %d)', 
                      point_to_publish.point_stamped.point.x,
                      point_to_publish.point_stamped.point.y,
                      point_to_publish.point_stamped.point.z,
                      point_to_publish.color_rgba.r,
                      point_to_publish.color_rgba.g,
                      point_to_publish.color_rgba.b)

        self.led_publisher.publish(point_to_publish)


    def to_r_g_b(self, rgb):
        '''
        Convert the PointCloud2 packed rgb into distinct ints in [0-255]
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
        b = (pack & 0x000000FF)
        ret = (r, g, b)
        # rospy.loginfo("to_r_g_b tuple (%d, %d, %d)" % ret)
        return ret


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

