#!/usr/bin/env python


import rospy

from srcsim.msg import Console
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import  Point, Vector3
from std_msgs.msg import ColorRGBA, Header

from patriot_robotics.msg import PointStampedColorRGBA


class Qual1Rviz:
    '''
    Cross check the task 1 results by placing markers at the LED locations
    in rviz.
    '''

    def __init__(self):

        rospy.Subscriber("/srcsim/qual1/light", Console, self.light)
        # rospy.Subscriber("/patriot_robotics/led_3D_location_camera", 
        #                  PointStampedColorRGBA, self.light_camera_frame)
        self.rviz_publisher = rospy.Publisher(
            'visualization_marker', Marker, queue_size = 0)

    def light(self, console):
        rospy.loginfo('light: %s', console)
        header = Header(
            frame_id='head',
            stamp=rospy.Time())
        ns='patriot_robotics'
        scale = Vector3(x=0.01, y=0.05)
        color = ColorRGBA(a=1.0, r=1.0, g=1.0, b=1.0)
        marker = Marker(
            header=header,
            ns = ns,
            id = 0,
            type = Marker.ARROW,
            action = Marker.ADD,
            points = [Point(x=0, y=0, z=0), 
                      Point(x=console.x, y=console.y, z=console.z)],
            scale=scale,
            color=color)
        rospy.loginfo('Publishing marker %s', marker)
        self.rviz_publisher.publish(marker)

    def light_camera_frame(self, msg):
        rospy.loginfo('light_camera: %s', msg)
        point = msg.point_stamped.point
        header = Header(
            frame_id='left_camera_optical_frame',
            stamp=rospy.Time())
        ns='patriot_robotics'
        scale = Vector3(x=0.01, y=0.05)
        color = ColorRGBA(a=1.0, r=1.0, g=1.0, b=1.0)
        marker = Marker(
            header=header,
            ns = ns,
            id = 1,
            type = Marker.ARROW,
            action = Marker.ADD,
            points = [Point(x=0, y=0, z=0), 
                      Point(x=point.x, y=point.y, z=point.z)],
            scale=scale,
            color=color)
        rospy.loginfo('Publishing camera frame marker %s', marker)
        self.rviz_publisher.publish(marker)


if __name__ == '__main__':
    rospy.init_node('qual1_rviz')
    rviz = Qual1Rviz()
    rospy.loginfo('Ready')
    rospy.spin()
