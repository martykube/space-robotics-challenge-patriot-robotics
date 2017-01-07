#!/usr/bin/env python

import copy
import time
import rospy
import tf
import tf2_ros
import numpy
import std_msgs
import geometry_msgs

from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped

ROBOT_NAME = None

RIGHT_HAND_FRAME_NAME = "rightPalm"
LEFT_HAND_FRAME_NAME = "leftPalm"

'''
A simple demo which moves the right hand out then back in, from it's current position.
'''
def sendRightHandTrajectory():
    handStart = getCurrentPosition(HandTrajectoryRosMessage.RIGHT)

    # This message commands the controller to move in taskspace a hand to the desired pose (position &
    # orientation) while going through the specified trajectory points. A third order polynomial function
    # is used to interpolate positions and a hermite based curve (third order) is used to interpolate the
    # orientations. To excute a single straight line trajectory to reach a desired hand pose, set only one
    # trajectory point with zero velocity and its time to be equal to the desired trajectory time.
    msg = HandTrajectoryRosMessage()

    msg.robot_side = HandTrajectoryRosMessage.RIGHT
    msg.base_for_control = HandTrajectoryRosMessage.CHEST
    msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
    msg.unique_id = rospy.Time.now().nsecs

    trajectory = createHandOffset(HandTrajectoryRosMessage.RIGHT, [0.0, -0.2, -0.2])
    trajectory.time = 1
    msg.taskspace_trajectory_points.append(trajectory)

    rospy.loginfo('publishing first right hand trajectory')
    handTrajectoryPublisher.publish(msg)

    time.sleep(5)
    
    # trajectory = createHandOffset(HandTrajectoryRosMessage.RIGHT, [0.0, 0.2, 0.2])
    trajectory.position = handStart.position
    trajectory.orientation = handStart.orientation
    trajectory.time = 1
    msg.previous_message_id = msg.unique_id
    msg.unique_id = rospy.Time.now().nsecs
    msg.execution_mode = HandTrajectoryRosMessage.QUEUE
    msg.taskspace_trajectory_points[0] = trajectory
 
    rospy.loginfo('publishing second right hand trajectory')
    handTrajectoryPublisher.publish(msg)

    time.sleep(5)

# Get current position and orientation of the hand in world frame as a SE3TrajectoryPointRosMessage
def getCurrentPosition(stepSide):
    handstep = SE3TrajectoryPointRosMessage()

    if stepSide == HandTrajectoryRosMessage.LEFT:
        hand_frame = LEFT_HAND_FRAME_NAME
    else:
        hand_frame = RIGHT_HAND_FRAME_NAME

    handWorld = tfBuffer.lookup_transform('world', hand_frame, rospy.Time())
    handstep.orientation = handWorld.transform.rotation
    handstep.position = handWorld.transform.translation

    rospy.loginfo('current world position: {0}'.format(handstep.position))

    return handstep

def createHandOffset(stepSide, offset):
    handstep = getCurrentPosition(stepSide)

    # do the move
    handstep.position.x += offset[0];
    handstep.position.y += offset[1];
    handstep.position.z += offset[2];

    return handstep

if __name__ == '__main__':
    try:
        rospy.init_node('patriot_hand_demo')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
	rospy.loginfo("robot name: {0}".format(ROBOT_NAME))

        handTrajectoryPublisherName = "/ihmc_ros/{0}/control/hand_trajectory".format(ROBOT_NAME);
        rospy.loginfo(handTrajectoryPublisherName)

        handTrajectoryPublisher = rospy.Publisher(handTrajectoryPublisherName, HandTrajectoryRosMessage, queue_size=1)

        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1) 

        # make sure the simulation is running otherwise wait
        
        if handTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while handTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()
                rospy.loginfo('Still waiting...')
	
	rospy.loginfo('Subscriber found.')
        
        if not rospy.is_shutdown():
            sendRightHandTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
