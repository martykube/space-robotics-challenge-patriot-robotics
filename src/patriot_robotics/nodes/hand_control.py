#!/usr/bin/env python

import copy
import time
import rospy
import tf
import tf2_ros
import math
import numpy

from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty, UInt8

from ihmc_msgs.msg import HandTrajectoryRosMessage, ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage, TrajectoryPoint1DRosMessage

from patriot_robotics.msg import ButtonPressMessage

from math import cos, sin

# http://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion

class HandControl:
    '''
    Perform higher-level hand motions.

    WARNING: If you command the robot to put the hand in a strange location, you can knock
    the robot over. There are no safeguards to prevent this.

    Subscribes:
    - button_push Tell the specified hand to push the button, with hand in y-z world plane
    - reset_hand Tell the specified hand to go back to resting position next to pelvis

    Test examples:
    rostopic pub --once /patriot_robotics/button_press patriot_robotics/ButtonPressMessage 1 5 "[0.7, -0.2, 0.9]"
    (above is safe only when robot is at start)
    rostopic pub --once /patriot_robotics/reset_hand std_msgs/UInt8 1

    '''
    def __init__(self):   
        # hardcoded to valkyrie values
        self.RIGHT_HAND_FRAME_NAME = "rightPalm"
        self.LEFT_HAND_FRAME_NAME = "leftPalm"
        self.PELVIS_FRAME_NAME = "pelvis"

        self.LEFT = HandTrajectoryRosMessage.LEFT
        self.RIGHT = HandTrajectoryRosMessage.RIGHT
        
        # messages to be processed
        rospy.Subscriber("button_press", ButtonPressMessage, self.pushButton)
        rospy.Subscriber("reset_hand", UInt8, self.returnHome)
   
        # trajectoryPublisherName = "/ihmc_ros/valkyrie/control/hand_trajectory"
        self.trajectoryPublisher = rospy.Publisher("trajectoryPublisher", 
            HandTrajectoryRosMessage, queue_size=5)
        self.armTrajectoryPublisher = rospy.Publisher("armTrajectoryPublisher", 
            ArmTrajectoryRosMessage, queue_size=5)
   
    '''
    Push a button. The button is assumed to be in the y-z plane so the palm will be oriented
    accordingly.

    For parameters, see ButtonPressMessage.

    side = which hand to use
    motion_time = defaults to 1s

     '''
    def pushButton(self, in_msg):
        rospy.loginfo("Button press starting.")

        # This message commands the controller to move in taskspace a hand to the desired pose (position &
        # orientation) while going through the specified trajectory points. A third order polynomial function
        # is used to interpolate positions and a hermite based curve (third order) is used to interpolate the
        # orientations. To excute a single straight line trajectory to reach a desired hand pose, set only one
        # trajectory point with zero velocity and its time to be equal to the desired trajectory time.
        msg = HandTrajectoryRosMessage()       

        hand_side = in_msg.side
        if hand_side == self.LEFT:
            msg.robot_side = HandTrajectoryRosMessage.LEFT
            hand_frame = self.LEFT_HAND_FRAME_NAME
            rospy.loginfo("Using left hand to press button.")
        else:
            msg.robot_side = HandTrajectoryRosMessage.RIGHT
            hand_frame = self.RIGHT_HAND_FRAME_NAME
            rospy.loginfo("Using right hand to press button.")
 
        msg.base_for_control = HandTrajectoryRosMessage.WORLD
        msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
        msg.unique_id = rospy.Time.now().nsecs

        trajectory = SE3TrajectoryPointRosMessage()
        trajectory.position = in_msg.button_position
        # do not delete these next two or rotation fails
        trajectory.linear_velocity = Vector3(0, 0, 0)
        trajectory.angular_velocity = Vector3(0, 0, 0)

        '''
        Set palm parallel to y-z world plane, with fingers going right (or left)
        By default, hand in world frame has palm up (z), fingers forward (x)

        To get right hand fingers oriented properly, rotate 90 degrees about z
        (for left it would be -90)
        
        Then, we rotate 90 degrees about y so palm faces wall
        '''
        y_axis_unit = (0, 1, 0)
        z_axis_unit = (0, 0, 1)

        if hand_side == self.LEFT:
            q1 = tf.transformations.quaternion_about_axis(-numpy.pi/2, z_axis_unit)

        else:
            q1 = tf.transformations.quaternion_about_axis(numpy.pi/2, z_axis_unit)
           
        q2 = tf.transformations.quaternion_about_axis(numpy.pi/2, y_axis_unit)

        q = tf.transformations.quaternion_multiply(q1, q2)

        # TESTING ONLY set to identity quaternion
        # q = [0, 0, 0, 1] 

        # this orientation IS a quaternion, in message definition
        trajectory.orientation.x = q[0]
        trajectory.orientation.y = q[1]
        trajectory.orientation.z = q[2]
        trajectory.orientation.w = q[3]

        rospy.loginfo(trajectory.orientation)

        motion_time = in_msg.motion_time
        if motion_time == None or motion_time <= 0:
            trajectory.time = 1
        else:
            trajectory.time = float(motion_time)/2.0
        msg.taskspace_trajectory_points.append(trajectory)
 
        rospy.loginfo('publishing button push trajectory')
        self.trajectoryPublisher.publish(msg)

    # Used in returnHome function; copied from armDemo.py
    def appendArmTrajectoryPoint(self, arm_trajectory, time, positions):
        if not arm_trajectory.joint_trajectory_messages:
            arm_trajectory.joint_trajectory_messages = \
                [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
        for i, pos in enumerate(positions):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = pos
            point.velocity = 0
            arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
        return arm_trajectory

    def returnHome(self, in_msg):
        msg = ArmTrajectoryRosMessage()

        msg.robot_side = in_msg.data

        # ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
        # to understand this, generate a frame list; these joints are in that order
        # the first is shoulder roll; last is wrist roll
        rest_state = [0.0, 1.5, 0.0, 2.0, 0.0, 0.0, 0.0]
        msg = self.appendArmTrajectoryPoint(msg, 1.0, rest_state)
 
        msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
        msg.unique_id = rospy.Time.now().nsecs

        rospy.loginfo('publishing right trajectory')
        self.armTrajectoryPublisher.publish(msg)
        

    def returnHomeOld(self, in_msg): 
        msg = HandTrajectoryRosMessage()       

        msg.base_for_control = HandTrajectoryRosMessage.WORLD
        msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
        msg.unique_id = rospy.Time.now().nsecs

        trajectory = SE3TrajectoryPointRosMessage()
        # do not delete these next two or rotation fails
        trajectory.linear_velocity = Vector3(0, 0, 0)
        trajectory.angular_velocity = Vector3(0, 0, 0)
        trajectory.time = 0.5
        trajectory.orientation.x = 0;
        trajectory.orientation.y = 0;
        trajectory.orientation.z = 0;
        trajectory.orientation.w = 1;

        pelvisWorld = tfBuffer.lookup_transform('world', self.PELVIS_FRAME_NAME, rospy.Time())
        trajectory.position = pelvisWorld.transform.translation

        hand_side = in_msg.data
        offset = 1.0
        if hand_side == self.LEFT:
            msg.robot_side = HandTrajectoryRosMessage.LEFT
            trajectory.position.x += offset
            rospy.loginfo("Left hand going home.")
        else:
            msg.robot_side = HandTrajectoryRosMessage.RIGHT
            trajectory.position.x -= offset
            rospy.loginfo("Right hand going home.")
        
        msg.taskspace_trajectory_points.append(trajectory)

        rospy.loginfo('publishing hand return home trajectory')
        self.trajectoryPublisher.publish(msg)        

if __name__ == '__main__':
    rospy.init_node("hand_control")
    hand_controller = HandControl()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo("HandControl ready")     
    rospy.spin()
