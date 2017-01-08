#!/usr/bin/env python

import copy
import time
import rospy
import tf
import tf2_ros
import math
import numpy

from geometry_msgs.msg import Vector3

from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage

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

    Publishes:
    - hand_return_sent The return trajectory has been determined and it is safe to move the 
    robot forward again
    '''

    def __init__(self):   
        self.RIGHT_HAND_FRAME_NAME = "rightPalm"
        self.LEFT_HAND_FRAME_NAME = "leftPalm"

        self.LEFT = HandTrajectoryRosMessage.LEFT
        self.RIGHT = HandTrajectoryRosMessage.RIGHT

        # messages to be processed
        rospy.Subscriber("button_press", ButtonPressMessage, self.pushButton)
   
        # trajectoryPublisherName = "/ihmc_ros/valkyrie/control/hand_trajectory"
        self.trajectoryPublisher = rospy.Publisher("trajectoryPublisher", 
            HandTrajectoryRosMessage, queue_size=1)
   
    '''
    Push a button. The button is assumed to be in the y-z plane so the palm will be oriented
    accordingly.

    For parameters, see ButtonPressMessage.

    side = which hand to use
    motion_time = defaults to 2s
    press_time = defaults to 0.1s

    The function moves the hand to do the press, then moves it back to its original position. If
    the rest of the robot has moved before the return motion occurs, the hand position will adjust 
    relative to the robot's chest. It is therefore possible to start the robot moving again immediately
    after this function completes. (Of course, the door might not be open then in that case, or
    you might catch the arm on the frame.)
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
 
        # remember where we started
        start_trajectory = SE3TrajectoryPointRosMessage()     
        hand_world = tfBuffer.lookup_transform('world', hand_frame, rospy.Time())
        start_trajectory.orientation = hand_world.transform.rotation
        start_trajectory.position = hand_world.transform.translation
        rospy.loginfo('starting hand position, in world: {0}'.format(start_trajectory.position))
        rospy.loginfo('starting hand orientation, in world: {0}'.format(start_trajectory.orientation))

        msg.base_for_control = HandTrajectoryRosMessage.WORLD
        msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
        msg.unique_id = rospy.Time.now().nsecs

        trajectory = SE3TrajectoryPointRosMessage()
        trajectory.position = in_msg.button_position
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

        '''
        Pause
        '''
        press_time = in_msg.press_time
        if press_time == None or press_time <= 0:
            press_time = 0.1
        time.sleep(press_time)

        '''
        Return hand where it originally started
        '''
        trajectory.position = start_trajectory.position
        trajectory.orientation = start_trajectory.orientation
        # in case the robot is moving while we issue the command, move with the chest
        # msg.base_for_control = HandTrajectoryRosMessage.CHEST
        msg.previous_message_id = msg.unique_id
        msg.unique_id = rospy.Time.now().nsecs
        msg.execution_mode = HandTrajectoryRosMessage.QUEUE
        msg.taskspace_trajectory_points[0] = trajectory
     
        rospy.loginfo('publishing return hand trajectory')
        self.trajectoryPublisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("hand_control")
    hand_controller = HandControl()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo("HandControl ready")     
    rospy.spin()
