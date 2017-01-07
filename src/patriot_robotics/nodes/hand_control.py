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

from patriot_robotics.msg import ButtonPressMessage

# Utility function for quaternion multiplication (probably ROS has a verison somewhere..)
def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

    return w, x, y, z

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

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
    
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
            rospy.loginfo("Using right hand to press button.")
            hand_frame = self.RIGHT_HAND_FRAME_NAME

        # remember where we started
        start_trajectory = SE3TrajectoryPointRosMessage()     
        hand_world = self.tfBuffer.lookup_transform('world', hand_frame, rospy.Time())
        start_trajectory.orientation = hand_world.transform.rotation
        start_trajectory.position = hand_world.transform.translation
        rospy.loginfo('starting hand position, in world: {0}'.format(start_trajectory.position))

        msg.base_for_control = HandTrajectoryRosMessage.WORLD
        msg.execution_mode = HandTrajectoryRosMessage.OVERRIDE
        msg.unique_id = rospy.Time.now().nsecs

        trajectory = SE3TrajectoryPointRosMessage()
        trajectory.position = in_msg.button_position

        '''
        Set palm parallel to y-z world plane, with fingers going right (or left)
        By default, hand in world frame has palm up (z), fingers forward (x)

        To get right hand fingers oriented properly, rotate 90 degrees about z
        (for left it would be -90)

        q1 = u x sin(theta/2) + cos(theta/2)
        u = [0, 0, 1], theta = 90
        q1 = [0, 0, sin(45), cos(45)]
        
        Then, we rotate 90 degrees about y so palm faces wall

        u = [0, 1, 0], theta = 90
        q2 = [0, sin(45), 0, cos(45)]
        '''
        if hand_side == self.LEFT:
            q1 = (-0.707, 0.707, 0, 0)
        else:
            q1 = (0.707, 0.707, 0, 0)
            
        q2 = (0, 0.707, 0, 0.707)
        quat = q_mult(q1, q2)
        trajectory.orientation = quat

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
            trajectory.time = 1
        else:
            trajectory.time = float(press_time)/2.0
        time.sleep(press_time)

        '''
        Return hand where it originally started (relative to the robot's chest)
        '''
        trajectory.position = start_trajectory.position
        trajectory.orientation = start_trajectory.orientation
        trajectory.time = 1
        # in case the robot is moving while we issue the command, move with the chest
        msg.base_for_control = HandTrajectoryRosMessage.CHEST
        msg.previous_message_id = msg.unique_id
        msg.unique_id = rospy.Time.now().nsecs
        msg.execution_mode = HandTrajectoryRosMessage.QUEUE
        msg.taskspace_trajectory_points[0] = trajectory
     
        rospy.loginfo('publishing return hand trajectory')
        self.trajectoryPublisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node("hand_control")
    hand_controller = HandControl()
    rospy.loginfo("HandControl ready")     
    rospy.spin()
