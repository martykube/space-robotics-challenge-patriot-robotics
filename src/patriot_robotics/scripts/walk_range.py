#!/usr/bin/env python

#=========================================================================================================
# Imports
#=========================================================================================================
import tf
import time
import copy
import numpy
import rospy
import tf2_ros
import argparse

from numpy import append

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

from std_msgs.msg import Empty

#=========================================================================================================
# Constants
#=========================================================================================================
LEFT = 0
RIGHT = 1
ROBOT_NAME = None
RANGE_TO_WALL = 9999
STEP_OFFSET_MINOR = 0.2
STEP_OFFSET_MAJOR = 0.4
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

#=========================================================================================================
# Supporting Methods
#=========================================================================================================
def crossDoorThreshold():
    msg = FootstepDataListRosMessage()

    # ----- Default Value: 1.5
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = FootstepDataListRosMessage.OVERRIDE
    msg.unique_id = rospy.Time.now().nsecs

    DOOR_OFFSET = 0.6
    # to make up for fact feet may have come together, move them out a bit
    sideOffset = 0.05
    threshold_height = 0.05

    # we list left already, so don't move that way; only go right
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [DOOR_OFFSET/2.0, -sideOffset, threshold_height]))
    rospy.loginfo("Stepping half-step right, and up.")
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion(3)
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []
        
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [DOOR_OFFSET, 0.0, 0.0]))
    rospy.loginfo("Stepping left.")
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion(3)
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []

    rospy.loginfo("Stepping right and down.")
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [DOOR_OFFSET/2.0, 0.0, 
-threshold_height]))
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion(3)
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []

    msg.footstep_data_list.append(createFootStepOffset(LEFT, [DOOR_OFFSET/2.0, 0.0, 0.0]))
    rospy.loginfo("Stepping half-step left.")
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion(3)
    time.sleep(15)
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []

    return

def walkToLocation(num_steps, separate_feet):
    global stepDistance
    msg = FootstepDataListRosMessage()
    
    # ----- Default Value: 1.5
    msg.transfer_time = 0.7
    msg.swing_time = 0.7
    msg.execution_mode = FootstepDataListRosMessage.OVERRIDE
    msg.unique_id = rospy.Time.now().nsecs

    stepCounter = 0
    footSeparation = 0.07

    if separate_feet:    
        #-------------------------------------------------------------------------
        # Separate the feet slightly so that we can go faster with less chance of
        # foot collisions.
        #
        # Don't count these as forward steps.
        #-------------------------------------------------------------------------
        rospy.loginfo('Separating feet...') 
        
        msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, footSeparation, 0.0]))        
        footStepListPublisher.publish(msg)
        waitForFootstepCompletion()
        # stepCounter += 1
        msg.footstep_data_list[:] = []
        
        msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -footSeparation, 0.0]))
        footStepListPublisher.publish(msg)
        waitForFootstepCompletion()
        # stepCounter += 1
        msg.footstep_data_list[:] = []

    #-------------------------------------------------------------------------
    # Takes the initial step using the left foot first.
    #-------------------------------------------------------------------------
    rospy.loginfo('Taking initial forward fstep...')
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [STEP_OFFSET_MINOR, 0.0, 0.0]))
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    stepCounter += 1
    msg.footstep_data_list[:] = []

    #---------------------------------------------------------------------------------
    # Decided to hard code the number of steps since the qualifying round is static.
    # 14 forward steps takes the robot to approximatelt 0.5m from the door.
    #
    # 12 more steps takes us through the door an dpast the red line.
    #---------------------------------------------------------------------------------
    while stepCounter < num_steps:
        if stepCounter % 2 == 0:
            msg.footstep_data_list.append(createFootStepOffset(LEFT, [STEP_OFFSET_MAJOR, 0.0, 0.0]))
            rospy.loginfo('Stepping Left')
        else:
            msg.footstep_data_list.append(createFootStepOffset(RIGHT, [STEP_OFFSET_MAJOR, 0.0, 0.0]))
            rospy.loginfo('Stepping Right')

        # rospy.loginfo('Walking - LiDAR Range: {0} Step Count: {1}'.format(RANGE_TO_WALL, stepCounter))
        rospy.loginfo('Step {0} finished.'.format(stepCounter))
        footStepListPublisher.publish(msg)
        waitForFootstepCompletion()
        stepCounter += 1
        msg.footstep_data_list[:] = []
        
    #-------------------------------------------------------------------------
    # Finish by bring trailing foot up next to leading foot.
    #-------------------------------------------------------------------------
    if stepCounter % 2 == 0:
        msg.footstep_data_list.append(createFootStepOffset(LEFT, [STEP_OFFSET_MINOR, 0.0, 0.0]))
        rospy.loginfo('Stepping Left')
    else:
        msg.footstep_data_list.append(createFootStepOffset(RIGHT, [STEP_OFFSET_MINOR, 0.0, 0.0]))
        rospy.loginfo('Stepping Right')

    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    stepCounter += 1
    msg.footstep_data_list[:] = []
    # rospy.loginfo('Walking Stopped - LiDAR Range: {0}'.format(RANGE_TO_WALL))
    rospy.loginfo('Step {0} finished.'.format(stepCounter))
 
    #-------------------------------------------------------------------------
    # Bring feet back together to get through the door without hitting sides.
    #-------------------------------------------------------------------------
    # if footSeparation == 0.07:
    #     rospy.loginfo('Bring feet back together...') 
    #     footSeparation = -0.05  
    #     
    #     msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, footSeparation, 0.0]))
    #     footStepListPublisher.publish(msg)
    #     waitForFootstepCompletion()
    #     stepCounter += 1
    #     msg.footstep_data_list[:] = []
    #     
    #     msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -footSeparation, 0.0]))
    #     footStepListPublisher.publish(msg)
    #     waitForFootstepCompletion()
    #     stepCounter += 1
    #     msg.footstep_data_list[:] = []


def createFootStepInPlace(stepSide):
    #-------------------------------------------------------------------------
    # Creates footstep with the current position and orientation of the foot.
    #-------------------------------------------------------------------------
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = LEFT_FOOT_FRAME_NAME
    else:
        foot_frame = RIGHT_FOOT_FRAME_NAME

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep


def createFootStepOffset(stepSide, offset):
    #-------------------------------------------------------------------------
    # Creates footstep offset from the current foot position.
    # The offset is in foot frame.
    #-------------------------------------------------------------------------
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

'''

Added optional timeout value (s) so we don't get stuck here.

'''
def waitForFootstepCompletion(timeout = 0):
    global stepComplete
    stepComplete = False
    
    if(timeout <= 0):
        while not stepComplete:
            rate.sleep()
    else:
        rospy.loginfo("footstep timeout is {0}".format(timeout))
        startTime = rospy.Time().nsecs
        currentTime = rospy.Time().nsecs
        while ((not stepComplete) and ((currentTime - startTime) * (10 ^ 9) < timeout)):
            rate.sleep()
            currentTime = rospy.Time().nsecs

    if not stepComplete:
        rospy.loginfo("Timeout hit for footstep completion.")
        

#=========================================================================================================
# Callbacks
#=========================================================================================================
def scan_callback(msg):
    global RANGE_TO_WALL
    RANGE_TO_WALL = min(msg.ranges)
 
def footStepStatus_callback(msg):
    global stepComplete
    if msg.status == 1:
        stepComplete = True
        
#=========================================================================================================
# Main
#=========================================================================================================
if __name__ == '__main__':
    min_dist = 0.0
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--min_d', help='Minimum distance from an object that the robot should come.', default=1.0)
    args = vars(parser.parse_args())
    
    if args['min_d']:
        min_dist = args['min_d']
        
    try:
        rospy.init_node('ihmc_walk_test')
        
        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
            rospy.loginfo('Robot Name: {0}'.format(ROBOT_NAME))

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                rospy.loginfo('Foot Parameters Found.')
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                #-------------------------------------------------------------------------
                # Subscribers
                #-------------------------------------------------------------------------
                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, footStepStatus_callback)
                # lidarScanRangeSubcriber = rospy.Subscriber('/multisense/lidar_scan'.format(ROBOT_NAME), LaserScan, scan_callback)
                rospy.loginfo('Subscribers Initiated.')

                #-------------------------------------------------------------------------
                # Publishers
                #-------------------------------------------------------------------------
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)
                buttonPressPublisher = rospy.Publisher("/patriot_robotics/button_press", Empty, queue_size=5)
                rospy.loginfo('Publishers Initiated.')
            else:
                rospy.logerr('Required parameters for subscribers missing!')

            tfBuffer = tf2_ros.Buffer()
            tfListener = tf2_ros.TransformListener(tfBuffer)

            rate = rospy.Rate(10) # 10hz
            time.sleep(1)

            #-------------------------------------------------------------------------
            # Make sure the simulation is running, otherwise wait
            #-------------------------------------------------------------------------
            if footStepListPublisher.get_num_connections() == 0:
                rospy.loginfo('waiting for subsciber...')
                while footStepListPublisher.get_num_connections() == 0:
                    rate.sleep()

            if buttonPressPublisher.get_num_connections() == 0:
                rospy.loginfo('waiting for button_press subscriber...did you start hand_control node?')
                while buttonPressPublisher.get_num_connections() == 0:
                    rate.sleep()

            #-------------------------------------------------------------------------
            # Initiate walking forwards.
            #-------------------------------------------------------------------------
            if not rospy.is_shutdown():
                #---------------------------------------------------------------------
                # Walk up to door.
                #---------------------------------------------------------------------
                rospy.loginfo('Begin walking towards door...')
                walkToLocation(14, True)
                rospy.loginfo('Arrived at door.')

                # without this, it appears subsequent messages don't get through
                time.sleep(15)
                         
                #---------------------------------------------------------------------
                # Push Button Here.
                #---------------------------------------------------------------------
                rospy.loginfo('Begin push door button...')
                # sendRightArmTrajectory() # includes reset
                buttonPressPublisher.publish(Empty())
                time.sleep(15)
                rospy.loginfo('Door is open.')
                
                #---------------------------------------------------------------------
                # Walk through the door.
                #---------------------------------------------------------------------
                rospy.loginfo('Walking up to door...')                
                walkToLocation(2, False)

                rospy.loginfo('Crossing door threshold...')
                crossDoorThreshold()

                rospy.loginfo('Begin walking through door...')
                walkToLocation(13, False)
                rospy.loginfo('Qual Task #2 Complete.')
               
    except rospy.ROSInterruptException:
        pass
