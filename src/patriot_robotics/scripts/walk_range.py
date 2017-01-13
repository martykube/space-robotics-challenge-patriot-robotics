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

ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [-1.151, .930, 1.110, 0.624, 0.0, 0.0, 0.0]

#=========================================================================================================
# Supporting Methods
#=========================================================================================================
def walkToLocation(num_steps, separate_feet):
    global stepDistance
    msg = FootstepDataListRosMessage()
    
    # ----- Default Value: 1.5
    msg.transfer_time = 0.7
    msg.swing_time = 0.7
    msg.execution_mode = 0
    msg.unique_id = -1

    stepCounter = 0
    footSeparation = 0.0

    if separate_feet:    
        #-------------------------------------------------------------------------
        # Separate the feet slightly so that we can go faster with less chance of
        # foot collisions.
        #-------------------------------------------------------------------------
        rospy.loginfo('Separating feet...') 
        footSeparation = 0.07  
        
        msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, footSeparation, 0.0]))
        footStepListPublisher.publish(msg)
        waitForFootstepCompletion()
        stepCounter += 1
        msg.footstep_data_list[:] = []
        
        msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -footSeparation, 0.0]))
        footStepListPublisher.publish(msg)
        waitForFootstepCompletion()
        stepCounter += 1
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
    # 16 steps takes the robot to approximatelt 0.5m from the door.
    # The 16 steps include the 2 side-steps at the beginning of the task.
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

        rospy.loginfo('Walking - LiDAR Range: {0} Step Count: {1}'.format(RANGE_TO_WALL, stepCounter))
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
    rospy.loginfo('Walking Stopped - LiDAR Range: {0}'.format(RANGE_TO_WALL))

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


def waitForFootstepCompletion():
    global stepComplete
    stepComplete = False
    while not stepComplete:
        rate.sleep()


def sendRightArmTrajectory():
    msgA = ArmTrajectoryRosMessage()
    
    msgA.robot_side = ArmTrajectoryRosMessage.RIGHT
    
    msgA = appendTrajectoryPoint(msgA, 3.0, ZERO_VECTOR)
    msgA = appendTrajectoryPoint(msgA, 4.0, ELBOW_BENT_UP)
    msgA = appendTrajectoryPoint(msgA, 5.0, ZERO_VECTOR)

    msgA.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msgA)


def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

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
                lidarScanRangeSubcriber = rospy.Subscriber('/multisense/lidar_scan'.format(ROBOT_NAME), LaserScan, scan_callback)
                rospy.loginfo('Subscribers Initiated.')

                #-------------------------------------------------------------------------
                # Publishers
                #-------------------------------------------------------------------------
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)
                armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
                rospy.loginfo('Publishers Initiated.')

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

            if armTrajectoryPublisher.get_num_connections() == 0:
                rospy.loginfo('waiting for subscriber...')
                while armTrajectoryPublisher.get_num_connections() == 0:
                    rate.sleep()

            #-------------------------------------------------------------------------
            # Initiate walking forwards.
            #-------------------------------------------------------------------------
            if not rospy.is_shutdown():
                #---------------------------------------------------------------------
                # Walk up to door.
                #---------------------------------------------------------------------
                rospy.loginfo('Begin walking towards door...')
                walkToLocation(17, True)
                rospy.loginfo('Arrived at door.')
                
                #---------------------------------------------------------------------
                # Push Button Here.
                #---------------------------------------------------------------------
                rospy.loginfo('Begin push door button...')
                sendRightArmTrajectory()
                time.sleep(2)
                rospy.loginfo('Door is open.')

                #---------------------------------------------------------------------
                # Walk through the door.
                #---------------------------------------------------------------------
                rospy.loginfo('Begin walking through door...')
                #walkToLocation(12, False)
                rospy.loginfo('Qual Task #2 Complete.')

                
    except rospy.ROSInterruptException:
        pass
