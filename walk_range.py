#!/usr/bin/env python

#=========================================================================================================
# Imports
#=========================================================================================================
import rospy
import time
import tf
import tf2_ros
import numpy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage


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
def walkToDoor():
    global stepDistance
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = 0
    msg.unique_id = -1

    stepCounter = 0
    
    #-------------------------------------------------------------------------
    # Takes the initial step using the left foot first.
    #-------------------------------------------------------------------------
    rospy.loginfo('Taking initial step...')
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [STEP_OFFSET_MINOR, 0.0, 0.0]))
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    stepCounter += 1
    msg.footstep_data_list[:] = []

    #---------------------------------------------------------------------------------
    # Continue taking steps, alternating feet, until less that 0.75 meters from door.
    #---------------------------------------------------------------------------------
    while RANGE_TO_WALL > 0.75:
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
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                #-------------------------------------------------------------------------
                # Subscribers
                #-------------------------------------------------------------------------
                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, footStepStatus_callback)
                lidarScanRangeSubcriber = rospy.Subscriber('/multisense/lidar_scan'.format(ROBOT_NAME), LaserScan, scan_callback)

                #-------------------------------------------------------------------------
                # Publishers
                #-------------------------------------------------------------------------
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

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

            #-------------------------------------------------------------------------
            # Initiate walking towards door.
            #-------------------------------------------------------------------------
            if not rospy.is_shutdown():
                walkToDoor()
    except rospy.ROSInterruptException:
        pass
