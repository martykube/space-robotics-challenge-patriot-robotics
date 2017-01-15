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

from std_msgs.msg import Empty, UInt8

#=========================================================================================================
# Constants
#=========================================================================================================
LEFT = 0
RIGHT = 1
ROBOT_NAME = None
STEP_OFFSET_MINOR = 0.2
STEP_OFFSET_MAJOR = 0.4
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None
FOOT_ORIENTATION = Quaternion(x=0, y=0, z=0, w=1)

#=========================================================================================================
# Supporting Methods
#=========================================================================================================
def crossDoorThreshold():
    msg = FootstepDataListRosMessage()

    # ----- Default Value: 1.5
    msg.transfer_time = 0.7
    msg.swing_time = 0.7
    msg.execution_mode = FootstepDataListRosMessage.OVERRIDE
    msg.unique_id = rospy.Time.now().nsecs

    forward_offset = 0.6
    door_center_y = -0.005 # where the door is centered in the world
    foot_separation = 0.19 # when the robot is dropped in world, it is 0.18
    walking_foot_separation = 0.25 # our usual "walking" separation
    # threshold_height = 0.05

    footstepLeft = createFootStepInPlace(LEFT)
    footstepLeft.location.x += forward_offset/2.0
    footstepLeft.location.y = door_center_y + (foot_separation/2.0)

    msg.footstep_data_list.append(footstepLeft)
    rospy.loginfo("Stepping half-step left.")
    msg.unique_id = rospy.Time.now().nsecs
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []
    
    footstepRight = createFootStepInPlace(RIGHT)
    footstepRight.location.x += forward_offset
    footstepRight.location.y = door_center_y + (-foot_separation/2.0)
        
    msg.footstep_data_list.append(footstepRight)
    msg.unique_id = rospy.Time.now().nsecs
    rospy.loginfo("Stepping right.")
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []

    rospy.loginfo("Stepping left.")
    footstepLeft.location.x += forward_offset

    msg.footstep_data_list.append(footstepLeft)
    msg.unique_id = rospy.Time.now().nsecs
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    rospy.loginfo("Step finished.")
    msg.footstep_data_list[:] = []

    rospy.loginfo("Stepping half-step right plus offset.")
    footstepRight.location.x += forward_offset/2.0
    footstepRight.location.y -= walking_foot_separation - foot_separation

    msg.footstep_data_list.append(footstepRight)
    msg.unique_id = rospy.Time.now().nsecs
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    rospy.loginfo("Steps finished.")
    msg.footstep_data_list[:] = []

    return

def setFeet(both_x = 0.02, right_y=-0.12, foot_separation=0.25):
    rospy.loginfo('Setting feet to fixed position.')
    
    right_footstep = FootstepDataRosMessage()
    right_footstep.robot_side = RIGHT
    right_footstep.orientation = FOOT_ORIENTATION
    right_footstep.location = Vector3(x=both_x, y=right_y, z=0)

    left_footstep = FootstepDataRosMessage()
    left_footstep.robot_side = LEFT
    left_footstep.orientation = FOOT_ORIENTATION
    left_footstep.location = Vector3(x=both_x, y=right_y + foot_separation, z=0)

    right_current = createFootStepInPlace(RIGHT)

    if right_current.location.y > right_y:
        first_footstep = right_footstep
        second_footstep = left_footstep
    else:
        first_footstep = left_footstep
        second_footstep = right_footstep

    msg = FootstepDataListRosMessage()

    # ----- Default Value: 1.5
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = FootstepDataListRosMessage.OVERRIDE

    msg.unique_id = rospy.Time.now().nsecs
    msg.footstep_data_list.append(first_footstep)
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    msg.footstep_data_list[:] = []     

    msg.unique_id = rospy.Time.now().nsecs
    msg.footstep_data_list.append(second_footstep)
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    msg.footstep_data_list[:] = []   

    return left_footstep, right_footstep

def walkToLocation(num_steps):
    msg = FootstepDataListRosMessage()
    
    # ----- Default Value: 1.5
    msg.transfer_time = 0.7
    msg.swing_time = 0.7
    msg.execution_mode = FootstepDataListRosMessage.OVERRIDE
    msg.unique_id = rospy.Time.now().nsecs

    stepCounter = 0
    # footSeparation = 0.07, x2

    left_footstep = createFootStepInPlace(LEFT)
    left_footstep.orientation = FOOT_ORIENTATION
    right_footstep = createFootStepInPlace(RIGHT)
    right_footstep.orientation = FOOT_ORIENTATION

    #-------------------------------------------------------------------------
    # Takes the initial step using the left foot first.
    #-------------------------------------------------------------------------
    rospy.loginfo('Taking initial forward step...')
    left_footstep.location.x += STEP_OFFSET_MINOR
    msg.unique_id = rospy.Time.now().nsecs
    msg.footstep_data_list.append(left_footstep)
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    stepCounter += 1
    rospy.loginfo('Step {0} finished.'.format(stepCounter))
    msg.footstep_data_list[:] = []

    msg.execution_mode = FootstepDataListRosMessage.QUEUE

    #---------------------------------------------------------------------------------
    # Decided to hard code the number of steps since the qualifying round is static.
    # 14 forward steps takes the robot to approximately 0.5m from the door.
    #
    # 12 more steps takes us through the door and past the red line.
    #--------------------------------------------------------------------------------
    while stepCounter < num_steps:
        if stepCounter % 2 == 0:
            left_footstep.location.x += STEP_OFFSET_MAJOR
            msg.footstep_data_list.append(left_footstep)
            rospy.loginfo('Stepping Left')
        else:
            right_footstep.location.x += STEP_OFFSET_MAJOR
            msg.footstep_data_list.append(right_footstep)
            rospy.loginfo('Stepping Right')

        msg.unique_id = rospy.Time.now().nsecs
        footStepListPublisher.publish(msg)
        waitForFootstepCompletion()
        stepCounter += 1
        rospy.loginfo('Step {0} finished.'.format(stepCounter))
        msg.footstep_data_list[:] = []
        
    #-------------------------------------------------------------------------
    # Finish by bring trailing foot up next to leading foot.
    #-------------------------------------------------------------------------
    if stepCounter % 2 == 0:
        left_footstep.location.x += STEP_OFFSET_MINOR
        msg.footstep_data_list.append(left_footstep)
        rospy.loginfo('Stepping Left')
    else:
        right_footstep.location.x += STEP_OFFSET_MINOR
        msg.footstep_data_list.append(right_footstep)
        rospy.loginfo('Stepping Right')

    msg.unique_id = rospy.Time.now().nsecs
    footStepListPublisher.publish(msg)
    waitForFootstepCompletion()
    stepCounter += 1
    msg.footstep_data_list[:] = []
    rospy.loginfo('Step {0} finished.'.format(stepCounter))

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
                rospy.loginfo('Subscribers Initiated.')

                #-------------------------------------------------------------------------
                # Publishers
                #-------------------------------------------------------------------------
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)
                buttonPressPublisher = rospy.Publisher("/patriot_robotics/button_press", Empty, queue_size=5)
                armResetPublisher = rospy.Publisher("/patriot_robotics/reset_hand", UInt8, queue_size=1)
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

            if armResetPublisher.get_num_connections() == 0:
                rospy.loginfo('waiting for reset_hand subscriber...did you start hand_control node?')
                while armResetPublisher.get_num_connections() == 0:
                   rate.sleep()

            #---------------------------------------------------------------------
            # Re-position left arm so it is not sticking out.
            #---------------------------------------------------------------------

            rospy.loginfo('Re-position Left Arm...')
            armResetPublisher.publish(LEFT)
            time.sleep(15)  # because otherwise this message doesn't always get through

            # put a little debug in here
            right_footstep = createFootStepInPlace(RIGHT)
            locLeft = createFootStepInPlace(LEFT).location
            locRight = right_footstep.location
            rospy.loginfo('Starting foot separation: {0}'.format(locLeft.y - locRight.y))
            rospy.loginfo('Starting right foot in x: {0}'.format(locRight.x))
            rospy.loginfo('Starting right foot in y: {0}'.format(locRight.y))
            rospy.loginfo('Foot orientation: {0}:'.format(right_footstep.orientation))

            #--------------------------------------------------
            # Put feet in a known, good location to start.
            #--------------------------------------------------            
            left_footstep, right_footstep = setFeet()

            #-------------------------------------------------------------------------
            # Initiate walking forwards.
            #-------------------------------------------------------------------------
            if not rospy.is_shutdown():
                #---------------------------------------------------------------------
                # Walk up to door.
                #---------------------------------------------------------------------
                rospy.loginfo('Begin walking towards door...')
                walkToLocation(14)
                rospy.loginfo('Arrived at door.')

                # put a little debug in here
                xLeft = createFootStepInPlace(LEFT).location.x
                rospy.loginfo('Button press location in x: {0}'.format(xLeft))

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
                walkToLocation(2)

                rospy.loginfo('Crossing door threshold...')
                crossDoorThreshold()

                rospy.loginfo('Begin walking through door...')
                walkToLocation(10)
                rospy.loginfo('Qual Task #2 Complete.')
               
    except rospy.ROSInterruptException:
        pass
