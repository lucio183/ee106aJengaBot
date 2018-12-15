#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped
import intera_interface
import intera_external_devices
from path_planner import PathPlanner
from intera_interface import Limb
from controller import Controller
from intera_interface import Limb



#===================================================
# Code to add gripper
from intera_interface import gripper as robot_gripper

#===================================================


#/ Arduino Code
import serial
import syslog
import time
#/ Arduino Code


#/ Arduino Code
#port = '/dev/ttyACM0'
#ard = serial.Serial(port,115200,timeout=5)
#/ Arduino Code




def main():
    """
    Main Script
    """
    #===================================================
    # Code to add gripper
    # rospy.init_node('gripper_test')

    # # Set up the right gripper
    # right_gripper = robot_gripper.Gripper('right_gripper')

    # #Calibrate the gripper (other commands won't work unless you do this first)
    # print('Calibrating...')
    # right_gripper.calibrate()
    # rospy.sleep(2.0)


    # #Close the right gripper to hold publisher
    # # MIGHT NOT NEED THIS
    # print('Closing...')
    # right_gripper.close()
    # rospy.sleep(1.0)

    #===================================================



    planner = PathPlanner("right_arm")

    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

    limb = intera_interface.Limb("right")
    control = Controller(Kp, Kd, Ki, Kw, limb)

    ##
    ## Add the obstacle to the planning scene here
    ##

   

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()

    pose.header.frame_id = "base"
    pose.pose.position.x = -1.1
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    planner.add_box_obstacle([1,1,5], "left_side_wall", pose)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()

    pose.header.frame_id = "base"
    pose.pose.position.x = 0
    pose.pose.position.y = -1.0
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    planner.add_box_obstacle([1,1,5], "right_side_wall", pose)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()

    pose.header.frame_id = "base"
    pose.pose.position.x = 0
    pose.pose.position.y = 1.1
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    planner.add_box_obstacle([1,1,5], "back_wall", pose)


 
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "ar_marker_1";
    #orien_const.orientation.y = np.pi/2
    #orien_const.orientation.w = 1;
    orien_const.absolute_x_axis_tolerance = 1;
    orien_const.absolute_y_axis_tolerance = 1;
    orien_const.absolute_z_axis_tolerance = 1;
    orien_const.weight = .5;

    

    joint_const = JointConstraint()    
    # Constrain the position of a joint to be within a certain bound
    joint_const.joint_name = "right_j6"
    joint_const.position = 1.7314052734375;
    # the bound to be achieved is [position - tolerance_below, position + tolerance_above]
    joint_const.tolerance_above = .2;
    joint_const.tolerance_below = .2;
    # A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
    joint_const.weight = .5;


    #for stage 2

    # orien_const1 = OrientationConstraint()
    # orien_const1.link_name = "right_gripper";
    # orien_const1.header.frame_id = "ar_marker_1";
    # #orien_const.orientation.y = np.pi/2
    # #orien_const.orientation.w = 1;
    # orien_const1.absolute_x_axis_tolerance = 0.5;
    # orien_const1.absolute_y_axis_tolerance = 0.5;
    # orien_const1.absolute_z_axis_tolerance = 0.5;
    # orien_const1.weight = 1.0;

    #rospy.sleep(1.0)
    # while not rospy.is_shutdown():
    #         while not rospy.is_shutdown():
    
    try:
        #right_gripper_tip
        goal_1 = PoseStamped()
        goal_1.header.frame_id = "ar_marker_1"

        #x, y, and z position
        goal_1.pose.position.x = -0.28
        goal_1.pose.position.y = -0.05
	#removing the 6th layer

        goal_1.pose.position.z = .38 #0.0825
        # z was .1
        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0
        goal_1.pose.orientation.y = 0
        goal_1.pose.orientation.z = 0
        goal_1.pose.orientation.w = 1

        plan = planner.plan_to_pose_joint(goal_1, [joint_const])

        if not planner.execute_plan(plan):
            raise Exception("Execution failed0")
    except Exception as e:
        print e



    # else:
    #     break


    #try:
        #stage 2
        #goal_2 = PoseStamped()
        #goal_2.header.frame_id = "ar_marker_1"

        #x, y, and z position
        #goal_2.pose.position.x = 0.08
        #goal_2.pose.position.y = -0.064
    #removing the 6th layer
        #goal_2.pose.position.z = 0.0825

        #Orientation as a quaternion
        #goal_2.pose.orientation.x = 0.0
        #goal_2.pose.orientation.y = 0.707
        #goal_2.pose.orientation.z = 0
       # goal_2.pose.orientation.w = 0.707

        #plan = planner.plan_to_pose(goal_2, list())#[orien_const1])

       # if not planner.execute_plan(plan):
      #      raise Exception("Execution failed")
    # except Exception as e:
         #print e
     #    else:
     #        break
        # else:
        #     break

    #planner.remove_obstacle("tower")



if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()

