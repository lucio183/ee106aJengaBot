#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
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
    #rospy.init_node('gripper_test')

    # # Set up the right gripper
    #right_gripper = robot_gripper.Gripper('right_gripper')

    # #Calibrate the gripper (other commands won't work unless you do this first)
    #print('Calibrating...')
    #right_gripper.calibrate()
    #rospy.sleep(2.0)


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




    #Tower

    X = .075
    Y = .15
    Z = .0675

    Xp = 0
    Yp = 0
    Zp = 0
    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00

    # pose = PoseStamped()
    # pose.header.stamp = rospy.Time.now()
    # #TODO: Is this the correct frame name?
    # pose.header.frame_id = "ar_marker_4"
    # pose.pose.position.x = Xp
    # pose.pose.position.y = Yp
    # pose.pose.position.z = Zp

    # pose.pose.orientation.x = Xpa
    # pose.pose.orientation.y = Ypa
    # pose.pose.orientation.z = Zpa
    # pose.pose.orientation.w = Wpa

    # planner.add_box_obstacle([X,Y,Z], "tower", pose)

    #Table

    X = 1
    Y = 2
    Z = .0675

    Xp = 1.2
    Yp = 0
    Zp = -0.22
    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    #TODO: Is this the correct frame name?
    pose.header.frame_id = "base"
    pose.pose.position.x = Xp
    pose.pose.position.y = Yp
    pose.pose.position.z = Zp

    pose.pose.orientation.x = Xpa
    pose.pose.orientation.y = Ypa
    pose.pose.orientation.z = Zpa
    pose.pose.orientation.w = Wpa

    planner.add_box_obstacle([X,Y,Z], "table", pose)



    try:
        #right_gripper_tip
        goal_1 = PoseStamped()
        goal_1.header.frame_id = "ar_marker_4"

        #x, y, and z position
        goal_1.pose.position.y = 0
        goal_1.pose.position.z =.5
	#removing the 6th layer
        goal_1.pose.position.x = 0 #0.0825

        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0
        goal_1.pose.orientation.y = 0
        goal_1.pose.orientation.z = 0.707
        goal_1.pose.orientation.w = 0.707

        plan = planner.plan_to_pose(goal_1, list())

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

