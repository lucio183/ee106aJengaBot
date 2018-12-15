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
from path_planning import PathPlanner
from intera_interface import Limb
from controller import Controller

# from intera_interface import Limb



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

    #Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    #Calibrate the gripper (other commands won't work unless you do this first)
    # print('Calibrating...')
    # right_gripper.calibrate()
    # rospy.sleep(2.0)


    #Close the right gripper to hold publisher
    # print('Closing...')
    # right_gripper.close()
    # rospy.sleep(1.0)

    #===================================================



    # Make sure that you've looked at and understand path_planner.py before starting

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

    #TODO: make wrt to sawyer (currently wrt to ar tag)

    X = 0.075
    Y = 0.075
    Z = 0.045

    Xp = 0.0
    Yp = -0.0425
    Zp = 0.0225

    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00

    # pose = PoseStamped()
    # pose.header.stamp = rospy.Time.now()

    # #TODO: is this the right frame name?
    # pose.header.frame_id = "ar_marker_1"
    # pose.pose.position.x = Xp
    # pose.pose.position.y = Yp
    # pose.pose.position.z = Zp

    # pose.pose.orientation.x = Xpa
    # pose.pose.orientation.y = Ypa
    # pose.pose.orientation.z = Zpa
    # pose.pose.orientation.w = Wpa

    # planner.add_box_obstacle([X,Y,Z], "tower", pose)


    #Table (currently wrt ar tag)

    X = 1
    Y = 1
    Z = .005

    Xp = 0
    Yp = 0
    Zp = 0
    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    #TODO: Is this the correct frame name?
    pose.header.frame_id = "ar_marker_1"
    pose.pose.position.x = Xp
    pose.pose.position.y = Yp
    pose.pose.position.z = Zp

    pose.pose.orientation.x = Xpa
    pose.pose.orientation.y = Ypa
    pose.pose.orientation.z = Zpa
    pose.pose.orientation.w = Wpa

    planner.add_box_obstacle([X,Y,Z], "table", pose)


    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper_tip";
    #changed from "base" frame_id
    orien_const.header.frame_id = "ar_marker_1";
    orien_const.orientation.w = 1;
    orien_const.absolute_x_axis_tolerance = 0.001;
    orien_const.absolute_y_axis_tolerance = 0.001;
    orien_const.absolute_z_axis_tolerance = 0.001;
    orien_const.weight = 20.0;

    rospy.sleep(1.0)

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                #currently wrt ar tag

                goal_1 = PoseStamped()
                goal_1.header.frame_id = "ar_marker_1"

                #x, y, and z position
                goal_1.pose.position.x = 0.0
                goal_1.pose.position.y = -0.0425
                goal_1.pose.position.z = 0.0225

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0
                goal_1.pose.orientation.y = np.pi/2
                goal_1.pose.orientation.z = 0
                goal_1.pose.orientation.w = 0

                plan = planner.plan_to_pose(goal_1, list())


    #                if not planner.execute_plan(plan):
                if not control.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
