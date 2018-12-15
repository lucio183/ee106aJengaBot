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



## TF Code
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import tf2_ros
from geometry_msgs.msg import TransformStamped
##


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

    ## TF CODE
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    
    ## TF CODE

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
    X = 0.075
    Y = 0.075
    Z = 0.0675

    Xp = 0.0
    Yp = -0.0425
    Zp = 0.0225

    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()

    pose.header.frame_id = "ar_marker_1"
    pose.pose.position.x = Xp
    pose.pose.position.y = Yp
    pose.pose.position.z = Zp

    pose.pose.orientation.x = Xpa
    pose.pose.orientation.y = Ypa
    pose.pose.orientation.z = Zpa
    pose.pose.orientation.w = Wpa

    planner.add_box_obstacle([X,Y,Z], "tower", pose)
    
    #-------------------------------walls

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()

    pose.header.frame_id = "base"
    pose.pose.position.x = -2
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
    pose.pose.position.y = -2
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
    pose.pose.position.y = 2
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    planner.add_box_obstacle([1,1,5], "back_wall", pose)
    #-------------------------------walls

    #Table

    X = .2
    Y = .2
    Z = .01

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

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "ar_marker_1";
    #orien_const.orientation.y = np.pi/2
    #orien_const.orientation.w = 1;
    orien_const.absolute_x_axis_tolerance = 1;
    orien_const.absolute_y_axis_tolerance = 1;
    orien_const.absolute_z_axis_tolerance = 1;
    orien_const.weight = .5;

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



    current_x = 0
    current_y = 0
    current_z = 0

    offset = 0.14
    i = 0
    while(i == 0):
        try:
            trans = tfBuffer.lookup_transform('ar_marker_1', 'right_gripper_tip', rospy.Time())
            tf_px = trans.transform.translation.x
            tf_py = trans.transform.translation.y
            tf_pz = trans.transform.translation.z
            tf_rx = trans.transform.rotation.x
            tf_ry = trans.transform.rotation.y
            tf_rz = trans.transform.rotation.z
            tf_rw = trans.transform.rotation.w
            current_x = tf_px + offset
            current_y = tf_py
            current_z = tf_pz
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue    
        i =1

    x = current_x
    y = current_y
    z = current_z











    try:
        #right_gripper_tip
        goal_1 = PoseStamped()
        goal_1.header.frame_id = "ar_marker_1"

        #x, y, and z position
        goal_1.pose.position.x = -0.0175 
        goal_1.pose.position.y = -0.0018 
        goal_1.pose.position.z = 0.065  #0.0825

        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0
        goal_1.pose.orientation.y = 0.707
        goal_1.pose.orientation.z = 0
        goal_1.pose.orientation.w = 0.707

        plan = planner.plan_to_pose(goal_1, list())#[orien_const])

        if not planner.execute_plan(plan):
            raise Exception("Execution failed0")
    except Exception as e:
        print e




    # try:
    #     #right_gripper_tip
    #     goal_1 = PoseStamped()
    #     goal_1.header.frame_id = "ar_marker_1"

    #     #x, y, and z position
    #     goal_1.pose.position.x = -0.0175 + 0.1
    #     goal_1.pose.position.y = -0.0018 + 0.1
    # #removing the 6th layer
    #     goal_1.pose.position.z = 0.065  + 0.1#0.0825

    #     #Orientation as a quaternion
    #     goal_1.pose.orientation.x = 0
    #     goal_1.pose.orientation.y = 0.707
    #     goal_1.pose.orientation.z = 0
    #     goal_1.pose.orientation.w = 0.707

    #     plan = planner.plan_to_pose(goal_1, list())#[orien_const])

    #     if not planner.execute_plan(plan):
    #         raise Exception("Execution failed0")
    # except Exception as e:
    #     print e



    # #Final Location

    # try:
    #     #right_gripper_tip
    #     goal_1 = PoseStamped()
    #     goal_1.header.frame_id = "ar_marker_1"

    #     #x, y, and z position
    #     goal_1.pose.position.x = -0.0175
    #     goal_1.pose.position.y = -0.0018
    # #removing the 6th layer
    #     goal_1.pose.position.z = 0.065 #0.0825

    #     #Orientation as a quaternion
    #     goal_1.pose.orientation.x = 0
    #     goal_1.pose.orientation.y = 0.707
    #     goal_1.pose.orientation.z = 0
    #     goal_1.pose.orientation.w = 0.707

    #     plan = planner.plan_to_pose(goal_1, list())#[orien_const])

    #     if not planner.execute_plan(plan):
    #         raise Exception("Execution failed0")
    # except Exception as e:
    #     print e




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

