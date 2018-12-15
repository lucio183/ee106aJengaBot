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
# from intera_interface import Limb

import matplotlib.pyplot as plt



## TF Code
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import tf2_ros
from geometry_msgs.msg import TransformStamped
##


def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

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
    X = 0.40 
    Y = 1.20 
    Z = 0.10

    Xp = 0.5
    Yp = 0.00
    Zp = -0.15
    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00
    
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base"
    pose.pose.position.x = Xp
    pose.pose.position.y = Yp 
    pose.pose.position.z = Zp

    pose.pose.orientation.x = Xpa
    pose.pose.orientation.y = Ypa
    pose.pose.orientation.z = Zpa
    pose.pose.orientation.w = Wpa

    #planner.add_box_obstacle([X,Y,Z], "wall", pose)


    #Wall 2

    X = 0.40 
    Y = 0.10 
    Z = 0.30

    Xp = 0.5
    Yp = -0.15
    Zp = 0.05
    Xpa = 0.00
    Ypa = 0.00
    Zpa = 0.00
    Wpa = 1.00
    
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base"
    pose.pose.position.x = Xp
    pose.pose.position.y = Yp 
    pose.pose.position.z = Zp

    pose.pose.orientation.x = Xpa
    pose.pose.orientation.y = Ypa
    pose.pose.orientation.z = Zpa
    pose.pose.orientation.w = Wpa

    #planner.add_box_obstacle([X,Y,Z], "wall2", pose)



    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    #orien_const.orientation.y = -1.0;
    #orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.05;
    orien_const.absolute_z_axis_tolerance = 0.05;
    orien_const.weight = 1.0;


    waypoints = list()



    # Section to add waypoints
    # step_size = size between x position changes
    # final_length = final x position

    #GOAL Points
        # goal_1.pose.position.x = 0.585
        # goal_1.pose.position.y = 0.156
        # goal_1.pose.position.z = -0.138
        # goal_1.pose.orientation.x =  0        
        # goal_1.pose.orientation.y =  0.707    
        # goal_1.pose.orientation.z =  0       
        # goal_1.pose.orientation.w =  0.707    
    #GOAL Points



    def zero(planner, control, waypoints):
        while not rospy.is_shutdown():
        
            ## ZERO THE ROBOT ##
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"
            goal_1.pose.position.x = 0.585
            goal_1.pose.position.y = 0.156
            goal_1.pose.position.z = -0.138
            goal_1.pose.orientation.x =  0        # -0.026
            goal_1.pose.orientation.y =  0.707    #  0.723
            goal_1.pose.orientation.z =  0        # -0.052
            goal_1.pose.orientation.w =  0.707    #  0.689

            waypoints.append(goal_1.pose)


            while not rospy.is_shutdown():
                try:    

                    plan = planner.plan_to_pose(waypoints, list() )

                    if not control.execute_path(plan):
                        raise Exception("Execution failed")                    
                except Exception as e:
                    print e
                else:
                    break



    def moveUp(planner, control, waypoints):
        j = 0
        while j < 50:
            #control._velocity_scalar = 0.5
            step_size = 0.0005 #0.0025
            FINAL_X = 0.785 #
            offset = 0.14
            tf_px = 0.585
            tf_py = 0.156
            tf_pz = -0.138
            tf_rx = 0 
            tf_ry = 0.707 
            tf_rz = 0
            tf_rw = 0.707 
            goal_x = 0.585
            goal_y = 0.156
            goal_z = -0.138

            try:
                trans = tfBuffer.lookup_transform('base', 'right_gripper_base', rospy.Time())
                tf_px = trans.transform.translation.x
                tf_py = trans.transform.translation.y
                tf_pz = trans.transform.translation.z
                tf_rx = trans.transform.rotation.x
                tf_ry = trans.transform.rotation.y
                tf_rz = trans.transform.rotation.z
                tf_rw = trans.transform.rotation.w
                goal_x = tf_px
                goal_y = tf_py
                goal_z = tf_pz

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue    

            offset = 0.14
            FINAL_Z = goal_z + 0.2
            z = tf_pz 

            update = 0
            execute = 0
            i = 0
            STOP = 0

            while ((z < FINAL_Z or update == 0 ) and (i < 2)):

                if (update == 0):
                    try:
                        trans = tfBuffer.lookup_transform('base', 'right_gripper_base', rospy.Time())
                        tf_px = trans.transform.translation.x
                        tf_py = trans.transform.translation.y
                        tf_pz = trans.transform.translation.z
                        tf_rx = trans.transform.rotation.x
                        tf_ry = trans.transform.rotation.y
                        tf_rz = trans.transform.rotation.z
                        tf_rw = trans.transform.rotation.w
                        z = tf_pz
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue

                    update = 1
                    execute = 0

                x = tf_px
                y = tf_py

                if (x < goal_x - 0.05):
                    x = x + step_size
                if (x > goal_x + 0.05):
                    x = x - step_size
                if (y < goal_y - 0.05):
                    y = y + step_size
                if (y > goal_y + 0.05):
                    y = y - step_size      

                #print("TF'd while")
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z
                #Orientation as a quaternion
                goal_1.pose.orientation.x =  0        # -0.026
                goal_1.pose.orientation.y =  0.707    #  0.723
                goal_1.pose.orientation.z =  0        # -0.052
                goal_1.pose.orientation.w =  0.707    #  0.689

                waypoints.append(goal_1.pose)

                #print("waypoints added UP")

                z = z + step_size

                if (z >= FINAL_Z):
                    execute = 1;
                #print("added waypoint")
                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:
                            print("EXEcuting...")   
                            plan = planner.plan_to_pose(waypoints, list() )
                            if not control.execute_path(plan):
                                raise Exception("Execution failed")                    
                        except Exception as e:
                            print e
                        else:
                            break
                    
                    #STOP = control._flag 
                    waypoints = list()
                    update = 0
                    print("EXEcute")
                    print(i)
                    i = i + 1
            j+=1
            print("END OF LOOP")



    zero(planner, control, waypoints)
    waypoints = list()
    print("ROBOT ZEROED")

    
    moveUp(planner, control, waypoints)
    waypoints = list()
    print("moved up")
        #moveDown()
        # waypoints = list()
        # print("moved down")
        # moveLeft()
        # waypoints = list()
        # print("moved left")        
        # moveRight()
        # waypoints = list()
        #print("moved right")

    zero(planner, control, waypoints)
    waypoints = list()
    control._zero = 0
    ## ZERO THE ROBOT ##
    print("ROBOT ZEROED...AGAIN")



"""        #plt.plot(control._sensor_data)
        s = list(map(float, control._sensor_data))
        scales = list(map(float, control._scalar_data))
        goal_line = list(map(float, control._goal_list))
        #s = list(map(float, control._debug))

        #control._sensor_data = sorted(control._sensor_data)
        ##print(min(s))
        ##print(max(s))
        plt.subplot(211)
        plt.plot(s)
        plt.plot(goal_line)

        plt.axis([0, len(control._sensor_data), min(s), max(s)])
        plt.ylabel('Sensor Data')
        plt.subplot(212)
        plt.plot(scales)
        plt.show()
        control._sensor_data = list()
        control._scalar_data = list()
        control._goal_list = list()
        waypoints = list()
        control._velocity_scalar = 1"""

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
