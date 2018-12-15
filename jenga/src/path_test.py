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

#import matplotlib.pyplot as plt
from matplotlib import colors





def grid_fill(block, command, data):
    # 21 BLOCKS

    fill_color = 0

    if (command == "red"):
        fill_color = 10

    if (command == "green"):
        fill_color = 20
    
    if (command == "removed"):
        fill_color = 0


    if(block == 1):
        data[14][8] = fill_color

    if(block == 2):
        data[14][7] = fill_color

    if(block == 3):
        data[14][6] = fill_color



    if(block == 4):
        data[12][8] = fill_color

    if(block == 5):
        data[12][7] = fill_color

    if(block == 6):
        data[12][6] = fill_color



    if(block == 7):
        data[10][8] = fill_color

    if(block == 8):
        data[10][7] = fill_color

    if(block == 9):
        data[10][6] = fill_color



    if(block == 10):
        data[8][8] = fill_color

    if(block == 11):
        data[8][7] = fill_color

    if(block == 12):
        data[8][6] = fill_color



    if(block == 13):
        data[6][8] = fill_color

    if(block == 14):
        data[6][7] = fill_color

    if(block == 15):
        data[6][6] = fill_color



    if(block == 16):
        data[4][8] = fill_color

    if(block == 17):
        data[4][7] = fill_color

    if(block == 18):
        data[4][6] = fill_color



    if(block == 19):
        data[2][8] = fill_color

    if(block == 20):
        data[2][7] = fill_color

    if(block == 21):
        data[2][6] = fill_color


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

    def moveUp(planner, control, waypoints , value_of_step):

        control._probing = 1
        #STARTOF WORKING CODE
        # Z MUST MOVE UP TWO BLOCK
        # Z = 0.025
        #control._velocity_scalar = 0.5
        jenga = 0
        while (jenga != 1):
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
            FINAL_Z = goal_z + value_of_step # + offsett
            z = tf_pz #+ offset
            #print("TF'd zero")
            update = 0
            execute = 0
            i = 0
            STOP = 0
            while ((z < FINAL_Z or update == 0 ) and (i < 3)):
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
                        z = tf_pz# + offset
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue
                    update = 1
                    execute = 0
                y = tf_py
                x = tf_px + offset
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
                z = z + step_size
                if (z >= FINAL_Z ):
                    execute = 1;
                #print("added waypoint")
                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:    

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
                    #print("EXEcute")
                    #print(i)
                    i = i + 1
            #print("END OF LOOP")                       
            jenga = 1
        control._probing = 0


        # END OF WORKING CODE

    def moveDown(planner, control, waypoints):
        control._probing = 1
        #STARTOF WORKING CODE
        # Z MUST MOVE UP TWO BLOCK
        # Z = 0.025
        value_of_step = 0.033
        #control._velocity_scalar = 0.5
        jenga = 0
        while (jenga != 1):
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
            FINAL_Z = goal_z - value_of_step # + offsett
            z = tf_pz #+ offset
            #print("TF'd zero")
            update = 0
            execute = 0
            i = 0
            STOP = 0
            while ((z > FINAL_Z or update == 0 ) and (i < 3)):
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
                        z = tf_pz# + offset
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue
                    update = 1
                    execute = 0
                y = tf_py
                x = tf_px + offset
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
                z = z - step_size
                if (z <= FINAL_Z ):
                    execute = 1;
                #print("added waypoint")
                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:    

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
                    #print("EXEcute")
                    #print(i)
                    i = i + 1
            #print("END OF LOOP")                       
            jenga = 1
        control._probing = 0
        # END OF WORKING CODE

    def moveLeft(planner, control, waypoints, value_of_step):
        control._probing = 1
        #STARTOF WORKING CODE
        # Z MUST MOVE UP TWO BLOCK
        # Z = 0.025
        #value_of_step = 0.014
        #control._velocity_scalar = 0.5
        jenga = 0
        while (jenga != 1):
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
                #print(goal_z)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue    
            offset = 0.14
            FINAL_Y = goal_y + value_of_step # + offsett
            y = tf_py #+ offset
            
            update = 0
            execute = 0
            i = 0
            STOP = 0
            while ((y < FINAL_Y or update == 0 ) and (i < 3)):
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
                        y = tf_py# + offset
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue
                    update = 1
                    execute = 0
                
                x = tf_px + offset
                z = tf_pz

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
                y = y + step_size
                if (y >= FINAL_Y ):
                    execute = 1;
                #print("added waypoint")
                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:    

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
                    #print("EXEcute")
                    #print(i)
                    i = i + 1
            #print("END OF LOOP")                       
            jenga = 1
        control._probing = 0
        # END OF WORKING CODE

    def moveRight(planner, control, waypoints):
        control._probing = 1
        #STARTOF WORKING CODE
        # Z MUST MOVE UP TWO BLOCK
        # Z = 0.025
        value_of_step = 0.014
        #control._velocity_scalar = 0.5
        jenga = 0
        while (jenga != 1):
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
            FINAL_Y = goal_y - value_of_step 
            y = tf_py #+ offset
            #print("TF'd zero")
            update = 0
            execute = 0
            i = 0
            STOP = 0
            while ((y > FINAL_Y or update == 0 ) and (i < 3)):
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
                        y = tf_py# + offset
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue
                    update = 1
                    execute = 0
                z = tf_pz
                x = tf_px + offset

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
                y = y - step_size
                if (y <= FINAL_Y ):
                    execute = 1;
                #print("added waypoint")
                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:    

                            plan = planner.plan_to_pose(waypoints, list() )

                            if not control.execute_path(plan):
                                raise Exception("Execution failed")                    
                        except Exception as e:
                            print e
                        else:
                            break
                    
                    #STOP = control._flag 
                    print("LAST WAYPOINT")
                    print(waypoints[-1].position)
                    waypoints = list()
                    update = 0
                    #print("EXEcute")
                    #print(i)
                    i = i + 1
            #print("END OF LOOP")                       
            jenga = 1
        control._probing = 0
        # END OF WORKING CODE

    def probe(planner, control, waypoints):
        control._probing = 1
        value_of_step = 0.004

        step_size = 0.0005 #0.0025
        goal_x = 0.585
        goal_y = 0.156
        goal_z = -0.138        
        jenga = 0
        while (jenga != 1):
            
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
            FINAL_X = goal_x + offset + value_of_step
            x = tf_px + offset
            update = 0
            execute = 0
            STOP = 0
            while ((x < FINAL_X or update == 0 ) ):
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
                        x = tf_px + offset
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue
                    update = 1
                    execute = 0

                y = tf_py
                z = tf_pz
                if (y < goal_y - 0.05):
                    y = y + step_size
                if (y > goal_y + 0.05):
                    y = y - step_size
                if (z < goal_z - 0.05):
                    z = z + step_size
                if (z > goal_z + 0.05):
                    z = z - step_size     

                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z
                
                goal_1.pose.orientation.x =  0        # -0.026
                goal_1.pose.orientation.y =  0.707    #  0.723
                goal_1.pose.orientation.z =  0        # -0.052
                goal_1.pose.orientation.w =  0.707    #  0..

                #print("added a point")
                waypoints.append(goal_1.pose)
                x = x + step_size
                if (x >= FINAL_X ):
                    execute = 1
                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:    
                            #print("Executing Probe")

                            plan = planner.plan_to_pose(waypoints, list() )
                            if not control.execute_path(plan):
                                raise Exception("Execution failed")                    
                        except Exception as e:
                            print e
                        else:
                            break


            jenga = 1



        waypoints = waypoints[::-1]


        goal_1 = PoseStamped()
        goal_1.header.frame_id = "base"
        goal_1.pose.position.x = goal_x + offset
        goal_1.pose.position.y = goal_y
        goal_1.pose.position.z = goal_z
                
        goal_1.pose.orientation.x =  0        # -0.026
        goal_1.pose.orientation.y =  0.707    #  0.723
        goal_1.pose.orientation.z =  0        # -0.052
        goal_1.pose.orientation.w =  0.707    #  0..

        #waypoints.append(goal_1.pose)



        while not rospy.is_shutdown():
            try:    
                plan = planner.plan_to_pose(waypoints, list() )
                if not control.execute_path(plan):
                    raise Exception("Execution failed")                    
            except Exception as e:
                print e
            else:
                break
                     


        print("PROBE TEST")
        print(min(list(map(float, control._sensor_data))))
        control._poke_data.append(min(list(map(float, control._sensor_data))))
        control._sensor_data = list()
        control._scalar_data = list()
        control._probing = 0

    def zero_spot(planner, control, waypoints):
        control._probing = 1
        jenga = 0
        while (jenga != 1):
            step_size = 0.0005 #0.0025
            FINAL_X = 0.585
            FINAL_Y = 0.156
            FINAL_Z = -0.138

            offset = 0.14

            tf_px = 0.585
            tf_py = 0.156
            tf_pz = -0.138

            goal_x = 0.585
            goal_y = 0.156
            goal_z = -0.138

            x_done = 0
            y_done = 0
            z_done = 0

            try:
                trans = tfBuffer.lookup_transform('base', 'right_gripper_base', rospy.Time())
                tf_px = trans.transform.translation.x
                tf_py = trans.transform.translation.y
                tf_pz = trans.transform.translation.z

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue 


            offset = 0.14
            
            x = tf_px + offset
            y = tf_py
            z = tf_pz

            update = 0
            execute = 0
            i = 0

            while (  (  (x_done == 0 or y_done == 0 or z_done == 0) or update == 0 ) and (i < 3)):
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
                        x = tf_px + offset
                        y = tf_py
                        z = tf_pz

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue
                    update = 1
                    execute = 0


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

                print(x)
                if (x < goal_x + offset - 0.05):
                    x = x + step_size
                    print("add")
                elif (x > goal_x + offset + 0.05):
                    x = x - step_size
                    print("sub")

                else:
                    x_done = 1
                    print("xdone")

                if (y < goal_y - 0.05):
                    y = y + step_size
                elif (y > goal_y + 0.05):
                    y = y - step_size
                else:
                    y_done = 1
                    print("ydone")


                if (z < goal_z - 0.05):
                    z = z + step_size
                elif (z > goal_z + 0.05):
                    z = z - step_size
                else:
                    z_done = 1
                    print("zdone")



                if (x_done == 1 and y_done == 1 and z_done == 1) :
                    execute = 1;
                    print("execute done")


                if(execute == 1 ):
                    while not rospy.is_shutdown():
                        try:    

                            plan = planner.plan_to_pose(waypoints, list() )
                            print("EXECUTED ZERO")
                            if not control.execute_path(plan):
                                raise Exception("Execution failed")                    
                        except Exception as e:
                            print e
                        else:
                            break
                    
                    #STOP = control._flag 
                    waypoints = list()
                    update = 0
                    #print("EXEcute")
                    #print(i)
                    i = i + 1
            #print("END OF LOOP")                       
            jenga = 1
        control._probing = 0
        # END OF WORKING CODE

    while not rospy.is_shutdown():

        value_of_step_up = 0.023
        value_of_step_left = 0.0125

        story = 0

        while (story < 1):
            control._probing = 1

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

            waypoints = list()
            ## ZERO THE ROBOT ##
            print("ROBOT ZEROED")
            #ospy.sleep(2)

            bump = 0
            while (story - bump > 0) :
                if (bump > 2):
                    value_of_step_up = 0.033
                else:
                    value_of_step_up = 0.025

                moveUp(planner, control, waypoints, value_of_step_up)
                waypoints = list()
                rospy.sleep(2)

                bump += 1

            probe(planner, control, waypoints)
            waypoints = list()
            rospy.sleep(2)


            moveLeft(planner, control, waypoints, value_of_step_left)
            waypoints = list()
            rospy.sleep(2)

            probe(planner, control, waypoints)
            waypoints = list()
            rospy.sleep(2)

            moveLeft(planner, control, waypoints, value_of_step_left)
            waypoints = list()
            rospy.sleep(2)

            probe(planner, control, waypoints)
            waypoints = list()
            rospy.sleep(2)


            story += 1

        control._probing = 1

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
        waypoints = list()
        ## ZERO THE ROBOT ##
        print("ROBOT ZEROED")
        #rospy.sleep(2)
        control._probing = 0
        control._trigger = 0
        control._flag = 0
        control._zero = 0




   


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
        FINAL_X = goal_x + offset + 0.2

        x = tf_px + offset
        print("TF'd zero")

        update = 0
        execute = 0

        i = 0
        STOP = 0

        while ((x < FINAL_X or update == 0 ) and (i < 5)):


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
                    x = tf_px + offset
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
                update = 1
                execute = 0



            y = tf_py
            z = tf_pz

            if (y < goal_y - 0.05):
                y = y + step_size
            
            if (y > goal_y + 0.05):
                y = y - step_size


            if (z < goal_z - 0.05):
                z = z + step_size
            
            if (z > goal_z + 0.05):
                z = z - step_size      

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

            x = x + step_size

            if (x >= FINAL_X ):
                execute = 1;
            #print("added waypoint")

            if(execute == 1 ):
                while not rospy.is_shutdown():
                    try:    
                        control._probing = 0


                        plan = planner.plan_to_pose(waypoints, list() )
                        print("TAKING BLOCK OUT EXECUTE")
                        if not control.execute_path(plan):
                            raise Exception("Execution failed")                    
                    except Exception as e:
                        print e
                    else:
                        break
                
                #STOP = control._flag 
                waypoints = list()



                update = 0
                #print("EXEcute")
                #print(i)
                i = i + 1
        print("END OF LOOP")     


        control._probing = 0
        control._trigger = 0
        control._flag = 0
        control._zero = 1

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
        waypoints = list()
        ## ZERO THE ROBOT ##
        print("ROBOT ZEROED")
        #ospy.sleep(2)

        control._probing = 0
        control._trigger = 0
        control._flag = 0
        control._zero = 0



        moveLeft(planner, control, waypoints, value_of_step_left)
        waypoints = list()
        rospy.sleep(2)
        

        moveLeft(planner, control, waypoints, value_of_step_left)
        waypoints = list()
        rospy.sleep(2)



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
        FINAL_X = goal_x + offset + 0.2

        x = tf_px + offset
        print("TF'd zero")

        update = 0
        execute = 0

        i = 0
        STOP = 0

        while ((x < FINAL_X or update == 0 ) and (i < 5)):


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
                    x = tf_px + offset
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
                update = 1
                execute = 0



            y = tf_py
            z = tf_pz

            if (y < goal_y - 0.05):
                y = y + step_size
            
            if (y > goal_y + 0.05):
                y = y - step_size


            if (z < goal_z - 0.05):
                z = z + step_size
            
            if (z > goal_z + 0.05):
                z = z - step_size      

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

            x = x + step_size

            if (x >= FINAL_X ):
                execute = 1;
            #print("added waypoint")

            if(execute == 1 ):
                while not rospy.is_shutdown():
                    try:    
                        control._probing = 0


                        plan = planner.plan_to_pose(waypoints, list() )
                        print("TAKING BLOCK OUT EXECUTE")
                        if not control.execute_path(plan):
                            raise Exception("Execution failed")                    
                    except Exception as e:
                        print e
                    else:
                        break
                
                #STOP = control._flag 
                waypoints = list()



                update = 0
                #print("EXEcute")
                #print(i)
                i = i + 1
        print("END OF LOOP")     


        control._probing = 0
        control._trigger = 0
        control._flag = 0
        control._zero = 1

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
        waypoints = list()
        ## ZERO THE ROBOT ##
        print("ROBOT ZEROED")
        #ospy.sleep(2)

        control._probing = 0
        control._trigger = 0
        control._flag = 0
        control._zero = 0
      
        #waypoints = list()
        #zero_spot(planner, control, waypoints)
        #waypoints = list()
        
        #rospy.sleep(2)




        #waypoints = list()
        
        #rospy.sleep(2)

        #zero_spot(planner, control, waypoints)
        #waypoints = list()
        
        #rospy.sleep(2)




        # #probe(planner, control, waypoints)
        # waypoints = list()

        # moveUp(planner, control, waypoints)
        # waypoints = list()

        # #probe(planner, control, waypoints)
        # waypoints = list()

        # moveLeft(planner, control, waypoints)
        # waypoints = list()

        # #probe(planner, control, waypoints)
        # waypoints = list()

        # moveLeft(planner, control, waypoints)
        # waypoints = list()
        
        # #probe(planner, control, waypoints)
        # waypoints = list()





        #plt.plot(control._sensor_data)
        s = list(map(float, control._sensor_data))
        scales = list(map(float, control._scalar_data))
        goal_line = list(map(float, control._goal_list))
        #s = list(map(float, control._debug))

        #control._sensor_data = sorted(control._sensor_data)
        #print(min(s))
        #print(max(s))
        plt.subplot(211)
        plt.plot(s)
        plt.plot(goal_line)

        plt.axis([0, len(control._sensor_data), min(s), max(s)])
        plt.ylabel('Sensor Data')
        plt.subplot(212)
        plt.plot(scales)
        plt.show()



# PRINT GRID








        control._sensor_data = list()
        control._scalar_data = list()
        control._goal_list = list()
        waypoints = list()
        control._velocity_scalar = 1


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
