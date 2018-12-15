#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import tf2_ros
from geometry_msgs.msg import TransformStamped


#Python's syntax for a main() method
if __name__ == '__main__':

    rospy.init_node('test', anonymous=True)

    tfBuffer = tf2_ros.Buffer()

    tfListener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base', 'right_gripper_base', rospy.Time())
            print(trans.transform.translation.x)
            print(trans.transform.translation.y)
            print(trans.transform.translation.z)
            print(trans.transform.rotation.x)
            print(trans.transform.rotation.y)
            print(trans.transform.rotation.z)
            print(trans.transform.rotation.w)



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

            continue
'''
header: 
  seq: 0
  stamp: 
    secs: 1544585843
    nsecs: 429761171
  frame_id: base
child_frame_id: right_gripper_base
transform: 
  translation: 
    x: 0.439384292247
    y: 0.156102240439
    z: -0.137808570009
  rotation: 
    x: 4.37652276639e-05
    y: -0.706617896396
    z: 0.000308939557265
    w: -0.707595259406
header: 


 base right_gripper_base
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('turtle2', 'carrot1', rospy.Time.now())

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
        rate.sleep()
        continue

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('turtle2', 'carrot1', rospy.Time.now(), rospy.Duration(1.0))

'''





'''
    tfBuffer = tf2_ros.Buffer()

    tfListener = tf2_ros.TransformListener(tfBuffer)

    trans = tfBuffer.lookup_transform('left_hand', 'base', rospy.Time())

    listener()
'''











