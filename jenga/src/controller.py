#!/usr/bin/env python
"""
Controller Class for Lab 8
Author: Valmik Prabhu, Chris Correa
"""

import rospy
import sys
import numpy as np
import itertools

import baxter_interface
import intera_interface

from moveit_msgs.msg import RobotTrajectory

#/ Arduino Code
import serial
import syslog
import time

#/ Arduino Code

#from test import Controller


#/ Arduino Code
port = '/dev/ttyACM1'
ard = serial.Serial(port,500000,timeout=5)
#/ Arduino Code

class Controller(object):
    """
    A controller object

    Fields:
    _Kp: 7x' ndarray of proportional constants
    _Ki: 7x' ndarray of integral constants
    _Kd: 7x' ndarray of derivative constants
    _Kw: 7x' ndarray of antiwindup constants
    _LastError: 7x' ndarray of previous position errors
    _LastTime: Time from start at which LastError was updated (in sec)
    _IntError: 7x' ndarray of integrated error values
    _path: a moveit_msgs/RobotTrajectory message
    _curIndex: the current index in the path
    _maxIndex: maximum index in the path
    _limb: baxter_interface.Limb or intera_interface.Limb

    _times: For Plotting
    _actual_positions: For Plotting
    _actual_velocities: For Plotting
    _target_positions: For Plotting
    _target_velocities: For Plotting

    Methods:
    __init__(self, Kp, Ki, Kd, Kw): constructor

    """

    def __init__(self, Kp, Ki, Kd, Kw, limb):
        """
        Constructor:

        Inputs:
        Kp: 7x' ndarray of proportional constants
        Ki: 7x' ndarray of integral constants
        Kd: 7x' ndarray of derivative constants
        Kw: 7x' ndarray of antiwindup constants
        limb: baxter_interface.Limb or sawyer_interface.Limb
        """

        # If the node is shutdown, call this function
        rospy.on_shutdown(self.shutdown)

        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kw = Kw

        self._LastError = np.zeros(len(Kd))
        self._LastTime = 0;
        self._IntError = np.zeros(len(Ki))

        self._path = RobotTrajectory()
        self._curIndex = 0;
        self._maxIndex = 0;

        self._limb = limb

        # For Plotting:
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()

        self._sensor_count = 0
        self._sensor_data = list()
        self._sensor_last = 0
        self._velocity_scalar = 0.4
        self._scalar_data = list()
        self._goal_list = list()
        self._delta = 0
        self._trigger = 0
        self._flag = 0
        self._zero = 0

        self._probing = 0
        self._poke_data = list()

    
    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), np.zeros(len(self._limb.joint_names())))))
       # rospy.sleep(0.1)

    def execute_path(self, path, timeout=2000.0, log=True):
        """
        Execute a given path

        Inputs:
        path: a moveit_msgs/RobotTrajectory message
        timeout: max time the controller will run
        log: should the controller display a plot
        
        """
        i = 0

        self._path = path

        self._curIndex = 0
        self._maxIndex = len(self._path.joint_trajectory.points)-1

        startTime = rospy.Time.now()

        # Set the last error as zero for t = 0
        self._LastError = np.zeros(len(self._Kd))
        self._LastTime = 0.0

        # Set the integral of positions to zero
        self._IntPos = np.zeros(len(self._Ki))

        # Reset plot values
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()
        self._debug = list()
        
        r = rospy.Rate(200)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - startTime).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), np.zeros(len(self._limb.joint_names())))))
                return False

            # Get the input for this time
            u = self.step_control(t)

            # Set the joint velocities
            self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), u)))
            # Sleep for a defined time (to let the robot move)
            r.sleep()

            # Once the end of the path has been reached, stop moving and break
            if self._curIndex >= self._maxIndex:
                # Set velocities to zero
                self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), np.zeros(len(self._limb.joint_names())))))
                break

        #ARDUINO CODE
        #print(self._sensor_data)
        #ARDUINO CODE
        #return self._sensor_data

        return True

    def error_percent(self, goal, actual):
        p_error =  ((float(goal) - actual) / goal)  
        railed_error = p_error
        return railed_error

    #def error_to_velocity(error, velocity)
    #    if (error)

    def rail_velocity(self, velocity):
        if(velocity > 0.4):
            velocity = 0.4

        if(velocity < 0.1):
            velocity = 0.1

        return velocity


    def step_control(self, t):
        """
        Return the control input given the current controller state at time t

        Inputs:
        t: time from start in seconds

        Output:
        u: 7x' ndarray of velocity commands
        
        """
        # Make sure you're using the latest time
        while (not rospy.is_shutdown() and self._curIndex < self._maxIndex and self._path.joint_trajectory.points[self._curIndex+1].time_from_start.to_sec() < t+0.001):
            self._curIndex = self._curIndex+1


        current_position = np.array(self._limb.joint_angles().values()).squeeze()
        current_velocity = np.array(self._limb.joint_velocities().values()).squeeze()

        if self._curIndex < self._maxIndex:
            time_low = self._path.joint_trajectory.points[self._curIndex].time_from_start.to_sec()
            time_high = self._path.joint_trajectory.points[self._curIndex+1].time_from_start.to_sec()

            target_position_low = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity_low = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

            target_position_high = np.array(self._path.joint_trajectory.points[self._curIndex+1].positions)
            target_velocity_high = np.array(self._path.joint_trajectory.points[self._curIndex+1].velocities)

            target_position = target_position_low + (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)

        else:
            target_position = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

        # For Plotting
        self._times.append(t)
        self._actual_positions.append(current_position)
        self._actual_velocities.append(current_velocity)
        self._target_positions.append(target_position)
        self._target_velocities.append(target_velocity)



#/ Arduino Code
        ard.flush()
        time.sleep(.04) 
        msg = ard.read(ard.inWaiting()) # read all characters in buffer
        number = msg.split(",")
        if(number[0] != ''):
            try:
                float(number[0])
            except Exception as e:
                print e
            else:
                self._sensor_data.append(number[0]) 
                if(float(number[0]) < -30):
                    self._trigger = 1
                if((float(number[0]) > -2) and self._trigger == 1):
                    self._flag = 1;
                    #print("FLAG UP")





#/ Arduino Code

        #self._sensor_last = self._sensor_data[-1]
        # Feed Forward Term
    

        # Controller Code

        Kp = .25
        goal = -30
        
        sensor_value = float(self._sensor_data[-1])

        # Velocity LOCK
        #u = target_velocity 

        self._velocity_scalar = self._velocity_scalar  + (Kp* (self.error_percent(goal, sensor_value)))
        self._velocity_scalar = self.rail_velocity(self._velocity_scalar)
        
        if(self._flag == 1):
            self._velocity_scalar = 0

        if(self._zero == 1):
            self._velocity_scalar = 1


        if(self._probing == 1):
            self._velocity_scalar = 1


        u = target_velocity * self._velocity_scalar

        self._scalar_data.append(self._velocity_scalar)
        self._goal_list.append(goal)
        #print("U list: ")
        #print(u)

        #self._debug.append(velocity_scalar)
        #u = target_velocity 
        # Velocity LOCK


        ###################### YOUR CODE HERE #########################
        ###################### YOUR CODE HERE #########################

        # Note, you should load the Kp, Ki, Kd, and Kw constants with
        # self._Kp
        # and so on. This is better practice than hard-coding

        ##error = target_force + float(self._sensor_last)

       # self._IntError = self._Kw*self._IntError + error
       # dt = t - self._LastTime
       # ed = (error - self._LastError)/dt

        ##u = u - self._Kp*error# - self._Ki*self._IntError - self._Kd*ed

        #self._LastTime = t
        #self._LastError = error

        ###################### YOUR CODE END ##########################
        ###################### YOUR CODE END ##########################

        #print(self._sensor_count)
        self._sensor_count += 1
        
        return u


if __name__ == '__main__': 
    Kp = 1.8 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.1 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned



    





