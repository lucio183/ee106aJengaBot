#/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION





def map_keyboard(side):
    delay = rospy.Rate(10)
    limb = intera_interface.Limb(side)

    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True

    joints = limb.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    def set_g(action):
        if has_gripper:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    def parser(input):
        angles = input.split()
        output = input.split()
        i = 0
        for angle in angles:
            output[i] = float(angle)
            if(output[i] > 1):
                output[i] = 0
            if(output[i] < -1):
                output[i] = 0
            i += 1    
        return output

    def delta_check(angles):
        output = 0
        i = 0
        for angle in angles:
            print("Goal angle", angle)
            print("Current angle", limb.joint_angle(joints[i]))
            if ( ((angle - 0.1) >= limb.joint_angle(joints[i])) ):
                index = (i*2) + 1
                cmd = bindings[index]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            elif (((angle + 0.1) <= limb.joint_angle(joints[i])) ):
                index = (i*2) + 2
                cmd = bindings[index]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                output += 1

            i += 1

        if(output == 7):
            return True
        else:
            return False


    bindings = {
        1 : (set_j, [limb, joints[0], 0.1], joints[0]+" increase"),
        2 : (set_j, [limb, joints[0], -0.1], joints[0]+" decrease"),
        3 : (set_j, [limb, joints[1], 0.1], joints[1]+" increase"),
        4 : (set_j, [limb, joints[1], -0.1], joints[1]+" decrease"),
        5 : (set_j, [limb, joints[2], 0.1], joints[2]+" increase"),
        6 : (set_j, [limb, joints[2], -0.1], joints[2]+" decrease"),
        7 : (set_j, [limb, joints[3], 0.1], joints[3]+" increase"),
        8 : (set_j, [limb, joints[3], -0.1], joints[3]+" decrease"),
        9 : (set_j, [limb, joints[4], 0.1], joints[4]+" increase"),
        10 : (set_j, [limb, joints[4], -0.1], joints[4]+" decrease"),
        11 : (set_j, [limb, joints[5], 0.1], joints[5]+" increase"),
        12 : (set_j, [limb, joints[5], -0.1], joints[5]+" decrease"),
        13 : (set_j, [limb, joints[6], 0.1], joints[6]+" increase"),
        14 : (set_j, [limb, joints[6], -0.1], joints[6]+" decrease")
     }

    if has_gripper:
        bindings.update({
        '8': (set_g, "close", side+" gripper close"),
        'i': (set_g, "open", side+" gripper open"),
        '9': (set_g, "calibrate", side+" gripper calibrate")
        })
    done = False
    print("Custom Controller")
    while not done and not rospy.is_shutdown():
        
        print("")
        print(str(joints[0]) + " " + str(limb.joint_angle(joints[0])))
        print(str(joints[1]) + " " + str(limb.joint_angle(joints[1])))
        print(str(joints[2]) + " " + str(limb.joint_angle(joints[2])))
        print(str(joints[3]) + " " + str(limb.joint_angle(joints[3])))
        print(str(joints[4]) + " " + str(limb.joint_angle(joints[4])))
        print(str(joints[5]) + " " + str(limb.joint_angle(joints[5])))
        print(str(joints[6]) + " " + str(limb.joint_angle(joints[6])))
        print("")
        

        c = raw_input("Insert 7 Angles \n")
        #print("\n")
        #c = intera_external_devices.getch()
        if c:
            angles = parser(c)
            print(angles)
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            else:
                delta_flag = False
                while(delta_flag == False):
                    output = 0
                    i = 0
                    for angle in angles:
                        print("Goal angle", angle)
                        print("Current angle", limb.joint_angle(joints[i]))
                        if ( ((angle - 0.1) >= limb.joint_angle(joints[i])) ):
                            index = (i*2) + 1
                            cmd = bindings[index]
                            cmd[0](*cmd[1])
                            print("command: %s" % (cmd[2],))
                            #delay.sleep()
                        elif (((angle + 0.1) <= limb.joint_angle(joints[i])) ):
                            index = (i*2) + 2
                            cmd = bindings[index]
                            cmd[0](*cmd[1])
                            print("command: %s" % (cmd[2],))
                            #delay.sleep()
                        else:
                            output += 1

                        i += 1

                    if(output == 7):
                        delta_flag = True
                    else:
                        delta_flag = False
                    delay.sleep()
                    print("False")


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
