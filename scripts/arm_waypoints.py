import argparse
import struct
import sys

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_external_devices

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

global left_gripper
global right_gripper

def Gripper_Init():
    global left_gripper
    global right_gripper
    left_gripper = baxter_interface.Gripper('left',CHECK_VERSION)
    right_gripper = baxter_interface.Gripper('right',CHECK_VERSION)
    if(left_gripper.calibrate()):
        print("Left Gripper Calibrated")
    if(right_gripper.calibrate()):
        print("Right Gripper Calibrated")

def baxter_Init():

    rospy.init_node("video_demo")

    print(">>>>>>>>>>>>Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    print(">>>>>>>>>>>>Enabling robot...")
    rs.enable()

    print(">>>>>>>>>>>>Initializing Gripper...")
    Gripper_Init()

def main():
    global left_gripper
    global right_gripper

    baxter_Init()
    #move = [5]
    # Forward Kinematics
    # Pt 1
    print "Executing Forward Kinematics pt1"
    e0=-0.0299126
    e1=1.66974
    s0=0.699495
    s1=-0.101626
    w0=0.0809175
    w1=-0.217442
    w2=0.0233932
    move1 = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}


    # Pt 2
    print "Executing Forward Kinematics pt2"
    e0 = -0.077466
    e1 = 1.20878
    s0 = 0.84484
    s1 = -0.0456359
    w0 = 0.0617427
    w1 = -1.1386
    w2 = -0.133456
    move2 = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}

    # Pt 3
    print "Executing Forward Kinematics pt3"
    e0 = -0.0801505
    e1 = 0.491257
    s0 = 0.880505
    s1 = 0.273816
    w0 = 0.170272
    w1 = -0.762388
    w2 = -0.157617
    move3 = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}

    # Pt 4
    print "Executing Forward Kinematics pt4"
    e0 = -0.077466
    e1 = 1.20878
    s0 = 0.84484
    s1 = -0.0456359
    w0 = 0.0617427
    w1 = -1.1386
    w2 = -0.133456
    move4 = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}

    # Pt 5
    print "Executing Forward Kinematics pt5"
    e0 = -0.168738
    e1 = 0.612825
    s0 = -0.22818
    s1 = 0.415709
    w0 = -2.58361
    w1 = -0.548398
    w2 = 0.0141893
    move5 = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}
    # Move arm
    limb = baxter_interface.Limb('right')
    limb.move_to_neutral()
    limb.move_to_joint_positions(move1)
    limb.move_to_joint_positions(move2)
    limb.move_to_joint_positions(move3)
    right_gripper.close()
    rospy.sleep(3.0)
    limb.move_to_joint_positions(move4)
    limb.move_to_joint_positions(move5)
    right_gripper.open()
    rospy.sleep(3.0)
    limb.move_to_joint_positions(move1)
    limb.move_to_neutral()

if __name__ == '__main__':
    sys.exit(main())







