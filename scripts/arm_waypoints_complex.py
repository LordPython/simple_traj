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

    print("Getting robot state")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    print("Enabling robot")
    rs.enable()

    print("Initializing Gripper")
    Gripper_Init()

def left_neutral():
    e0= -0.016106798254394532
    e1= 1.8998352036254884
    s0= -0.7340098061645508
    s1= -0.33364082098388675
    w0= -0.03796602445678711
    w1= -0.05330583231811524
    w2= 0.02569417816772461
    move = {'left_s0': s0, 'left_s1': s1, 'left_e0': e0, 'left_e1': e1, 'left_w0': w0, 'left_w1': w1, 'left_w2': w2}
    return move

def right_neutral():
    e0= -0.013038836682128907
    e1=  1.8419274289489747
    s0= 0.7489661188293457
    s1= -0.20133497817993165
    w0= 0.00843689432373047
    w1= -0.15148060263061525
    w2= -0.04141748122558594
    move = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}
    return move

def left_point1(): # Line up to grab object
    e0= -0.13805827075195312
    e1= 1.8568837416137696
    s0= -0.4394854952270508
    s1= -0.3083301380126953
    w0= 0.26039323844604495
    w1= -1.3522040629760743
    w2= 0.0
    move = {'left_s0': s0, 'left_s1': s1, 'left_e0': e0, 'left_e1': e1, 'left_w0': w0, 'left_w1': w1, 'left_w2': w2}
    return move

def left_point2(): # Move to object location
    e0= -0.12540292926635743
    e1= 1.5228594254333496
    s0= -0.48090297645263674
    s1= -0.18599517031860352
    w0= 0.2876213973999024
    w1= -1.2616991965942383
    w2= -0.05330583231811524
    move = {'left_s0': s0, 'left_s1': s1, 'left_e0': e0, 'left_e1': e1, 'left_w0': w0, 'left_w1': w1, 'left_w2': w2}
    return move

def left_point3(): # Lift up object
    e0= -0.21322332927246096
    e1= 1.7280293555786135
    s0= -0.42184471618652347
    s1= -0.3838786917297364
    w0= 0.2653786760009766
    w1= -1.2528788070739747
    w2= -0.018024274237060548
    move = {'left_s0': s0, 'left_s1': s1, 'left_e0': e0, 'left_e1': e1, 'left_w0': w0, 'left_w1': w1, 'left_w2': w2}
    return move

def left_point4(): # Move to object out of shelf
    e0= -0.3528155808105469
    e1= 2.128014845562744
    s0= -0.122718462890625
    s1= -0.26806314237670903
    w0= 0.5553010445800781
    w1= -1.5711798201965332
    w2= 0.2795679982727051
    move = {'left_s0': s0, 'left_s1': s1, 'left_e0': e0, 'left_e1': e1, 'left_w0': w0, 'left_w1': w1, 'left_w2': w2}
    return move

def left_point5(): # Move to handoff position for left arm
    e0= -1.2889273555480958
    e1= 1.8327235442321779
    s0= 0.43565054326171876
    s1= 0.6998787336730957
    w0= 0.6527088244995117
    w1= 0.8084078742919922
    w2= 0.0732475825378418
    move = {'left_s0': s0, 'left_s1': s1, 'left_e0': e0, 'left_e1': e1, 'left_w0': w0, 'left_w1': w1, 'left_w2': w2}
    return move

def right_point1(): # Move to handoff position for right arm
    e0= 0.5065971546203614
    e1= 1.574247781768799
    s0= -0.25310682971191406
    s1= -0.3271214026428223
    w0= 0.5503156070251465
    w1= 1.566961373034668
    w2= -0.9828981887145997
    move = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}
    return move

def right_point2(): # Move to grab position for right arm
    e0= 0.5825292035339356
    e1= 1.7656118848388673
    s0= -0.12616991965942384
    s1= -0.2665291615905762
    w0= 0.6312330934936524
    w1= 1.2877768699584962
    w2= -1.125174906628418
    move = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}
    return move

def right_point3(): # Move to dropoff position
    e0= 0.22396119477539064
    e1= 0.7819467057312012
    s0= -0.3915485956604004
    s1= 0.5717913380310059
    w0= 0.5457136646667481
    w1= -0.2577087720703125
    w2= -0.8620972018066407
    move = {'right_s0': s0, 'right_s1': s1, 'right_e0': e0, 'right_e1': e1, 'right_w0': w0, 'right_w1': w1, 'right_w2': w2}
    return move






def main():
    global left_gripper
    global right_gripper

# Initialize baxter
    baxter_Init()

    # set up arm variables
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')

    # Move to neutral position for both arms
    #right.move_to_joint_positions(right_neutral())
    left.move_to_joint_positions(left_neutral())

    # Line up to grab object
    left.move_to_joint_positions(left_point1())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move to object location
    #left.move_to_joint_positions(left_point2())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Close left gripper
    #left_gripper.close()
    # sleep 3 secs
    #rospy.sleep(2.0)
    # Lift up object
    #left.move_to_joint_positions(left_point3())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move out of shelf
    #left.move_to_joint_positions(left_point4())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move to handoff position for left arm
    #left.move_to_joint_positions(left_point5())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move to handoff position for right arm
    #right.move_to_joint_positions(right_point1())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move to grab position for right arm
    #right.move_to_joint_positions(right_point2())
    # sleep 3 secs
    #rospy.sleep(1.0)
    # Close right gripper
    #right_gripper.close()
    # sleep
    #rospy.sleep(1.0)
    # Open left gripper
    #left_gripper.open()
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move to handoff position
    #right.move_to_joint_positions(right_point1())
    # sleep 1 secs
    #rospy.sleep(1.0)
    # Move to dropoff position
    #right.move_to_joint_positions(right_point3())
    # sleep
    #rospy.sleep(1.0)
    # Open right gripper
    #right_gripper.open()


    print "Task Done :-)"




    '''
    left_gripper.open()
    rospy.sleep(4.0)
    left_gripper.close()
    rospy.sleep(4.0)
    left_gripper.open()

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
    '''

if __name__ == '__main__':
    sys.exit(main())







