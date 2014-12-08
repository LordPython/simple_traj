#!/usr/bin/env python

import argparse
import struct
import sys
import rospy
import math
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion,
)
from gazebo_msgs.msg import (
    WorldState
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from baxter_core_msgs.msg import (
    EndpointState
)

# right input
rightInputX = .77
rightInputY = -0.46
rightInputZ = -0.07

rightInputXR = 0.0
rightInputYR = 0.71
rightInputZR = 0.0
rightInputWR = 0.71

# left input
leftInputX = 0.6
leftInputY = 0.4
leftInputZ = 0.5

leftInputXR = -0.4
leftInputYR = 0.92
leftInputZR = 0
leftInputWR = 0.02

'''
rostopic pub /simple_traj/target geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "base"}, pose: {position: {x: 0.77, y: -0.46, z: -0.07}, orientation: {x: 0.0, y: 0.70710678118, z: 0.0, w: 0.70710678118}}}'
'''


def main():
    rospy.init_node("Goal_publisher")
    rightHandPub = rospy.Publisher('/simple_traj/target', PoseStamped, queue_size=10)
    leftHandPub = rospy.Publisher('/left_arm_pose_controller/target', PoseStamped, queue_size=10)

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    leftPose = PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=leftInputX,
                y=leftInputY,
                z=leftInputZ,
            ),
            orientation=Quaternion(
                x=leftInputXR,
                y=leftInputYR,
                z=leftInputZR,
                w=leftInputWR,
            ),
        ),
    )

    rightPose = PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=rightInputX,
                y=rightInputY,
                z=rightInputZ,
            ),
            orientation=Quaternion(
                x=rightInputXR,
                y=rightInputYR,
                z=rightInputZR,
                w=rightInputWR,
            ),
        ),
    )

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    if args.limb == 'left':
        while True:
            print "publishing left hand..."
            leftHandPub.publish(leftPose)
            rospy.sleep(.1)

    if args.limb == 'right':
        while True:
            print "publishing right hand..."
            rightHandPub.publish(rightPose)
            rospy.sleep(.1)

    return 0


if __name__ == '__main__':
    main()
    rospy.spin()
