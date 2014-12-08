import rospy
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

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
    print(">>>>>>>>>>>>Initializeing Baxter and nodes...")
    rospy.init_node("gripper_controller")

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

    rospy.sleep(5.0)
    #left_gripper.close()
    right_gripper.close()

    rospy.sleep(5.0)
    #left_gripper.open()
    right_gripper.open()
if __name__ == '__main__':
	baxter_Init()
    main()


