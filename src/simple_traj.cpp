// Fellow Joint Trajectory Action
#include <ros/ros.h>

// traj command
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
// Gripper command
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>


// Goal Desire
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

// MoveIt! Kinematics
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Standard Library
#include <stdlib.h>
#include <iostream>

// Baxter Head File
#include <baxter_core_msgs/EndEffectorCommand.h>

#define _USE_MATH_DEFINES
#define JOINTS 7

#define RIGHT_ARM 1
#define LEFT_ARM 2

#define Right_Gripper 3
#define Left_Gripper 4

enum GripperCommand{
    CalibrateGripper,
    OpenGripper,
    CloseGripper
};

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;
typedef actionlib::SimpleActionClient< control_msgs::GripperCommandAction > GripClient;

struct HardCode_Jonit_Values
{
    double e0;
    double e1;
    double s0;
    double s1;
    double w0;
    double w1;
    double w2;
};

class RobotArm
{
private:
    // ROS HANDLE
    ros::NodeHandle nh_;
    // Subscriber to joint states
    ros::Subscriber joint_state_sub_;

    // Action client for the joint trajectory action
    TrajClient* Right_traj_client_;
    TrajClient* Left_traj_client_;
    // Robot Joint Name Storage
    std::vector<std::string> baxter_left_arm_joint_names;
    std::vector<std::string> baxter_right_arm_joint_names;
    // Current Joint State
    std::vector<double> Left_Current_Joint_State;
    std::vector<double> Right_Current_Joint_State;
    // Joint Model Group
    const robot_model::JointModelGroup* baxter_right_arm_group;
    const robot_model::JointModelGroup* baxter_left_arm_group;

    // Action Client for Gripper Command
    GripClient* Right_Gripper_client_;
    GripClient* Left_Gripper_client_;

    // Robot Kinecmatic state
    robot_state::RobotStatePtr kinematic_state;
    // Set up an internal robot model
    const robot_model::RobotModelConstPtr kinematic_model;

    // Endeffector pose test
    geometry_msgs::PoseStamped EE_pose;
    Eigen::Affine3d desired_pose;

public:
    //our traj goal variable
    control_msgs::FollowJointTrajectoryGoal goal_ToBeSend;

    // our gripper goal variable
    control_msgs::GripperCommandGoal gripper_goal;

    //! Initialize the action client and wait for action server to come up
    RobotArm(ros::NodeHandle& nh, const robot_model::RobotModelPtr& model) : nh_(nh), kinematic_model(model)
    {
        //=====================================================================
        // tell the action client that we want to spin a thread by default
        Right_traj_client_ = new TrajClient("/robot/limb/right/follow_joint_trajectory", true);
        Left_traj_client_ = new TrajClient("/robot/limb/left/follow_joint_trajectory", true);
        // wait for action server to come up
        while(   !Left_traj_client_->waitForServer(ros::Duration(5.0))
              || !Right_traj_client_->waitForServer(ros::Duration(5.0)))
        {
            std::cout<<"Waiting for the joint_trajectory_action server"<<std::endl;
        }
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        Right_Gripper_client_ = new GripClient("robot/end_effector/right_gripper/gripper_action", true);
        Left_Gripper_client_ = new GripClient("robot/end_effector/left_gripper/gripper_action", true);
        while(   !Right_Gripper_client_->waitForServer(ros::Duration(5.0))
              && !Left_Gripper_client_->waitForServer(ros::Duration(5.0)))
        {
            std::cout<< "Waiting for the Gripper_Action server" <<std::endl;
        }
        //=====================================================================
        // Get Joint Model Group
        baxter_right_arm_group = this->kinematic_model->getJointModelGroup("right_arm");
        baxter_left_arm_group = this->kinematic_model->getJointModelGroup("left_arm");
        // Get Joint Names
        baxter_left_arm_joint_names = this->GetLeftArmLinkNames();
        baxter_right_arm_joint_names = this->GetRightArmLinkNames();

        // Using Default values to initiate robot kinematic state
        kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();
        kinematic_state->update();

        // Register a subscriber and callback func to joint_state topic
        joint_state_sub_ = nh_.subscribe("/robot/joint_states", 1, &RobotArm::RobotStateCB,this);
        // Binding a publisher to gripper_command topic
        //Right_gripper_commander_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",1);
    }

    //! Clean up the action client
    ~RobotArm()
    {
        delete Right_traj_client_;
        delete Left_traj_client_;

        delete Right_Gripper_client_;
        delete Left_Gripper_client_;
    }

    /* CALL BACK */
    std::vector<std::string> GetLeftArmLinkNames()
        {
            std::vector<std::string> link_names(JOINTS);
            /*
            link_names[0] = "left_upper_shoulder";
            link_names[1] = "left_lower_shoulder";
            link_names[2] = "left_upper_elbow";
            link_names[3] = "left_lower_elbow";
            link_names[4] = "left_upper_forearm";
            link_names[5] = "left_lower_forearm";
            link_names[6] = "left_hand";
            */
            link_names[0] = "left_e0";
            link_names[1] = "left_e1";
            link_names[2] = "left_s0";
            link_names[3] = "left_s1";
            link_names[4] = "left_w0";
            link_names[5] = "left_w1";
            link_names[6] = "left_w2";

            return link_names;
        }
    std::vector<std::string> GetRightArmLinkNames()
        {
            std::vector<std::string> link_names(JOINTS);
            /*
            link_names[0] = "right_upper_shoulder";
            link_names[1] = "right_lower_shoulder";
            link_names[2] = "right_upper_elbow";
            link_names[3] = "right_lower_elbow";
            link_names[4] = "right_upper_forearm";
            link_names[5] = "right_lower_forearm";
            link_names[6] = "right_hand";
            */
            link_names[0] = "right_e0";
            link_names[1] = "right_e1";
            link_names[2] = "right_s0";
            link_names[3] = "right_s1";
            link_names[4] = "right_w0";
            link_names[5] = "right_w1";
            link_names[6] = "right_w2";
            return link_names;
        }

    void RobotStateCB(sensor_msgs::JointState msg)
      {
        std::map<std::string, double> joint_positions;
        for (size_t idx = 0; idx < msg.name.size(); idx++)
        {
            joint_positions[msg.name[idx]] = msg.position[idx];
        }
        /*
       // This is just for test
        for(std::map<std::string, double>::const_iterator it = joint_positions.begin(); it != joint_positions.end(); it++)
        {
            std::cout<<" Joint Name "<<it->first<<":"<< " value:"<< it->second << std::endl;
        }
        //ROS_INFO("************************************"); */

        // Get configuration in order
        std::vector<double> left_arm_config(JOINTS);
        std::vector<double> right_arm_config(JOINTS);
        left_arm_config[0] = joint_positions[baxter_left_arm_joint_names[0]];
        left_arm_config[1] = joint_positions[baxter_left_arm_joint_names[1]];
        left_arm_config[2] = joint_positions[baxter_left_arm_joint_names[2]];
        left_arm_config[3] = joint_positions[baxter_left_arm_joint_names[3]];
        left_arm_config[4] = joint_positions[baxter_left_arm_joint_names[4]];
        left_arm_config[5] = joint_positions[baxter_left_arm_joint_names[5]];
        left_arm_config[6] = joint_positions[baxter_left_arm_joint_names[6]];

        right_arm_config[0] = joint_positions[baxter_right_arm_joint_names[0]];
        right_arm_config[1] = joint_positions[baxter_right_arm_joint_names[1]];
        right_arm_config[2] = joint_positions[baxter_right_arm_joint_names[2]];
        right_arm_config[3] = joint_positions[baxter_right_arm_joint_names[3]];
        right_arm_config[4] = joint_positions[baxter_right_arm_joint_names[4]];
        right_arm_config[5] = joint_positions[baxter_right_arm_joint_names[5]];
        right_arm_config[6] = joint_positions[baxter_right_arm_joint_names[6]];

        // Update Current State
        kinematic_state->setJointGroupPositions(baxter_left_arm_group, left_arm_config);
        kinematic_state->setJointGroupPositions(baxter_right_arm_group, right_arm_config);
        /*
        Right_Current_Joint_State = right_arm_config;
        Left_Current_Joint_State = left_arm_config;
        */
      }

    //******************** For Arm Trajectory Command **********************
    Eigen::Affine3d SetTargetPos()
    {
        /*// For Right Arm test
        // Stamped Pose

        EE_pose.header.frame_id = "right_torso_itb";
        EE_pose.header.stamp = ros::Time::now();
        // Set pose msg
        EE_pose.pose.position.x = 0.84;
        EE_pose.pose.position.y = -0.43;
        EE_pose.pose.position.z = -0.11;
        EE_pose.pose.orientation.x = 0.07;
        EE_pose.pose.orientation.y = 0.93;
        EE_pose.pose.orientation.z = -0.01;
        EE_pose.pose.orientation.w = 0.34;

        */
        // For Left arm test
        EE_pose.header.frame_id = "left_torso_itb";
        EE_pose.header.stamp = ros::Time::now();
        // Set pose msg
        EE_pose.pose.position.x = 0.730350107464;
        EE_pose.pose.position.y = -0.558802375566;
        EE_pose.pose.position.z = -0.110572141382;

        EE_pose.pose.orientation.x = 0.0130226388095;
        EE_pose.pose.orientation.y = 0.713723126601;
        EE_pose.pose.orientation.z = -0.000821908620352;
        EE_pose.pose.orientation.w = 0.698057890754;

        Eigen::Translation3d translation(EE_pose.pose.position.x, EE_pose.pose.position.y, EE_pose.pose.position.z);
        Eigen::Quaterniond rotation(EE_pose.pose.orientation.w, EE_pose.pose.orientation.x, EE_pose.pose.orientation.y, EE_pose.pose.orientation.z);
        Eigen::Affine3d target_pose = translation * rotation;
        return target_pose;
    }

    std::vector<double> JointValueCalculation(Eigen::Affine3d* target_posePtr, int Arm_Selection)
    {
        std::vector<double> joint_values;
        //const std::vector<std::string> &joint_names = baxter_right_arm_group->getJointModelNames();

        std::vector<std::string> joint_names(JOINTS);

        //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
        // Get Current Joint Values
        switch(Arm_Selection)
        {
            case RIGHT_ARM:
                joint_names = baxter_right_arm_joint_names;
                kinematic_state->copyJointGroupPositions(baxter_right_arm_group, joint_values);
                break;
            case LEFT_ARM:
                joint_names = baxter_left_arm_joint_names;
                kinematic_state->copyJointGroupPositions(baxter_left_arm_group, joint_values);
                break;
            default:
                std::cout<<" No Arm Selected, return default joint values"<<std::endl;
                break;
        }
        /*
        // Just Print Out
        std::cout<<"Current set of joint values"<<std::endl;
        for(std::size_t i = 0; i < joint_names.size(); ++i)
        {
            std::cout<<"Joint"<<joint_names[i].c_str()<<":"<<joint_values[i]<<std::endl;
        }
        std::cout<<"==============================================="<<std::endl;
        */
        // Check If we have a target point
        if(target_posePtr==NULL)
        {
            ROS_INFO("No targetPos Assigned, return current joint values");
            return joint_values;
        }

        // Here we should have a functino to check the error distance between assgined goal and current joint values

        // Solve IK
        bool found_ik = kinematic_state->setFromIK(baxter_right_arm_group, *target_posePtr, 20, 0.1);
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(baxter_right_arm_group, joint_values);
            std::cout<<"Model frame:"<<kinematic_model->getModelFrame().c_str()<<std::endl;
            //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

            for(std::size_t i=0; i < joint_names.size(); ++i)
            {
                //std::cout<<"Joint"<<joint_names[i].c_str()<<":"<<joint_values[i]<<std::endl;
                //ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            return joint_values;
        }
        else
        {
            //ROS_INFO("Did not find IK solution, return current joint values");
            std::cout<<"Did not find IK solution, return current joint values"<<std::endl;
            return joint_values;
        }
    }

    control_msgs::FollowJointTrajectoryGoal CreatSendingGoal(int GoalNum,
                                                             std::vector<double> DesiredjointValues,
                                                             int Arm_Selection)
    {
        // Creat a return msg
        control_msgs::FollowJointTrajectoryGoal SendingGoal;

        switch(Arm_Selection)
        {
            case RIGHT_ARM:
            {
                SendingGoal.trajectory.joint_names.push_back("right_e0");
                SendingGoal.trajectory.joint_names.push_back("right_e1");
                SendingGoal.trajectory.joint_names.push_back("right_s0");
                SendingGoal.trajectory.joint_names.push_back("right_s1");
                SendingGoal.trajectory.joint_names.push_back("right_w0");
                SendingGoal.trajectory.joint_names.push_back("right_w1");
                SendingGoal.trajectory.joint_names.push_back("right_w2");

            }
            break;

        case LEFT_ARM:
            {
                SendingGoal.trajectory.joint_names.push_back("left_e0");
                SendingGoal.trajectory.joint_names.push_back("left_e1");
                SendingGoal.trajectory.joint_names.push_back("left_s0");
                SendingGoal.trajectory.joint_names.push_back("left_s1");
                SendingGoal.trajectory.joint_names.push_back("left_w0");
                SendingGoal.trajectory.joint_names.push_back("left_w1");
                SendingGoal.trajectory.joint_names.push_back("left_w2");
            }
            break;
        default:
            std::cout<<"No arm selected, return a default sending goal"<<std::endl;
            break;
        }
        // First, the joint names, which apply to all waypoints


        // Resize trajectory points based on specified goalnumber
        SendingGoal.trajectory.points.resize(GoalNum);
        // trajectory point
        // Positions
        int ind = 0;
        SendingGoal.trajectory.points[ind].positions.resize(7);
        SendingGoal.trajectory.points[ind].positions[0] = DesiredjointValues[0];
        SendingGoal.trajectory.points[ind].positions[1] = DesiredjointValues[1];
        SendingGoal.trajectory.points[ind].positions[2] = DesiredjointValues[2];
        SendingGoal.trajectory.points[ind].positions[3] = DesiredjointValues[3];
        SendingGoal.trajectory.points[ind].positions[4] = DesiredjointValues[4];
        SendingGoal.trajectory.points[ind].positions[5] = DesiredjointValues[5];
        SendingGoal.trajectory.points[ind].positions[6] = DesiredjointValues[6];

        // Velocities
        SendingGoal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            SendingGoal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        SendingGoal.trajectory.points[ind].time_from_start = ros::Duration(3.0);
        //we are done; return the goal
        return SendingGoal;
    }

    //! Sends the command to start a given trajectory
    void SendingTrajectory(control_msgs::FollowJointTrajectoryGoal goal, int Arm_Selection)
    {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(3.0);
        switch(Arm_Selection)
        {
        case RIGHT_ARM:
            Right_traj_client_->sendGoal(goal);
            break;

        case LEFT_ARM:
            Left_traj_client_->sendGoal(goal);
            break;

        default:
            break;
        }
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState(int Arm_Selection)
    {
        switch(Arm_Selection)
        {
        case RIGHT_ARM:
            return this->Right_traj_client_->getState();
            break;
        case LEFT_ARM:
            return this->Left_traj_client_->getState();
            break;
        default:
            std::cout<<"No Arm Selected, Returning the RightArmState!"<<std::endl;
            return this->Right_traj_client_->getState();
            break;
        }
    }

    //********************** For Gripper Command ***********************
    control_msgs::GripperCommandGoal CreateGripperCommand(GripperCommand cmd)
    {
        control_msgs::GripperCommandGoal msg;

        switch(cmd)
        {
        case CalibrateGripper:
            //msg.args = std::string("");
            break;

        case OpenGripper:
            msg.command.position = 100.0;
            msg.command.max_effort = 5;
            //msg.args = "position:0.0";
            break;

        case CloseGripper:
            msg.command.position = 0.0;
            msg.command.max_effort = 5;
            break;
        }

        return msg;
    }

    void SendingGripperGoal(control_msgs::GripperCommandGoal Gripper_goal)
    {
        Right_Gripper_client_->sendGoal(Gripper_goal);
    }

    actionlib::SimpleClientGoalState getGripperState(int GripperSelection)
    {
        switch (GripperSelection) {
        case Right_Gripper:
            return this->Right_Gripper_client_->getState();
            break;

        case Left_Gripper:
            return this->Left_Gripper_client_->getState();
            break;

        default:
            return this->Right_Gripper_client_->getState();
            break;
        }
    }

    //====================================================
    // HardCoded Joint Values;
    std::vector<struct HardCode_Jonit_Values> Video_Joint_Values;
    void CreatHardCodeJointValues()
    {
        HardCode_Jonit_Values Temp_Joint_Group_Values;

        // First Point
        Temp_Joint_Group_Values.e0 = -0.0299126;
        Temp_Joint_Group_Values.e1 = 1.66974;

        Temp_Joint_Group_Values.s0 = 0.699495;
        Temp_Joint_Group_Values.s1 = -0.101626;

        Temp_Joint_Group_Values.w0 = 0.0809175;
        Temp_Joint_Group_Values.w1 =-0.217442;
        Temp_Joint_Group_Values.w2 = 0.0233932;
        this->Video_Joint_Values.push_back(Temp_Joint_Group_Values);

        // Second Point
        Temp_Joint_Group_Values.e0 = -0.077466;
        Temp_Joint_Group_Values.e1 = 1.20878;

        Temp_Joint_Group_Values.s0 = 0.84484;
        Temp_Joint_Group_Values.s1 = -0.0456359;

        Temp_Joint_Group_Values.w0 = 0.0617427;
        Temp_Joint_Group_Values.w1 = -1.1386;
        Temp_Joint_Group_Values.w2 = -0.133456;
        this->Video_Joint_Values.push_back(Temp_Joint_Group_Values);

        // Third Point
        Temp_Joint_Group_Values.e0 = -0.0801505;
        Temp_Joint_Group_Values.e1 = 0.491257;

        Temp_Joint_Group_Values.s0 = 0.880505;
        Temp_Joint_Group_Values.s1 = 0.273816;

        Temp_Joint_Group_Values.w0 = 0.170272;
        Temp_Joint_Group_Values.w1 = -0.762388;
        Temp_Joint_Group_Values.w2 = -0.157617;
        this->Video_Joint_Values.push_back(Temp_Joint_Group_Values);

        // Forth Point
        Temp_Joint_Group_Values.e0 = -0.077466;
        Temp_Joint_Group_Values.e1 = 1.20878;

        Temp_Joint_Group_Values.s0 = 0.84484;
        Temp_Joint_Group_Values.s1 = -0.0456359;

        Temp_Joint_Group_Values.w0 = 0.0617427;
        Temp_Joint_Group_Values.w1 = -1.1386;
        Temp_Joint_Group_Values.w2 = -0.133456;
        this->Video_Joint_Values.push_back(Temp_Joint_Group_Values);

        // Fifth Point
        Temp_Joint_Group_Values.e0 = -0.168738;
        Temp_Joint_Group_Values.e1 = 0.612825;

        Temp_Joint_Group_Values.s0 = -0.22818;
        Temp_Joint_Group_Values.s1 = 0.415709;

        Temp_Joint_Group_Values.w0 = -2.58361;
        Temp_Joint_Group_Values.w1 = -0.548398;
        Temp_Joint_Group_Values.w2 = 0.0141893;
        this->Video_Joint_Values.push_back(Temp_Joint_Group_Values);

    }
    //====================================================
};

std::vector<double> GettingJointValuesFromContainer(HardCode_Jonit_Values Input)
{
    std::vector<double> HardCodedJointValue(JOINTS);

    HardCodedJointValue[0] = Input.e0;
    HardCodedJointValue[1] = Input.e1;
    HardCodedJointValue[2] = Input.s0;
    HardCodedJointValue[3] = Input.s1;
    HardCodedJointValue[4] = Input.w0;
    HardCodedJointValue[5] = Input.w1;
    HardCodedJointValue[6] = Input.w2;

    return HardCodedJointValue;
}

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "baxter_arm_traj_controller");
    // Make Sure that the Loader has get the model
    robot_model_loader::RobotModelLoader Robot_model_loader("robot_description");
    ros::NodeHandle Nhandle;
    // Create an object
    RobotArm arm(Nhandle, Robot_model_loader.getModel());
    //-----------------------------------------

    std::cout<<">>>>>>>>>>>>>>>>ROBOT Initilized>>>>>>>>>>>>>>>>"<<std::endl;

    // Define the variables to start a traj
    control_msgs::FollowJointTrajectoryGoal traj_goal;
    Eigen::Affine3d my_pose;

    std::vector<double> DesiredJointValue;

    // Create a hardcode joint vector
    arm.CreatHardCodeJointValues();
    std::vector<HardCode_Jonit_Values>::iterator it = arm.Video_Joint_Values.begin();

    bool StartANewTraj = true;

    bool StartGripperAction =true;

    bool GripperOpened = false;

    // Set up spin
    ros::Rate spin_rate(60.0);
    while(ros::ok() && (it!=arm.Video_Joint_Values.end()))
    {
        ros::spinOnce();

        if(StartANewTraj)
        {
            /*
            // Setting target pose
            my_pose = arm.SetTargetPos();
            // Calculating Desired Joint Values based on pose
            DesiredJointValue = arm.JointValueCalculation(&my_pose, LEFT_ARM);*/

            /*
            DesiredJointValue = GettingJointValuesFromContainer(*it);
            it++;
            // Creating a Goal Chain to be sent
            traj_goal = arm.CreatSendingGoal(1, DesiredJointValue, RIGHT_ARM);
            // Sending Goal to JointTrajectoryServer
            arm.SendingTrajectory(traj_goal,RIGHT_ARM);
            */
            StartANewTraj = false;
        }

        // Wait for trajectory completion
        if(arm.getState(RIGHT_ARM).isDone())
        {
            //std::cout<<"Desired Pose Reached"<<std::endl;
            StartANewTraj = true;
        }

        // Gripper Manipulation
        if(StartGripperAction)
        {
            if(!GripperOpened)
            {
                arm.gripper_goal = arm.CreateGripperCommand(OpenGripper);
                arm.SendingGripperGoal(arm.gripper_goal);
                GripperOpened = true;
            }
            else
            {
                arm.gripper_goal = arm.CreateGripperCommand(CloseGripper);
                arm.SendingGripperGoal(arm.gripper_goal);
                GripperOpened = false;
            }

            StartGripperAction = false;
        }

        // Wait for gripper action finish
        if(arm.getGripperState(Right_Gripper).isDone())
        {
            ros::Duration(3.0);
            StartGripperAction = true;
        }

        spin_rate.sleep();
    }

    std::cout<<"================Node Ends=================="<<std::endl;

}

