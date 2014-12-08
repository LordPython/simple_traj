// Fellow Joint Trajectory Action
#include <ros/ros.h>

// Controller Communication
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

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

#define _USE_MATH_DEFINES
#define JOINTS 7

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
    // ROS HANDLE
    ros::NodeHandle nh_;
    // Subscriber to joint states
    ros::Subscriber joint_state_sub_;
    ros::Subscriber goal_msg_sub_;
    ros::Publisher goal_state_topic_;

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

    // Robot Kinecmatic state
    robot_state::RobotStatePtr kinematic_state;
    // Set up an internal robot model
    const robot_model::RobotModelConstPtr kinematic_model;
    // Endeffector pose test
    geometry_msgs::PoseStamped EE_pose;
    geometry_msgs::PoseStamped goal_pose;
    Eigen::Affine3d desired_pose;

public:
    Eigen::Affine3d subscribed_pose_;
    bool StartANewTraj;
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal_ToBeSend;
    //! Initialize the action client and wait for action server to come up
    RobotArm(ros::NodeHandle& nh, const robot_model::RobotModelPtr& model) : nh_(nh), kinematic_model(model)
    {
        // Initialize
        this->StartANewTraj = false;

        // tell the action client that we want to spin a thread by default
        Right_traj_client_ = new TrajClient("/robot/limb/right/follow_joint_trajectory", true);
        Left_traj_client_ = new TrajClient("/robot/limb/left/follow_joint_trajectory", true);
        // wait for action server to come up
        while(!Right_traj_client_->waitForServer(ros::Duration(2.0))||
              !Left_traj_client_->waitForServer(ros::Duration(2.0)))
        {
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
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

        // Advertise a new topic to recieved new target pose
        goal_state_topic_ = nh_.advertise<geometry_msgs::PoseStamped>("/simple_traj/target", 1, true);

        // Register a subscriber and callback func to joint_state topic
        joint_state_sub_ = nh_.subscribe("/robot/joint_states", 1, &RobotArm::RobotStateCB,this);

        // Register a subscriber to get new target_pose and callback func to set the target_pose
        goal_msg_sub_ = nh_.subscribe("/simple_traj/target", 1, &RobotArm::GoalMsgCB,this);

    }

    //! Clean up the action client
    ~RobotArm()
    {
        delete Right_traj_client_;
        delete Left_traj_client_;
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

    bool IF_GOAL_VALUABLE()
    {
        return true;
    }

    void GoalMsgCB(geometry_msgs::PoseStamped target_pose)
    {
        // set the target pose msg for baxter
        // Stamped Pose
        target_pose.header.frame_id = "base"; // Should this be changed to BASE
        target_pose.header.stamp = ros::Time::now();
        // Convert to Eigen
        // Set target pose from subscribed message
        Eigen::Translation3d translation(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        Eigen::Quaterniond rotation(target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z);
        Eigen::Affine3d new_target_pose = translation * rotation;
        // return the pose
        subscribed_pose_ = new_target_pose;
        this->StartANewTraj = true;

    }

    void RobotStateCB(sensor_msgs::JointState msg)
      {
        std::map<std::string, double> joint_positions;
        for (size_t idx = 0; idx < msg.name.size(); idx++)
        {
            joint_positions[msg.name[idx]] = msg.position[idx];
        }
        /*// This is just for test
        for(std::map<std::string, double>::const_iterator it = joint_positions.begin(); it != joint_positions.end(); it++)
        {
            std::cout<<" Joint Name "<<it->first<<":"<< " value:"<< it->second << std::endl;
        }
        //ROS_INFO("************************************");*/

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
        kinematic_state->update(true);
      }

    /* POSE ASSIGNMENT */
    Eigen::Affine3d SetTargetPos()
    {
        // Stamped Pose
        EE_pose.header.frame_id = "right_torso_itb"; // Should this be changed to BASE
        EE_pose.header.stamp = ros::Time::now();
        /* Set pose msg */
        EE_pose.pose.position.x = 0.84;
        EE_pose.pose.position.y = -0.43;
        EE_pose.pose.position.z = -0.11;
        EE_pose.pose.orientation.x = 0.07;
        EE_pose.pose.orientation.y = 0.93;
        EE_pose.pose.orientation.z = -0.01;
        EE_pose.pose.orientation.w = 0.34;
        Eigen::Translation3d translation(EE_pose.pose.position.x, EE_pose.pose.position.y, EE_pose.pose.position.z);
        Eigen::Quaterniond rotation(EE_pose.pose.orientation.w, EE_pose.pose.orientation.x, EE_pose.pose.orientation.y, EE_pose.pose.orientation.z);
        Eigen::Affine3d target_pose = translation * rotation;
        return target_pose;
    }

    std::vector<double> JointValueCalculation(Eigen::Affine3d* target_posePtr)
    {
        std::vector<double> joint_values;

        //const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
        //const robot_model::JointModelGroup joint_model_group = baxter_right_arm_group;

        const std::vector<std::string> &joint_names = baxter_right_arm_group->getJointModelNames();

        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
        // Get Current Joint Values
        ROS_INFO("Current set of joint values");
        kinematic_state->copyJointGroupPositions(baxter_right_arm_group, joint_values);
        for(std::size_t i = 0; i < joint_names.size(); ++i)
        {
            std::cout<<"Joint"<<joint_names[i].c_str()<<":"<<joint_values[i]<<std::endl;
            //ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
        std::cout<<"==============================================="<<std::endl;

        // Get current state
        //kinematic_state->setJointGroupPositions(baxter_right_arm_group, joint_values);

        if(target_posePtr==NULL)
        {
            ROS_INFO("No targetPos Assigned, return current joint values");
            return joint_values;
        }

        // Solve IK
        bool found_ik = kinematic_state->setFromIK(baxter_right_arm_group, *target_posePtr, 20, 0.1);
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(baxter_right_arm_group, joint_values);
            ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
            for(std::size_t i=0; i < joint_names.size(); ++i)
            {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }

            return joint_values;
        }
        else
        {
            ROS_INFO("Did not find IK solution, return current joint values");
            return joint_values;
        }
    }

    control_msgs::FollowJointTrajectoryGoal CreatSendingGoal(int GoalNum, std::vector<double> DesiredjointValues)
    {
        control_msgs::FollowJointTrajectoryGoal SendingGoal;

        // First, the joint names, which apply to all waypoints
        SendingGoal.trajectory.joint_names.push_back("right_s0");
        SendingGoal.trajectory.joint_names.push_back("right_s1");
        SendingGoal.trajectory.joint_names.push_back("right_e0");
        SendingGoal.trajectory.joint_names.push_back("right_e1");
        SendingGoal.trajectory.joint_names.push_back("right_w0");
        SendingGoal.trajectory.joint_names.push_back("right_w1");
        SendingGoal.trajectory.joint_names.push_back("right_w2");

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
        SendingGoal.trajectory.points[ind].time_from_start = ros::Duration(5.0);
        //we are done; return the goal
        return SendingGoal;
    }

    //! Sends the command to start a given trajectory
    void SendingTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
        Right_traj_client_->sendGoal(goal);
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
        return Right_traj_client_->getState();
    }
};

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "baxter_arm_traj_controller");
    // Make Sure that the Loader has get the model
    robot_model_loader::RobotModelLoader Robot_model_loader("robot_description");
    //ros::Duration ModelLoadingTime = ros::Duration(0.5, 0);
    //ModelLoadingTime.sleep();
    ros::NodeHandle Nhandle;
    RobotArm arm(Nhandle, Robot_model_loader.getModel());
    std::cout<<">>>>>>>>>>>>>>>>ROBOT Initilized>>>>>>>>>>>>>>>>"<<std::endl;

    // Start the trajectory
    control_msgs::FollowJointTrajectoryGoal traj_goal;
    Eigen::Affine3d my_pose;
    std::vector<double> DesiredJointValue;


    // Setting target pose
    //my_pose = arm.SetTargetPos();



    // Set up spin
    ros::Rate spin_rate(60.0);

    while(ros::ok())
    {
        ros::spinOnce();
        // Setting target pose to one that is recieved in subscribed topic
        my_pose = arm.subscribed_pose_;



        if(arm.StartANewTraj)
        {
            // Calculating Desired Joint Values based on pose
            DesiredJointValue = arm.JointValueCalculation(&my_pose);
            // Creating a Goal Chain to be sent
            traj_goal = arm.CreatSendingGoal(1,DesiredJointValue);
            // Sending Goal to JointTrajectoryServer
            arm.SendingTrajectory(traj_goal);
            arm.StartANewTraj = false;
        }

        /*
        if(StartANewTraj)
        {
            // Calculating Desired Joint Values based on pose
            DesiredJointValue = arm.JointValueCalculation(&my_pose);
            // Creating a Goal Chain to be sent
            traj_goal = arm.CreatSendingGoal(1,DesiredJointValue);
            // Sending Goal to JointTrajectoryServer
            arm.SendingTrajectory(traj_goal);

            StartANewTraj = false;
        }
        */

        if(arm.getState().isDone())
        {
            std::cout<<"Desired Pose Reached. Waiting for new target pose input"<<std::endl;
            //StartANewTraj = true;
        }

        spin_rate.sleep();
    }

    ROS_INFO("Done");
}

