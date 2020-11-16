#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/Pose.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <actionlib/client/simple_action_client.h>


#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>    //TF2 to convert YPR to Quaternion

#include <MyFunc.h>

//Prova commit


#define PLANNING_ATTEMPTS_NO 500
#define INPUT_RPY 1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test2");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher cmd_pub = nh.advertise<std_msgs::Float32>("/gripper_command",100);
    ros::Publisher width_pub = nh.advertise<std_msgs::Float32>("/My_new_input", 100);

    std_msgs::Float32 width_msg;
    std_msgs::Float32 cmd_msg;

    namespace rvt = rviz_visual_tools;

    float T_O;
    if (not (nh.getParam("/TrajectoryTest/Offset", T_O)))
        T_O = 0.4;
    ros::Duration T_offset = ros::Duration(0.4);
    ros::Duration T_round = ros::Duration(0.4);
   

    float velocity_fix = 0.02;
    float velocity_round = 0.1;
    float velocity_pass = 0.05;
   

    float mean_velocity;



    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    std::string path1 = "/home/kevin/ros/ur5_ws/src/test/src/PointList/Test";

    ROS_INFO("Reach Ready Position");
    move_group.setMaxVelocityScalingFactor(0.2);

    
/***************  SYSTEM PARAMETERS **************************/
    
    TrajectoryVector waypoints;
    TrajectoryPlanner_param param;

    param.radius = 0.05;                // Radius of the semi-circle for fixing part
    param.heigh = 0.03;                 // Heigh of upward movement
    param.circ_point = 1000;             // Point generated in the semi-circle
    param.res = 0.001;                  // Distance beetween two following points in the final trajectory
    param.distance_approach = 0.05;     // Distance from which starts the grasping operation
    param.Ctrl_pt_d1 = 0.05;            // Distance of the first control point in the rounding function
    param.Ctrl_pt_d2 = 0.05;            // Distance of the second point
    param.corner_points = 100;          // Numver of point generated in the corner.

/**************** COMPUTE TRAJECTORY PARTS      ******************/


    geometry_msgs::Pose grasp1;
    geometry_msgs::PoseArray waypoints_final;

    ReadFileTxt(INPUT_RPY, waypoints.point, waypoints.pt_label, path1, grasp1);
    MyTrajectoryPlanner3(param, waypoints);
    visual_tools.deleteAllMarkers();
    


    for(int i=0; i<waypoints.SecondaryTrajectory.size(); i++)
    {
        visual_tools.publishPath(waypoints.SecondaryTrajectory[i].poses, rvt::BLUE, rvt::XXXSMALL);
        FromEE2Link8(waypoints.SecondaryTrajectory[i]);
    //    EE_Shift(waypoints.SecondaryTrajectory[i]);
    }

/**** Print result ****/


    visual_tools.publishPath(waypoints.point.poses, rvt::LIME_GREEN, rvt::XXXSMALL);

    for(int i=0; i<waypoints.point.poses.size(); i++)
    {
        visual_tools.publishAxisLabeled(waypoints.point.poses[i], "init_point" , rvt::XXXSMALL);
    }

    for(int i=0; i<waypoints.SecondaryTrajectory.size(); i++)
    {
        for(int k=0; k<waypoints.SecondaryTrajectory[i].poses.size(); k++)
        {
            visual_tools.publishAxisLabeled(waypoints.SecondaryTrajectory[i].poses[k], waypoints.pt_label[i] , rvt::XXXSMALL);
        }
    }
    visual_tools.trigger();
    visual_tools.prompt("Next: to continue");


/****************** MOTION INIT *****************************************/
        

    moveit_msgs::RobotTrajectory trajectory_cartesian;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction;   

    moveit::planning_interface::MoveGroupInterface::Plan Plan_Cartesian, Plan2;

    bool success;
    int size_plan, final_plan_size;

    move_group.setNumPlanningAttempts(PLANNING_ATTEMPTS_NO); 
    move_group.setPlanningTime(10);
    move_group.setMaxVelocityScalingFactor(0.3);

/*********************************************************************************************************/
    // Movimento alla posizione di partenza

    moveit::planning_interface::MoveGroupInterface::Plan Plan_SetPoseTarget;
    move_group.setPoseTarget(waypoints.SecondaryTrajectory[0].poses[0]);
    move_group.plan(Plan_SetPoseTarget);
    move_group.execute(Plan_SetPoseTarget);

/********* PLAN OF THE TRAJECTORIES ****************/

    
    std::vector<std::string> LabelVector;
    std::vector<moveit_msgs::RobotTrajectory> TrajVector;
    moveit_msgs::RobotTrajectory Traj_flag;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> PlansVector;      // potrebbe non essere usato


    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    robot_state::RobotState start_state(*move_group.getCurrentState());

    std::vector<double[7] > joints_list_Cartesian;
    double joints_Cartesian[7];

    geometry_msgs::PoseArray PoseFlag;
    std::string last_label;
    last_label = " ";

    std::cout << "Second Trajectory size: " << waypoints.SecondaryTrajectory.size() << "\n";
    std::cout << "waypoint label size : " << waypoints.pt_label.size() << "\n";

    std::cout << waypoints.SecondaryTrajectory[0].poses.size() << "\n";
    std::cout << "start \n";
   
    for(int k=0; k < waypoints.SecondaryTrajectory[0].poses.size(); k++)
    {
        PoseFlag.poses.push_back(waypoints.SecondaryTrajectory[0].poses[k]);
    }
    last_label = waypoints.pt_label[0];

    std::cout << "start_for \n";
    for(int i=1; i < waypoints.SecondaryTrajectory.size(); i++)
    {
        std::cout << "Point: "<< waypoints.pt_label[i] << "\n";

        if(waypoints.pt_label[i] == last_label)
        {
            for(int k=0; k<waypoints.SecondaryTrajectory[i].poses.size(); k++)
            {
                PoseFlag.poses.push_back(waypoints.SecondaryTrajectory[i].poses[k]);
            }
        }
        else
        {
            // Nuovo nome da confrontare
            // Scegliere la velocità per il planner
            if(waypoints.pt_label[i-1] == "pass")
            {
                mean_velocity = velocity_pass;
            }
            if(waypoints.pt_label[i-1] == "round")
            {
                mean_velocity = velocity_round;
            }
            if(waypoints.pt_label[i-1] == "fix" || waypoints.pt_label[i-1] == "cornerfix")
            {
                mean_velocity = velocity_fix;
            }
            else
            {
                mean_velocity = velocity_fix;
            }
            
            LabelVector.push_back(waypoints.pt_label[i-1]);
            move_group.computeCartesianPath(PoseFlag.poses, eef_step, jump_threshold, trajectory_cartesian);
            if (trajectory_cartesian.joint_trajectory.points.size() == 0)
            {
                ROS_INFO("Skip");
            }
            else
            {   
                Traj_flag = VelocityScaling(trajectory_cartesian, PoseFlag,  mean_velocity, T_offset);  
                TrajVector.push_back(Traj_flag);             
                size_plan = trajectory_cartesian.joint_trajectory.points.size();
                for (int j=0; j<7; j++)
                {
                    joints_Cartesian[j] = trajectory_cartesian.joint_trajectory.points[size_plan-1].positions[j];
                }

                start_state.setJointGroupPositions(joint_model_group, joints_Cartesian);       
                move_group.setStartState(start_state);
                PoseFlag.poses.clear();
            }


            for(int k=0; k<waypoints.SecondaryTrajectory[i].poses.size(); k++)
            {
                PoseFlag.poses.push_back(waypoints.SecondaryTrajectory[i].poses[k]);
            }
            last_label = waypoints.pt_label[i];
        }
        ROS_INFO("Point completed");
    }

    LabelVector.push_back(waypoints.pt_label[waypoints.pt_label.size()-1]);
    move_group.computeCartesianPath(PoseFlag.poses, eef_step, jump_threshold, trajectory_cartesian);

    if (trajectory_cartesian.joint_trajectory.points.size() > 0)
    {
        TrajVector.push_back(VelocityScaling(trajectory_cartesian, PoseFlag,  mean_velocity, T_offset));
    }
    else
    {
        ROS_INFO("SKIP");
    }

    PlansVector.resize(TrajVector.size());
    for(int i=0; i<TrajVector.size();i++)
        PlansVector[i].trajectory_ = TrajVector[i];



    for(int i=0; i<PlansVector[0].trajectory_.joint_trajectory.joint_names.size(); i++ )
        std::cout << PlansVector[0].trajectory_.joint_trajectory.joint_names[PlansVector[0].trajectory_.joint_trajectory.joint_names.size()-1] << "\n";
    
    std::cout << PlansVector.size() << "";

    for(int i=0; i<PlansVector.size(); i++)
    {
        move_group.execute(PlansVector[i]);
    }

    
    ros::shutdown();
    return 0;
}

