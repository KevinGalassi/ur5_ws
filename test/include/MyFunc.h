#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Accel.h"

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_tutorials/planning_interface/move_group_interface/Plan.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>    //TF2 to convert YPR to Quaternion
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include "GP_Functions.h"

namespace rvt = rviz_visual_tools;

// Robot known positions
const std::vector<double> arm_ready_state       = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
const std::vector<double> hand_ready_state      = {0.04, 0.04};
const std::vector<double> hand_open_position    = {0.037, 0.037};
const std::vector<double> hand_closed_position  = {0.0, 0.0};
const std::vector<double> hand_grasp_position  = {0.004, 0.004};

// Denavit-Hartenberg Parameters of Panda
const float DH_param_a[8]       = {0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0 };
const float DH_param_d[8]       = {0.333, 0, 0.316, 0, 0.384, 0,  0, 0.107};
const float DH_param_alpha[8]   = {0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0};

// Panda limits
const double q_ul[7]        = {0, 0, 0, 0, 0, 0, 0};
const double q_ll[7]        = {0, 0, 0, 0, 0, 0, 0};
const double qd_max[7]	    = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
const double qdd_max[7]	    = {15, 7.5, 10, 12.5, 15, 20, 20};
const double qddd_max[7]    = {7500, 3750, 5000, 6250, 7500, 10000, 10000};

const double gripper_offset[3] = {0, 0, 0.1};

struct TrajectoryVector 
{
    geometry_msgs::PoseArray point;
    std::vector<std::string>  pt_label;
    std::vector<geometry_msgs::PoseArray> SecondaryTrajectory;
    std::vector<std::string> info;
};

struct RescaleVelocityTrajectory
{
    geometry_msgs::PoseArray ws_Pose;
    std::vector<geometry_msgs::Twist> ws_Twist;
    std::vector<geometry_msgs::Accel> ws_Accel;
    std::vector<trajectory_msgs::JointTrajectoryPoint> jointspace_points;
};

struct Jacobian_Computation
{
    double A[4][4];
    double T[4][4];
};

struct Jacobian_Computation_eigen
{
    Eigen::Matrix<double,4,4> A;
    Eigen::Matrix<double,4,4> T;
};

struct TrajectoryPlanner_param
{
    double radius;
    double heigh;
    int circ_point;
    double res;
    double distance_approach;
    double Ctrl_pt_d1;
    double Ctrl_pt_d2;
    int corner_points; 
};

/*
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
*/


/*
    pass
    corner
    fix
    patch
*/

void MyTrajectoryPlanner(TrajectoryPlanner_param param, TrajectoryVector& waypoints);

void FromEE2Link8(geometry_msgs::PoseArray& waypoints);



void FromLink82EE(geometry_msgs::PoseArray& waypoints);

void addFixPoint(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, double radius, double heigh, int res);
void addFixPoint2(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, double radius, double heigh, int res, double dis_init, double dist_final, bool orientation);
void addFixPoint3(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, double radius, double heigh, int res, double dis_init, double dist_final, bool orientation);

void CornerFix2(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, geometry_msgs::Pose corner_point, geometry_msgs::Pose starting_point,
                             TrajectoryPlanner_param param, double dis_init, double dist_final, bool orientation);


void CornerDetection(geometry_msgs::PoseArray waypoints, std::vector<std::string>& id_list);
void CornerDetection_test(std::vector<geometry_msgs::PoseArray> waypoints, std::vector<std::string>& id_list);

void CornerRounding(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2, geometry_msgs::Pose Point3, double distance1, double distance2, geometry_msgs::PoseArray& waypoints, double res);

void GraspWire(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface& hand_group, geometry_msgs::Pose grasp_pose, double d, double res, geometry_msgs::PoseArray& waypoints);

int ComputePatch(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2, geometry_msgs::PoseArray& waypoints, double res);

void ComputeTmatrix_float (Eigen::Matrix<float, 4,4>& A, float DH_param_a, float DH_param_alpha, float DH_param_d, float Joint_value);

void getJacobianInverse(Eigen::Matrix<double,6,7>  Jacobian, Eigen::Matrix<double, 7, 6>& Jacobian_inverse);

void getJacobianDerivate (Eigen::Matrix<double,6,7> Jacobian, Eigen::Matrix<double,6,7>& Jacobian_der, std::vector<double> vel_in);

void computeEE_Vel_Acc(geometry_msgs::PoseArray waypoints, std::vector<geometry_msgs::Twist>& EE_velocity, std::vector<geometry_msgs::Accel>& EE_accel,  float velocity, double res);

void getJointSpaceVelocity(std::vector<trajectory_msgs::JointTrajectoryPoint>& joints_configuration, std::vector<geometry_msgs::Twist> EE_velocity);

void setJointSpaceVel_Acc(std::vector<trajectory_msgs::JointTrajectoryPoint>& joints_configuration, std::vector<geometry_msgs::Twist> EE_velocity, std::vector<geometry_msgs::Accel> EE_acceleration);



moveit_msgs::RobotTrajectory FactorScaling(moveit_msgs::RobotTrajectory trajectory_cartesian, float scaling_factor, ros::Duration T_offset);

float ComputeDistance(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2);

float ComputePathLength(geometry_msgs::PoseArray waypoint);

// Definizione funzioni

void CornerFix(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, geometry_msgs::Pose corner_point, geometry_msgs::Pose starting_point, TrajectoryPlanner_param param, double dis_init, double dist_final, bool orientation);

bool ReadFileTxt(bool input_rpy, geometry_msgs::PoseArray& point, std::vector<std::string>& label, std::string file_path, geometry_msgs::Pose& grasp_pose)
{
    std::ifstream pose_list;
    geometry_msgs::Pose target_goal;
    std::string pose_identifie;

    pose_list.open(file_path);
    if( pose_list.is_open())
    {
        ROS_INFO("File succesfully open");
    }
    else
    {
        ROS_INFO("file not open");
        return false;
    }

    if(input_rpy)
    {
        float input_RPY[3] = {0, 0, 0};
        tf2::Quaternion input_quaternion;
        while ( !pose_list.eof() )
        { 
            std::cout << "data readen \n";
            pose_list >> target_goal.position.x;
            pose_list >> target_goal.position.y;
            pose_list >> target_goal.position.z;
            pose_list >> input_RPY[0];
            pose_list >> input_RPY[1];
            pose_list >> input_RPY[2];
            input_quaternion.setRPY(input_RPY[0], input_RPY[1], input_RPY[2]);
            target_goal.orientation = tf2::toMsg(input_quaternion);
            pose_list >> pose_identifie;

            if (pose_identifie == "grasp")
            {
                grasp_pose = target_goal;
            }
            else
            {
                point.poses.push_back(target_goal);
                label.push_back(pose_identifie);
            }

        }
    }
    else
    {
    while ( !pose_list.eof() )
        { 
            std::cout << "data readen \n";
            pose_list >> target_goal.position.x;
            pose_list >> target_goal.position.y;
            pose_list >> target_goal.position.z;
            pose_list >> target_goal.orientation.w;
            pose_list >> target_goal.orientation.x;
            pose_list >> target_goal.orientation.y;
            pose_list >> target_goal.orientation.z;
            pose_list >> pose_identifie;

            if (pose_identifie == "grasp")
            {
                grasp_pose = target_goal;
            }
            else
            {
                point.poses.push_back(target_goal);
                label.push_back(pose_identifie);
            }
        }
    }
    pose_list.close();
    return true;
}

bool ReadFileTxt_M(bool input_rpy, std::vector<geometry_msgs::PoseArray>& point, std::vector<std::vector<std::string>>& label, std::string file_path, std::vector<geometry_msgs::Pose>& grasp_pose)
{
    std::ifstream pose_list;
    geometry_msgs::Pose target_goal;
    std::string pose_identifie;

    geometry_msgs::PoseArray wire_point;
    std::vector<std::string> label_list;
    int i = 0;

    pose_list.open(file_path);
    if( pose_list.is_open())
    {
        ROS_INFO("File succesfully open");
    }
    else
    {
        ROS_INFO("file not open");
        return false;
    }

    if(input_rpy)
    {
        float input_RPY[3] = {0, 0, 0};
        tf2::Quaternion input_quaternion;

        while ( !pose_list.eof() )
        { 
            std::cout << "data readen \n";
            pose_list >> target_goal.position.x;
            pose_list >> target_goal.position.y;
            pose_list >> target_goal.position.z;
            pose_list >> input_RPY[0];
            pose_list >> input_RPY[1];
            pose_list >> input_RPY[2];
            input_quaternion.setRPY(input_RPY[0], input_RPY[1], input_RPY[2]);
            target_goal.orientation = tf2::toMsg(input_quaternion);
            pose_list >> pose_identifie;

            if (pose_identifie == "grasp")
            {   
                std::cout << "wire acquired \n";
                point.push_back(wire_point);
                label.push_back(label_list);
                label_list.clear();
                wire_point.poses.clear();
                grasp_pose.push_back(target_goal);
                i++;
            }
            else
            {
                std::cout << "data saved \n";
                wire_point.poses.push_back(target_goal);
                label_list.push_back(pose_identifie);
            }
        }

        std::cout << "End of file \n";
    }
    else
    {
    while ( !pose_list.eof() )
        { 
            std::cout << "data readen \n";
            pose_list >> target_goal.position.x;
            pose_list >> target_goal.position.y;
            pose_list >> target_goal.position.z;
            pose_list >> target_goal.orientation.w;
            pose_list >> target_goal.orientation.x;
            pose_list >> target_goal.orientation.y;
            pose_list >> target_goal.orientation.z;
            pose_list >> pose_identifie;

            if (pose_identifie == "grasp")
            {
                point.push_back(wire_point);
                label.push_back(label_list);
                label_list.clear();
                wire_point.poses.clear();
                grasp_pose.push_back(target_goal);
                i++;
            }
            else
            {
                wire_point.poses.push_back(target_goal);
                label_list.push_back(pose_identifie);
            }
        }
    }
    pose_list.close();
    return true;
}

void MyTrajectoryPlanner(TrajectoryPlanner_param param, TrajectoryVector& waypoints)
{    
    std::vector<geometry_msgs::PoseArray> Trajectories_list, Trajectory_list_with_patch;        // Store the trajectory of each point of the file

    geometry_msgs::PoseArray Trajectory_part;    
    std::vector<std::string> Trajectory_id;            
    geometry_msgs::PoseArray Patch_To_Add;

    float PP_distance;          
    double new_Ctrl_Pt_d1;
    double new_Ctrl_Pt_d2;

    geometry_msgs::Pose P1,P2,P3;
    int trajectory_size;
    
    for (std::size_t i = 0; i < waypoints.point.poses.size(); i ++)
    {  
        if (waypoints.pt_label[i] == "fix")
        {
            addFixPoint(waypoints.point.poses[i], Trajectory_part, param.radius, param.heigh, param.circ_point);
            Trajectories_list.push_back(Trajectory_part);
            Trajectory_part.poses.clear();
            Trajectory_id.push_back("fix");
        }

        if (waypoints.pt_label[i] == "pass")
        {
            Trajectory_part.poses.push_back(waypoints.point.poses[i]);
            Trajectories_list.push_back(Trajectory_part);
            Trajectory_part.poses.clear();   
            Trajectory_id.push_back("pass");
        }
    } 

    CornerDetection_test(Trajectories_list, Trajectory_id);

    for (size_t i = 1; i < (Trajectories_list.size()-1); i++)
    {
        if(Trajectory_id[i] == "corner")
        {
            trajectory_size = Trajectories_list[i-1].poses.size();
            P1 = Trajectories_list[i-1].poses[trajectory_size-1];
            P2 = Trajectories_list[i].poses[0];
            P3 = Trajectories_list[i+1].poses[0];

            PP_distance = ComputeDistance(P1, P2);
            if( Trajectory_id[i-1] != "corner")
            {
                if(PP_distance < param.Ctrl_pt_d1)
                    new_Ctrl_Pt_d1 = PP_distance;
                else
                    new_Ctrl_Pt_d1 =  param.Ctrl_pt_d1;
            }
            else
            {
                new_Ctrl_Pt_d1 = PP_distance/2;
            }

            PP_distance = ComputeDistance(P2, P3);
            if(waypoints.pt_label[i+1] != "corner")
            {
                if (PP_distance < param.Ctrl_pt_d2)
                    new_Ctrl_Pt_d2 = PP_distance;
                else 
                    new_Ctrl_Pt_d2 =  param.Ctrl_pt_d2;
            }
            else
            {
                new_Ctrl_Pt_d2 = PP_distance/2;
            }
            CornerRounding(P1, P2, P3, new_Ctrl_Pt_d1, new_Ctrl_Pt_d2, Trajectory_part, param.corner_points);
            Trajectories_list[i].poses.clear();
            Trajectories_list[i].poses = Trajectory_part.poses;
            Trajectory_id[i] = "corner";
            Trajectory_part.poses.clear();
        }
    }

    std::vector<std::string> LabelVector_flag;

/**************** INCLUDE PATCH ******************/  

    waypoints.SecondaryTrajectory.push_back(Trajectories_list[0]);
    LabelVector_flag.push_back(Trajectory_id[0]);

    for(size_t i=1; i<Trajectories_list.size(); i++)
    {
        if (ComputePatch(Trajectories_list[i-1].poses.back(), Trajectories_list[i].poses[0], Patch_To_Add, param.res) >0)
        {
            waypoints.SecondaryTrajectory.push_back(Patch_To_Add);
            LabelVector_flag.push_back("pass");
        }    
        waypoints.SecondaryTrajectory.push_back(Trajectories_list[i]);   
        LabelVector_flag.push_back(Trajectory_id[i]);     
        Patch_To_Add.poses.clear();
    }

    waypoints.pt_label.clear();
    for(int i=0; i< LabelVector_flag.size(); i++)
    {
        waypoints.pt_label.push_back(LabelVector_flag[i]);
    }
}

void MyTrajectoryPlanner2(TrajectoryPlanner_param param, TrajectoryVector& waypoints)
{    
    std::vector<geometry_msgs::PoseArray> Trajectories_list, Trajectory_list_with_patch;        // Store the trajectory of each point of the file

    geometry_msgs::PoseArray Trajectory_part;    
    std::vector<std::string> Trajectory_id;            
    geometry_msgs::PoseArray Patch_To_Add;

    float PP_distance;          
    double new_Ctrl_Pt_d1;
    double new_Ctrl_Pt_d2;

    geometry_msgs::Pose P1,P2,P3;
    int trajectory_size;
    
    for (std::size_t i = 0; i < waypoints.point.poses.size(); i ++)
    {  
        if (waypoints.pt_label[i] == "fix")
        {
            addFixPoint2(waypoints.point.poses[i], Trajectory_part, param.radius, param.heigh, param.circ_point, 0.05, 0.04, false);
            Trajectories_list.push_back(Trajectory_part);
            Trajectory_part.poses.clear();
            Trajectory_id.push_back("fix");
        }

        if (waypoints.pt_label[i] == "pass")
        {
            Trajectory_part.poses.push_back(waypoints.point.poses[i]);
            Trajectories_list.push_back(Trajectory_part);
            Trajectory_part.poses.clear();   
            Trajectory_id.push_back("pass");
        }
    } 

    CornerDetection_test(Trajectories_list, Trajectory_id);

    for (size_t i = 1; i < (Trajectories_list.size()-1); i++)
    {
        if(Trajectory_id[i] == "corner")
        {
            trajectory_size = Trajectories_list[i-1].poses.size();
            P1 = Trajectories_list[i-1].poses[trajectory_size-1];
            P2 = Trajectories_list[i].poses[0];
            P3 = Trajectories_list[i+1].poses[0];

            PP_distance = ComputeDistance(P1, P2);
            if( Trajectory_id[i-1] != "corner")
            {
                if(PP_distance < param.Ctrl_pt_d1)
                    new_Ctrl_Pt_d1 = PP_distance;
                else
                    new_Ctrl_Pt_d1 =  param.Ctrl_pt_d1;
            }
            else
            {
                new_Ctrl_Pt_d1 = PP_distance/2;
            }

            PP_distance = ComputeDistance(P2, P3);
            if(waypoints.pt_label[i+1] != "corner")
            {
                if (PP_distance < param.Ctrl_pt_d2)
                    new_Ctrl_Pt_d2 = PP_distance;
                else 
                    new_Ctrl_Pt_d2 =  param.Ctrl_pt_d2;
            }
            else
            {
                new_Ctrl_Pt_d2 = PP_distance/2;
            }
            CornerRounding(P1, P2, P3, new_Ctrl_Pt_d1, new_Ctrl_Pt_d2, Trajectory_part, param.corner_points);
            Trajectories_list[i].poses.clear();
            Trajectories_list[i].poses = Trajectory_part.poses;
            Trajectory_id[i] = "corner";
            Trajectory_part.poses.clear();
        }
    }

    std::vector<std::string> LabelVector_flag;

/**************** INCLUDE PATCH ******************/  

    waypoints.SecondaryTrajectory.push_back(Trajectories_list[0]);
    LabelVector_flag.push_back(Trajectory_id[0]);

    for(size_t i=1; i<Trajectories_list.size(); i++)
    {
        if (ComputePatch(Trajectories_list[i-1].poses.back(), Trajectories_list[i].poses[0], Patch_To_Add, param.res) >0)
        {
            waypoints.SecondaryTrajectory.push_back(Patch_To_Add);
            LabelVector_flag.push_back("pass");
        }    
        waypoints.SecondaryTrajectory.push_back(Trajectories_list[i]);   
        LabelVector_flag.push_back(Trajectory_id[i]);     
        Patch_To_Add.poses.clear();
    }

    waypoints.pt_label.clear();
    for(int i=0; i< LabelVector_flag.size(); i++)
    {
        waypoints.pt_label.push_back(LabelVector_flag[i]);
    }
}


void CornerDetection_3(geometry_msgs::PoseArray point, std::vector<std::string>& label)
{
    float det1, det2;
    geometry_msgs::Pose P1, P3;
    int Traj1_size, Traj3_size;

    float AB, AC, BC;

    for(int i = 1; i < (point.poses.size()-1); i++)
    {
        if(label[i] == "pass" && label[i-1] != "grasp")
        {
            AB = ComputeDistance(point.poses[i], point.poses[i-1]);
            AC = ComputeDistance(point.poses[i+1], point.poses[i-1]);
            BC = ComputeDistance(point.poses[i], point.poses[i+1]);
            if (AB + BC != AC) // Aggiungere un eventuale valore minimo per cui non è necessario eseguire la curva
            {
                label[i] = "corner";
            }
        }
    }

    return;
}

void MyTrajectoryPlanner3(TrajectoryPlanner_param param, TrajectoryVector& waypoints)
{    
    std::vector<geometry_msgs::PoseArray> Trajectories_list, Trajectory_list_with_patch;        // Store the trajectory of each point of the file

    geometry_msgs::PoseArray Trajectory_part;    
    std::vector<std::string> Trajectory_id;            
    geometry_msgs::PoseArray Patch_To_Add;

    float PP_distance;          
    double new_Ctrl_Pt_d1;
    double new_Ctrl_Pt_d2;

    geometry_msgs::Pose P1,P2,P3;
    int trajectory_size;
    
    CornerDetection_3(waypoints.point, waypoints.pt_label);
    
    for (std::size_t i = 0; i < waypoints.point.poses.size(); i ++)
    {  
        std::cout << i << "\n";
        std::cout << waypoints.pt_label[i] << "\n";
        if (waypoints.pt_label[i] == "fix")
        {   
            if(i > 2)
            {
                if(waypoints.pt_label[i-1] == "corner")
                {
                    std::cout << "corner fix \n";
                    std::cout << waypoints.point.poses[i] << "\n";
                    std::cout << waypoints.point.poses[i-1] << "\n";
                    std::cout << Trajectories_list[Trajectories_list.size()-1].poses[Trajectories_list[Trajectories_list.size()-1].poses.size()-1] << "\n";

                    CornerFix2(waypoints.point.poses[i], Trajectory_part, waypoints.point.poses[i-1], Trajectories_list[Trajectories_list.size()-1].poses[Trajectories_list[Trajectories_list.size()-1].poses.size()-1], param, 0.05, 0.05, true);
                    std::cout << "fine corner \n";
                    Trajectories_list.push_back(Trajectory_part);
                    Trajectory_part.poses.clear();
                    std::cout << "modifica \n";
                    waypoints.pt_label[i-1] == "cornerfix";
                    Trajectory_id.push_back("cornerfix");
                    
                }
                else
                {
                    addFixPoint2(waypoints.point.poses[i], Trajectory_part, param.radius, param.heigh, param.circ_point, 0.06, 0.04, false);
                    Trajectories_list.push_back(Trajectory_part);
                    Trajectory_part.poses.clear();
                    Trajectory_id.push_back("fix");
                }
            }
            else
            {
                addFixPoint2(waypoints.point.poses[i], Trajectory_part, param.radius, param.heigh, param.circ_point, 0.06, 0.04, false);
                Trajectories_list.push_back(Trajectory_part);
                Trajectory_part.poses.clear();
                Trajectory_id.push_back("fix");
            }
        }

        if(waypoints.pt_label[i] == "pass")
        {
            Trajectory_part.poses.push_back(waypoints.point.poses[i]);
            Trajectories_list.push_back(Trajectory_part);
            Trajectory_part.poses.clear();   
            Trajectory_id.push_back("pass");
        }

        if(waypoints.pt_label[i] == "corner")
        {
            if( i < waypoints.point.poses.size()-1)
            {
                if(waypoints.pt_label[i+1] == "fix")
                {
                    std::cout << "Skip";
                }
                else
                {
                    Trajectory_part.poses.push_back(waypoints.point.poses[i]);
                    Trajectories_list.push_back(Trajectory_part);
                    Trajectory_part.poses.clear();   
                    Trajectory_id.push_back("corner");
                }     
            }
        }
    } 

 
    for(size_t i = 1; i < (Trajectories_list.size()-1); i++)
    {
        if(Trajectory_id[i] == "corner" && Trajectory_id[i+1] != "cornerfix")
        {
            trajectory_size = Trajectories_list[i-1].poses.size();
            P1 = Trajectories_list[i-1].poses[trajectory_size-1];
            P2 = Trajectories_list[i].poses[0];
            P3 = Trajectories_list[i+1].poses[0];

            PP_distance = ComputeDistance(P1, P2);
            if( Trajectory_id[i-1] != "corner")
            {
                if(PP_distance < param.Ctrl_pt_d1)
                    new_Ctrl_Pt_d1 = PP_distance;
                else
                    new_Ctrl_Pt_d1 =  param.Ctrl_pt_d1;
            }
            else
            {
                new_Ctrl_Pt_d1 = PP_distance/2;
            }

            PP_distance = ComputeDistance(P2, P3);
            if(waypoints.pt_label[i+1] != "corner")
            {
                if (PP_distance < param.Ctrl_pt_d2)
                    new_Ctrl_Pt_d2 = PP_distance;
                else 
                    new_Ctrl_Pt_d2 =  param.Ctrl_pt_d2;
            }
            else
            {
                new_Ctrl_Pt_d2 = PP_distance/2;
            }
            
            CornerRounding(P1, P2, P3, new_Ctrl_Pt_d1, new_Ctrl_Pt_d2, Trajectory_part, param.corner_points);
            Trajectories_list[i].poses.clear();
            Trajectories_list[i].poses = Trajectory_part.poses;
            Trajectory_id[i] = "corner";
            Trajectory_part.poses.clear();
        }
    }

    std::vector<std::string> LabelVector_flag;

/**************** INCLUDE PATCH ******************/  

    waypoints.SecondaryTrajectory.push_back(Trajectories_list[0]);
    LabelVector_flag.push_back(Trajectory_id[0]);

    for(size_t i=1; i<Trajectories_list.size(); i++)
    {
        if (ComputePatch(Trajectories_list[i-1].poses.back(), Trajectories_list[i].poses[0], Patch_To_Add, param.res) > 1 )
        {
            waypoints.SecondaryTrajectory.push_back(Patch_To_Add);
            LabelVector_flag.push_back("pass");
        }    
        waypoints.SecondaryTrajectory.push_back(Trajectories_list[i]);   
        LabelVector_flag.push_back(Trajectory_id[i]);     
        Patch_To_Add.poses.clear();
    }

    waypoints.pt_label.clear();
    for(int i=0; i< LabelVector_flag.size(); i++)
    {
        waypoints.pt_label.push_back(LabelVector_flag[i]);
    }
}


void FromEE2Link8(geometry_msgs::PoseArray& waypoints)
{
    /*****
     * Convert the position from EE to Link8
     * 
     *  The offset is specified by gripper_offset (defined outside the function at the beginning)
     *  And a rotation matrix of M_PI / 4 
     * 
     * The Pose is updated directly in the same vector passed
     * 
     */

    geometry_msgs::Pose target_pose;
    tf2::Matrix3x3 R, R_shift, R_final;
    tf2::Quaternion quat, quat_shift, quat_new;
    tf2::Vector3 axis;

    quat.setRPY(0,0,M_PI/4);
    R_shift.setRotation(quat);

    for (size_t i=0; i < waypoints.poses.size(); i++)
    {
        quat.setX(waypoints.poses[i].orientation.x);
        quat.setY(waypoints.poses[i].orientation.y);
        quat.setZ(waypoints.poses[i].orientation.z);
        quat.setW(waypoints.poses[i].orientation.w);
        R.setRotation(quat);

        target_pose.position.x = waypoints.poses[i].position.x -( R[0][0]*gripper_offset[0] + R[0][1]*gripper_offset[1] + R[0][2]*gripper_offset[2]) ;
        target_pose.position.y = waypoints.poses[i].position.y -( R[1][0]*gripper_offset[0] + R[1][1]*gripper_offset[1] + R[1][2]*gripper_offset[2]) ;
        target_pose.position.z = waypoints.poses[i].position.z -( R[2][0]*gripper_offset[0] + R[2][1]*gripper_offset[1] + R[2][2]*gripper_offset[2]) ;

        quat_new = quat*quat_shift;
        quat_new.normalize();

        R_final = R*R_shift;

        R_final.getRotation(quat);
        quat.normalize();
        target_pose.orientation = tf2::toMsg(quat);

        waypoints.poses[i].position    = target_pose.position;
        waypoints.poses[i].orientation = target_pose.orientation;    

    }
    return ;
}

void EE_Shift(geometry_msgs::PoseArray& waypoints)
{
    /*****
     * Convert the position from EE to Link8
     * 
     *  The offset is specified by gripper_offset (defined outside the function at the beginning)
     *  And a rotation matrix of M_PI / 4 
     * 
     * The Pose is updated directly in the same vector passed
     * 
     */

    geometry_msgs::Pose target_pose;

    tf2::Matrix3x3 R, R_shift, R_final;


    tf2::Quaternion quat, quat_shift, quat_new;
    tf2::Vector3 axis;

    double roll, pitch, yaw;

    geometry_msgs::Transform T_shift;


    //quat.setEulerZYX(M_PI/4, 0, 0);

    quat.setRPY(0,0,M_PI);
    R_shift.setRotation(quat);

    for (size_t i=0; i < waypoints.poses.size(); i++)
    {
        quat.setX(waypoints.poses[i].orientation.x);
        quat.setY(waypoints.poses[i].orientation.y);
        quat.setZ(waypoints.poses[i].orientation.z);
        quat.setW(waypoints.poses[i].orientation.w);
        R.setRotation(quat);


        quat_new = quat*quat_shift;
        quat_new.normalize();

        R_final = R*R_shift;

        R_final.getRotation(quat);
        quat.normalize();
        target_pose.orientation = tf2::toMsg(quat);

        waypoints.poses[i].orientation = target_pose.orientation;    

    }
    return ;
}

void FromLink82EE(geometry_msgs::PoseArray& waypoints)
{

    // DA SISTEMARE
    geometry_msgs::Pose target_pose;

    tf2::Matrix3x3 R, R_shift, R_final;
    tf2::Quaternion quat, quat_shift, quat_new;
    tf2::Vector3 axis;

    double roll, pitch, yaw;
    tf2::Transform T, T_origin, T_final;

    geometry_msgs::Transform T_shift;
/*
    T_shift.rotation = tf2::toMsg(quat.setRPY(0,0,M_PI/4));
    T_shift.translation.x = 0;
    T_shift.translation.y = 0;
    T_shift.translation.z = -0.1;

    quat_shift.setRPY(0,0,M_PI/4);
    T.setRotation(quat_shift);
*/
    quat.setRPY(0,0,M_PI/4);
    R_shift.setRotation(quat);

    for (size_t i=0; i < waypoints.poses.size(); i++)
    {

        quat.setX(waypoints.poses[i].orientation.x);
        quat.setY(waypoints.poses[i].orientation.y);
        quat.setZ(waypoints.poses[i].orientation.z);
        quat.setW(waypoints.poses[i].orientation.w);
        R.setRotation(quat);
     //   T_origin.setRotation(quat);

        target_pose.position.x = waypoints.poses[i].position.x +( R[0][0]*gripper_offset[0] + R[0][1]*gripper_offset[1] + R[0][2]*gripper_offset[2]) ;
        target_pose.position.y = waypoints.poses[i].position.y +( R[1][0]*gripper_offset[0] + R[1][1]*gripper_offset[1] + R[1][2]*gripper_offset[2]) ;
        target_pose.position.z = waypoints.poses[i].position.z +( R[2][0]*gripper_offset[0] + R[2][1]*gripper_offset[1] + R[2][2]*gripper_offset[2]) ;

    //    quat_new = quat*quat_shift;
        quat_new.normalize();

    //    T_final = T_origin*T;
    //    R_final = R_shift*R;
    //    R_final.getEulerZYX(roll, pitch, yaw);
    //    quat_new.setRPY(roll, pitch, yaw);
        
        axis.setW(quat.getW());
        axis.setX(quat.getX());
        axis.setY(quat.getY());
        axis.setZ(quat.getZ());



        quat.setRotation(axis, M_PI);
        target_pose.orientation = tf2::toMsg(quat);
        std::cout << quat << "\n";

        waypoints.poses[i].position    = target_pose.position;
        waypoints.poses[i].orientation = target_pose.orientation;      
    }
    return ;    

}

void addFixPoint(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, double radius, double heigh, int res)
{
    /**
     * Create a trajectory to fix a point
     * 
     * The trajectory is created starting from the pose "fixing_point" specified and added in "waypoints" through .push_back
     * The movements performed are:
     *      lateral right shift of "2*radius" distance
     *      raise of "heigh"
     *      Circular movement to position over the fix point
     *      lower
     */

    geometry_msgs::Pose target_pose;
    geometry_msgs::Point center;

    tf2::Matrix3x3 R;
    tf2::Quaternion quat;

    quat.setX(fixing_point.orientation.x);
    quat.setY(fixing_point.orientation.y);
    quat.setZ(fixing_point.orientation.z);
    quat.setW(fixing_point.orientation.w);
    R.setRotation(quat);

    double dist_init[3] = {0.1, 0, 0};

    target_pose.position.x = fixing_point.position.x - (R[0][0]*dist_init[0] + R[0][1]*dist_init[1] + R[0][1]*dist_init[2]);
    target_pose.position.y = fixing_point.position.y - (R[1][0]*dist_init[0] + R[0][1]*dist_init[1] + R[0][1]*dist_init[2]);
    target_pose.position.z = fixing_point.position.z - (R[2][0]*dist_init[0] + R[0][1]*dist_init[1] + R[0][1]*dist_init[2]);
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);
    
    target_pose.position.x = fixing_point.position.x + R[0][1]*radius*2;
    target_pose.position.y = fixing_point.position.y + R[1][1]*radius*2;
    target_pose.position.z = fixing_point.position.z + R[2][1]*radius*2;
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);


    // lateral shift of the diameter of the circle
    target_pose.position.x = fixing_point.position.x + R[0][1]*radius*2;
    target_pose.position.y = fixing_point.position.y + R[1][1]*radius*2;
    target_pose.position.z = fixing_point.position.z + R[2][1]*radius*2;
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);

    // Half-circle, the minus in the "heigh" is required since the z axies is taken pointing down
    center.x = fixing_point.position.x + R[0][1]*radius - R[0][2]*heigh;
    center.y = fixing_point.position.y + R[1][1]*radius - R[1][2]*heigh;
    center.z = fixing_point.position.z + R[2][1]*radius - R[2][2]*heigh;

    /*
    center.x = fixing_point.position.x + R[0][0]*radius[0] + R[0][1]*radius[1] + R[0][1]*radius[2] - (R[0][0]*heigh[0] + R[0][1]*heigh[1] + R[0][1]*heigh[2]);
    center.y = fixing_point.position.y + R[1][0]*radius[0] + R[0][1]*radius[1] + R[0][1]*radius[2] - (R[0][0]*heigh[0] + R[0][1]*heigh[1] + R[0][1]*heigh[2]);
    center.z = fixing_point.position.z + R[2][0]*radius[0] + R[0][1]*radius[1] + R[0][1]*radius[2] - (R[0][0]*heigh[0] + R[0][1]*heigh[1] + R[0][1]*heigh[2]);
    */

    // Upshift
    target_pose.position.x = target_pose.position.x - R[0][2]*heigh;
    target_pose.position.y = target_pose.position.y - R[1][2]*heigh;
    target_pose.position.z = target_pose.position.z - R[2][2]*heigh;
    waypoints.poses.push_back(target_pose);

    for(int k = 0; k < res; k++)
    {
        target_pose.position.x = center.x + R[0][1]*radius*cos(M_PI/res*k) - R[0][2]*radius*sin(M_PI/res*k);
        target_pose.position.y = center.y + R[1][1]*radius*cos(M_PI/res*k) - R[1][2]*radius*sin(M_PI/res*k);
        target_pose.position.z = center.z + R[2][1]*radius*cos(M_PI/res*k) - R[2][2]*radius*sin(M_PI/res*k);
        waypoints.poses.push_back(target_pose);
    }

    target_pose.position.x = fixing_point.position.x - R[0][2]*heigh;
    target_pose.position.y = fixing_point.position.y - R[1][2]*heigh;
    target_pose.position.z = fixing_point.position.z - R[2][2]*heigh;
    waypoints.poses.push_back(target_pose);

    // Downshift
    target_pose.position.x = fixing_point.position.x;
    target_pose.position.y = fixing_point.position.y;
    target_pose.position.z = fixing_point.position.z;
    waypoints.poses.push_back(target_pose);

    target_pose.position.x = fixing_point.position.x + (R[0][0]*dist_init[0] + R[0][1]*dist_init[1] + R[0][1]*dist_init[2]);
    target_pose.position.y = fixing_point.position.y + (R[1][0]*dist_init[0] + R[0][1]*dist_init[1] + R[0][1]*dist_init[2]);
    target_pose.position.z = fixing_point.position.z + (R[2][0]*dist_init[0] + R[0][1]*dist_init[1] + R[0][1]*dist_init[2]);
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);

    return ;
}

void addFixPoint2(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, double radius, double heigh, int res, double dis_init, double dist_final, bool orientation)
{
    /**
     * Create a trajectory to fix a point
     * 
     * The trajectory is created starting from the pose "fixing_point" specified and added in "waypoints" through .push_back
     * The movements performed are:
     *      lateral right shift of "2*radius" distance
     *      raise of "heigh"
     *      Circular movement to position over the fix point
     *      lower
     * 
     * orientation = true  avvicinamento da destra
     * orientation = false avvicinamento da sinistra
     * 
     *      * CON SPOSTAMENTO IN AVANTI

     */

    geometry_msgs::Pose target_pose;
    geometry_msgs::Point center;

    tf2::Matrix3x3 R;
    tf2::Quaternion quat;

    quat.setX(fixing_point.orientation.x);
    quat.setY(fixing_point.orientation.y);
    quat.setZ(fixing_point.orientation.z);
    quat.setW(fixing_point.orientation.w);
    R.setRotation(quat);

    double dist_init_vector[3] = {dis_init, 0, 0};
    double dist_final_vector[3] = {dist_final, 0 ,0};

    target_pose.position.x = fixing_point.position.x - (R[0][0]*dist_init_vector[0] + R[0][1]*dist_init_vector[1] + R[0][1]*dist_init_vector[2]);
    target_pose.position.y = fixing_point.position.y - (R[1][0]*dist_init_vector[0] + R[1][1]*dist_init_vector[1] + R[1][1]*dist_init_vector[2]);
    target_pose.position.z = fixing_point.position.z - (R[2][0]*dist_init_vector[0] + R[2][1]*dist_init_vector[1] + R[2][1]*dist_init_vector[2]) - 0.02;    // !!!
   // target_pose.position.z = fixing_point.position.z - (R[2][0]*dist_init_vector[0] + R[2][1]*dist_init_vector[1] + R[2][1]*dist_init_vector[2]);
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);

    if (orientation)
    {
        // lateral shift of the diameter of the circle
        target_pose.position.x = fixing_point.position.x + R[0][1]*radius*2;
        target_pose.position.y = fixing_point.position.y + R[1][1]*radius*2;
        target_pose.position.z = fixing_point.position.z + R[2][1]*radius*2 - 0.02;
        // target_pose.position.z = fixing_point.position.z + R[2][1]*radius*2;
        target_pose.orientation = fixing_point.orientation;
        waypoints.poses.push_back(target_pose);

        // Half-circle, the minus in the "heigh" is required since the z axies is taken pointing down
        center.x = fixing_point.position.x + R[0][1]*radius - R[0][2]*heigh;
        center.y = fixing_point.position.y + R[1][1]*radius - R[1][2]*heigh;
        center.z = fixing_point.position.z + R[2][1]*radius - R[2][2]*heigh;
        
        // Upshift
        /*
        target_pose.position.x = target_pose.position.x - R[0][2]*heigh;
        target_pose.position.y = target_pose.position.y - R[1][2]*heigh;
        target_pose.position.z = target_pose.position.z - R[2][2]*heigh;
        waypoints.poses.push_back(target_pose);
        */
        for(int k = 0; k < res; k++)
        {
            target_pose.position.x = center.x + R[0][1]*radius*cos(M_PI/res*k) - R[0][2]*radius*sin(M_PI/res*k) + (R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][2]*dist_final_vector[2])*k/res ; // Piccolo movimento in avanti
            target_pose.position.y = center.y + R[1][1]*radius*cos(M_PI/res*k) - R[1][2]*radius*sin(M_PI/res*k) + (R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][2]*dist_final_vector[2])*k/res;
            target_pose.position.z = center.z + R[2][1]*radius*cos(M_PI/res*k) - R[2][2]*radius*sin(M_PI/res*k) + (R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][2]*dist_final_vector[2])*k/res;
            waypoints.poses.push_back(target_pose);
        }
    }
    else
    {
        // lateral shift of the diameter of the circle
        target_pose.position.x = fixing_point.position.x - R[0][1]*radius*2;
        target_pose.position.y = fixing_point.position.y - R[1][1]*radius*2;
        target_pose.position.z = fixing_point.position.z - R[2][1]*radius*2 -0.02;
        target_pose.orientation = fixing_point.orientation;
        waypoints.poses.push_back(target_pose);

        // Half-circle, the minus in the "heigh" is required since the z axies is taken pointing down
        center.x = fixing_point.position.x - R[0][1]*radius - R[0][2]*heigh;
        center.y = fixing_point.position.y - R[1][1]*radius - R[1][2]*heigh;
        center.z = fixing_point.position.z - R[2][1]*radius - R[2][2]*heigh;
        
        // Upshift
        target_pose.position.x = target_pose.position.x - R[0][2]*heigh;
        target_pose.position.y = target_pose.position.y - R[1][2]*heigh;
        target_pose.position.z = target_pose.position.z - R[2][2]*heigh;
        waypoints.poses.push_back(target_pose);

        for(int k = 0; k < res; k++)
        {
            target_pose.position.x = center.x - R[0][1]*radius*cos(M_PI/res*k) - R[0][2]*radius*sin(M_PI/res*k) + (R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][2]*dist_final_vector[2])*k/res ; // Piccolo movimento in avanti
            target_pose.position.y = center.y - R[1][1]*radius*cos(M_PI/res*k) - R[1][2]*radius*sin(M_PI/res*k) + (R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][2]*dist_final_vector[2])*k/res;
            target_pose.position.z = center.z - R[2][1]*radius*cos(M_PI/res*k) - R[2][2]*radius*sin(M_PI/res*k) + (R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][2]*dist_final_vector[2])*k/res;
            waypoints.poses.push_back(target_pose);
        }
    }
    

    /*
    center.x = fixing_point.position.x + R[0][0]*radius[0] + R[0][1]*radius[1] + R[0][1]*radius[2] - (R[0][0]*heigh[0] + R[0][1]*heigh[1] + R[0][1]*heigh[2]);
    center.y = fixing_point.position.y + R[1][0]*radius[0] + R[0][1]*radius[1] + R[0][1]*radius[2] - (R[0][0]*heigh[0] + R[0][1]*heigh[1] + R[0][1]*heigh[2]);
    center.z = fixing_point.position.z + R[2][0]*radius[0] + R[0][1]*radius[1] + R[0][1]*radius[2] - (R[0][0]*heigh[0] + R[0][1]*heigh[1] + R[0][1]*heigh[2]);
    */



    target_pose.position.x = fixing_point.position.x - R[0][2]*heigh + R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][1]*dist_final_vector[2];
    target_pose.position.y = fixing_point.position.y - R[1][2]*heigh + R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][1]*dist_final_vector[2];
    target_pose.position.z = fixing_point.position.z - R[2][2]*heigh + R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][1]*dist_final_vector[2];
    waypoints.poses.push_back(target_pose);

    // Downshift
    target_pose.position.x = fixing_point.position.x + R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][1]*dist_final_vector[2];
    target_pose.position.y = fixing_point.position.y + R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][1]*dist_final_vector[2];
    target_pose.position.z = fixing_point.position.z + R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][1]*dist_final_vector[2];
    waypoints.poses.push_back(target_pose);
/*
    target_pose.position.x = fixing_point.position.x + (R[0][0]*dist_init_vector[0] + R[0][1]*dist_init_vector[1] + R[0][1]*dist_init_vector[2]);
    target_pose.position.y = fixing_point.position.y + (R[1][0]*dist_init_vector[0] + R[0][1]*dist_init_vector[1] + R[0][1]*dist_init_vector[2]);
    target_pose.position.z = fixing_point.position.z + (R[2][0]*dist_init_vector[0] + R[0][1]*dist_init_vector[1] + R[0][1]*dist_init_vector[2]);
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);
*/
    return ;
}

void addFixPoint3(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, double radius, double heigh, int res, double dis_init, double dist_final, bool orientation)
{
        /**
     * Create a trajectory to fix a point
     * 
     * The trajectory is created starting from the pose "fixing_point" specified and added in "waypoints" through .push_back
     * The movements performed are:
     *      lateral right shift of "2*radius" distance
     *      raise of "heigh"
     *      Circular movement to position over the fix point
     *      lower
     * 
     * orientation = true  avvicinamento da destra
     * orientation = false avvicinamento da sinistra
     * 
     * 
     */

    geometry_msgs::Pose target_pose;
    geometry_msgs::Point center;

    tf2::Matrix3x3 R;
    tf2::Quaternion quat;

    quat.setX(fixing_point.orientation.x);
    quat.setY(fixing_point.orientation.y);
    quat.setZ(fixing_point.orientation.z);
    quat.setW(fixing_point.orientation.w);
    R.setRotation(quat);

    double dist_init_vector[3] = {dis_init, 0, 0};
    double dist_final_vector[3] = {dist_final, 0 ,0};

    target_pose.position.x = fixing_point.position.x - (R[0][0]*dist_init_vector[0] + R[0][1]*dist_init_vector[1] + R[0][2]*dist_init_vector[2]);
    target_pose.position.y = fixing_point.position.y - (R[1][0]*dist_init_vector[0] + R[1][1]*dist_init_vector[1] + R[1][2]*dist_init_vector[2]);
    target_pose.position.z = fixing_point.position.z - (R[2][0]*dist_init_vector[0] + R[2][1]*dist_init_vector[1] + R[2][2]*dist_init_vector[2]);
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);

    // lateral shift of the diameter of the circle
    target_pose.position.x = fixing_point.position.x - R[0][1]*radius*2 + (R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][2]*dist_final_vector[2]);
    target_pose.position.y = fixing_point.position.y - R[1][1]*radius*2 + (R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][2]*dist_final_vector[2]);
    target_pose.position.z = fixing_point.position.z - R[2][1]*radius*2 + (R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][2]*dist_final_vector[2]);
    target_pose.orientation = fixing_point.orientation;
    waypoints.poses.push_back(target_pose);
       

    // Upshift (Earlier point + heigh)
    target_pose.position.x = target_pose.position.x - R[0][2]*heigh;
    target_pose.position.y = target_pose.position.y - R[1][2]*heigh;
    target_pose.position.z = target_pose.position.z - R[2][2]*heigh;
    waypoints.poses.push_back(target_pose);


    // Half-circle, the minus in the "heigh" is required since the z axies is taken pointing down
    center.x = target_pose.position.x + R[0][1]*radius;
    center.y = target_pose.position.y + R[1][1]*radius;
    center.z = target_pose.position.z + R[2][1]*radius;
        

    for(int k = 0; k < res; k++)
    {
        target_pose.position.x = center.x - R[0][1]*radius*cos(M_PI/res*k) - R[0][2]*radius*sin(M_PI/res*k);
        target_pose.position.y = center.y - R[1][1]*radius*cos(M_PI/res*k) - R[1][2]*radius*sin(M_PI/res*k);
        target_pose.position.z = center.z - R[2][1]*radius*cos(M_PI/res*k) - R[2][2]*radius*sin(M_PI/res*k);
        waypoints.poses.push_back(target_pose);
    }


    target_pose.position.x = fixing_point.position.x - R[0][2]*heigh + R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][1]*dist_final_vector[2];
    target_pose.position.y = fixing_point.position.y - R[1][2]*heigh + R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][1]*dist_final_vector[2];
    target_pose.position.z = fixing_point.position.z - R[2][2]*heigh + R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][1]*dist_final_vector[2];
    waypoints.poses.push_back(target_pose);

    // Downshift
    target_pose.position.x = fixing_point.position.x + R[0][0]*dist_final_vector[0] + R[0][1]*dist_final_vector[1] + R[0][1]*dist_final_vector[2];
    target_pose.position.y = fixing_point.position.y + R[1][0]*dist_final_vector[0] + R[1][1]*dist_final_vector[1] + R[1][1]*dist_final_vector[2];
    target_pose.position.z = fixing_point.position.z + R[2][0]*dist_final_vector[0] + R[2][1]*dist_final_vector[1] + R[2][1]*dist_final_vector[2];
    waypoints.poses.push_back(target_pose);

    return ;

}

void CornerFix(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, geometry_msgs::Pose corner_point, geometry_msgs::Pose starting_point, TrajectoryPlanner_param param, double dis_init, double dist_final, bool orientation)
{
    geometry_msgs::PoseArray CornerPath;    

    float PP_distance;          
    double new_Ctrl_Pt_d1;
    double new_Ctrl_Pt_d2;
    geometry_msgs::Pose P1,P2,P3;

    float corner_length;

    float dist_init_vector[3]   ={float(dis_init), 0, 0};
    float dist_final_vector[3] = {float(dist_final), 0, 0};
        
    P1 = starting_point;
    P2 = corner_point;
    P3 = fixing_point;
    PP_distance = ComputeDistance(P1, P2);
    
    if(PP_distance < param.Ctrl_pt_d1)
        new_Ctrl_Pt_d1 = PP_distance;
    else
        new_Ctrl_Pt_d1 =  param.Ctrl_pt_d1;

    PP_distance = ComputeDistance(P2, P3);
    if (PP_distance < param.Ctrl_pt_d2)
        new_Ctrl_Pt_d2 = PP_distance;
    else 
        new_Ctrl_Pt_d2 =  param.Ctrl_pt_d2;
    
    // Calcolo della lunghezza totale 
    CornerRounding(P1, P2, P3, new_Ctrl_Pt_d1, new_Ctrl_Pt_d2, CornerPath, param.corner_points);
    corner_length = ComputeDistance(P1,P2) + ComputeDistance(P2,P3) + dist_final;


    tf2::Quaternion quat;
    tf2::fromMsg(starting_point.orientation, quat);
    tf2::Matrix3x3 Rot;
    Rot.setRotation(quat);
    

    geometry_msgs::Pose target_pose;
    geometry_msgs::PoseArray Trajectory;

    corner_length = ComputeDistance(P1,P2) + dist_final + ComputeDistance(P2,P3);
    target_pose.position.x = starting_point.position.x + Rot[0][0]*corner_length;
    target_pose.position.y = starting_point.position.y + Rot[1][0]*corner_length;
    target_pose.position.z = starting_point.position.z + Rot[2][0]*corner_length;
    target_pose.orientation = starting_point.orientation;
    ComputePatch(starting_point, target_pose, waypoints, param.res);
    
    // Rotation

   // corner_length = ComputeDistance(P2,P3) + dist_final;
    corner_length = dist_final + ComputeDistance(P2,P3); 
    float point_numbers = (M_PI/2*corner_length)/param.res;
    float t;
    float a_angle = asin(param.radius/corner_length);


    geometry_msgs::Point center;

    center.x = corner_point.position.x;
    center.y = corner_point.position.y;
    center.z = corner_point.position.z;

    for(int i=1; i < point_numbers; i++)
    {
        t=1/point_numbers*i;
        target_pose.position.x = center.x + corner_length*Rot[0][0]*cos(M_PI/2*t) + corner_length*Rot[0][1]*sin(M_PI/2*t);
        target_pose.position.y = center.y + corner_length*Rot[1][0]*cos(M_PI/2*t) + corner_length*Rot[1][1]*sin(M_PI/2*t);
        target_pose.position.z = center.z + corner_length*Rot[2][0]*cos(M_PI/2*t) + corner_length*Rot[2][1]*sin(M_PI/2*t) + param.heigh*t + param.radius*t;
        target_pose.orientation = slerp(starting_point.orientation, fixing_point.orientation, t);

        waypoints.poses.push_back(target_pose);
    }

    // Downshift
    tf2::fromMsg(fixing_point.orientation, quat);
    Rot.setRotation(quat);
    target_pose.position.x = fixing_point.position.x + Rot[0][0]*dist_final_vector[0] + Rot[0][1]*dist_final_vector[1] + Rot[0][2]*dist_final_vector[2];
    target_pose.position.y = fixing_point.position.y + Rot[1][0]*dist_final_vector[0] + Rot[1][1]*dist_final_vector[1] + Rot[1][2]*dist_final_vector[2];
    target_pose.position.z = fixing_point.position.z + Rot[2][0]*dist_final_vector[0] + Rot[2][1]*dist_final_vector[1] + Rot[2][2]*dist_final_vector[2];
    waypoints.poses.push_back(target_pose);

}

void CornerFix2(geometry_msgs::Pose fixing_point, geometry_msgs::PoseArray& waypoints, geometry_msgs::Pose corner_point, geometry_msgs::Pose starting_point, TrajectoryPlanner_param param, double dis_init, double dist_final, bool orientation)
{

    // modifica per evitare il piolo


    geometry_msgs::PoseArray CornerPath;    

    float PP_distance;          
    double new_Ctrl_Pt_d1;
    double new_Ctrl_Pt_d2;
    geometry_msgs::Pose P1,P2,P3;

    float corner_length;

    float dist_init_vector[3]   ={float(dis_init), 0, 0};
    float dist_final_vector[3] = {float(dist_final), 0, 0};
        
    P1 = starting_point;
    P2 = corner_point;
    P3 = fixing_point;
    PP_distance = ComputeDistance(P1, P2);
    
    if(PP_distance < param.Ctrl_pt_d1)
        new_Ctrl_Pt_d1 = PP_distance;
    else
        new_Ctrl_Pt_d1 =  param.Ctrl_pt_d1;

    PP_distance = ComputeDistance(P2, P3);
    if (PP_distance < param.Ctrl_pt_d2)
        new_Ctrl_Pt_d2 = PP_distance;
    else 
        new_Ctrl_Pt_d2 =  param.Ctrl_pt_d2;
    
    // Calcolo della lunghezza totale 
    CornerRounding(P1, P2, P3, new_Ctrl_Pt_d1, new_Ctrl_Pt_d2, CornerPath, param.corner_points);
    corner_length = ComputeDistance(P1,P2) + ComputeDistance(P2,P3) + dist_final;


    tf2::Quaternion quat;
    tf2::fromMsg(starting_point.orientation, quat);
    tf2::Matrix3x3 Rot;
    Rot.setRotation(quat);
    

    geometry_msgs::Pose target_pose;
    geometry_msgs::PoseArray Trajectory;


    geometry_msgs::Pose intermediate_pose;

    intermediate_pose.position.x = corner_point.position.x;
    intermediate_pose.position.y = corner_point.position.y + 0.03;
    intermediate_pose.position.z = corner_point.position.z - 0.02;
    intermediate_pose.orientation = starting_point.orientation;
    ComputePatch(starting_point, intermediate_pose, waypoints, param.res);
    
    corner_length = ComputeDistance(P1,P2) + dist_final + ComputeDistance(P2,P3);
    target_pose.position.x = starting_point.position.x + Rot[0][0]*corner_length;
    target_pose.position.y = starting_point.position.y + Rot[1][0]*corner_length;
  //  target_pose.position.z = starting_point.position.z + Rot[2][0]*corner_length - 0.02;
    target_pose.position.z = starting_point.position.z + Rot[2][0]*corner_length;
    target_pose.orientation = starting_point.orientation;
    ComputePatch(intermediate_pose, target_pose, waypoints, param.res);



/*
    corner_length = ComputeDistance(P1,P2) + dist_final + ComputeDistance(P2,P3);
    target_pose.position.x = starting_point.position.x + Rot[0][0]*corner_length;
    target_pose.position.y = starting_point.position.y + Rot[1][0]*corner_length;
    target_pose.position.z = starting_point.position.z + Rot[2][0]*corner_length;
    target_pose.orientation = starting_point.orientation;
    ComputePatch(starting_point, target_pose, waypoints, param.res);
*/    
    // Rotation

    // corner_length = ComputeDistance(P2,P3) + dist_final;
    corner_length = dist_final + ComputeDistance(P2,P3); 
    float point_numbers = (M_PI/2*corner_length)/param.res;
    float t;
    float a_angle = asin(param.radius/corner_length);


    geometry_msgs::Point center;

    center.x = corner_point.position.x;
    center.y = corner_point.position.y;
    center.z = corner_point.position.z;

    for(int i=1; i < point_numbers; i++)
    {
        t=1/point_numbers*i;
        target_pose.position.x = center.x + corner_length*Rot[0][0]*cos(M_PI/2*t) + corner_length*Rot[0][1]*sin(M_PI/2*t);
        target_pose.position.y = center.y + corner_length*Rot[1][0]*cos(M_PI/2*t) + corner_length*Rot[1][1]*sin(M_PI/2*t);
        target_pose.position.z = center.z + corner_length*Rot[2][0]*cos(M_PI/2*t) + corner_length*Rot[2][1]*sin(M_PI/2*t) + param.heigh*t + 0.015*t;
        target_pose.orientation = slerp(starting_point.orientation, fixing_point.orientation, t);

        waypoints.poses.push_back(target_pose);
    }

    // Downshift
    tf2::fromMsg(fixing_point.orientation, quat);
    Rot.setRotation(quat);
    target_pose.position.x = fixing_point.position.x + Rot[0][0]*dist_final_vector[0] + Rot[0][1]*dist_final_vector[1] + Rot[0][2]*dist_final_vector[2];
    target_pose.position.y = fixing_point.position.y + Rot[1][0]*dist_final_vector[0] + Rot[1][1]*dist_final_vector[1] + Rot[1][2]*dist_final_vector[2];
    target_pose.position.z = fixing_point.position.z + Rot[2][0]*dist_final_vector[0] + Rot[2][1]*dist_final_vector[1] + Rot[2][2]*dist_final_vector[2];
    waypoints.poses.push_back(target_pose);





}

void CornerDetection(geometry_msgs::PoseArray waypoints, std::vector<std::string>& id_list)
{
    float det1, det2;
    for(int i = 1; i < (waypoints.poses.size()-1); i++)
    {
        // Condizione sul determinante
        det1 =  (waypoints.poses[i].position.x-waypoints.poses[i-1].position.x)*(waypoints.poses[i+1].position.y-waypoints.poses[i-1].position.y) -
                (waypoints.poses[i+1].position.x-waypoints.poses[i-1].position.x)*(waypoints.poses[i].position.y-waypoints.poses[i-1].position.y);
        
        det2 =  (waypoints.poses[i].position.x-waypoints.poses[i-1].position.x)*(waypoints.poses[i+1].position.z-waypoints.poses[i-1].position.y) -
                (waypoints.poses[i+1].position.x-waypoints.poses[i-1].position.x)*(waypoints.poses[i].position.z-waypoints.poses[i-1].position.y);

        // abs(det1) return always 0 by an error becouse the function enter in conflict 
        // since is available in multiple library called and use the ones that returns 1 in case of double value

        if (det1<0) det1 = -det1;
        if (det2<0) det2 = -det2;
        // When both are = 0 the points are alligned, the condition is denied and then add 0.01 as safety in case of numerical error rounding of the number
        
        if(det1 > 0.01 || det2 > 0.01)
        {
            if(id_list[i] == "pass")
            {
                id_list[i] = "rounding";
            }
        }
    }
}

void CornerRounding(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2, geometry_msgs::Pose Point3, double distance1, double distance2, geometry_msgs::PoseArray& waypoints, double res)
{
    /*
    *   The function read three points, in which "Point2" is the one in the middle
    * 
    *   It is applied a " Quadratic Bézier" curves aproach using as starting point the Point2 shifted by "distance1" in the direction of "Point1"
    *   The terminal point will be Point2 shifted by "distance2" in the direction of Point3
    * 
    *   The point are saved via .push_back in "waypoints"
    * 
    *   To compute the intermediate point is made an interpolation of the xyz coordinates (see Bèzier curves)
    *   The orientation updated through the slerp algorithm 
    * 
    * */

    geometry_msgs::Pose point1_shift, point2_shift, target_point;
    geometry_msgs::Point delta_1, delta_2;
    geometry_msgs::Point P_1, P_2;
    tf2::Quaternion Tg_Pose_Orientation1, Tg_Pose_Orientation2;
    double norm_1, norm_2;
    double d1, d2;
    float t;                // Used in for cycle €[0,1]

    // Compute the distance beetween the points with respect to x,y,z
    delta_1.x = Point2.position.x - Point1.position.x;
    delta_1.y = Point2.position.y - Point1.position.y;
    delta_1.z = Point2.position.z - Point1.position.z;
    delta_2.x = Point3.position.x - Point2.position.x;
    delta_2.y = Point3.position.y - Point2.position.y;
    delta_2.z = Point3.position.z - Point2.position.z;    

    // Check if the shifting distance to start the cornering is greater than the distance beetween the two points to avoid to go backward
    norm_1 = sqrt(delta_1.x*delta_1.x + delta_1.y*delta_1.y + delta_1.z*delta_1.z);    
    if (norm_1 > distance1)
    {
        d1 = distance1; 
        point1_shift.position.x = Point1.position.x + (norm_1-d1)/norm_1*(Point2.position.x - Point1.position.x);
        point1_shift.position.y = Point1.position.y + (norm_1-d1)/norm_1*(Point2.position.y - Point1.position.y);
        point1_shift.position.z = Point1.position.z + (norm_1-d1)/norm_1*(Point2.position.z - Point1.position.z);
        point1_shift.orientation = Point1.orientation;
        waypoints.poses.push_back(point1_shift);
    }
    else
    {
        d1 = norm_1;
        point1_shift = Point1;
    }

    norm_2 = sqrt(delta_2.x*delta_2.x + delta_2.y*delta_2.y + delta_2.z*delta_2.z);    
    if (norm_2 > distance2)
    {
        d2 = distance2;
        point2_shift.position.x = Point2.position.x + d2/norm_2*(Point3.position.x - Point2.position.x);
        point2_shift.position.y = Point2.position.y + d2/norm_2*(Point3.position.y - Point2.position.y);
        point2_shift.position.z = Point2.position.z + d2/norm_2*(Point3.position.z - Point2.position.z);
        point2_shift.orientation = Point3.orientation;
        // No push_back here becouse id done later
    }
    else
    {
        d2 = norm_2;
        point2_shift = Point3;
    }
    tf2::fromMsg(Point1.orientation, Tg_Pose_Orientation1);
    tf2::fromMsg(Point3.orientation, Tg_Pose_Orientation2);

    // Cornering function using bèzier quadratic function
    for (int i = 0; i < res; i++)
    {
        t = 1/res*i;
        // P1 + (1-t)^2(P0-P1)+t^2(P2-P1)
        target_point.position.x = Point2.position.x + (1-t)*(1-t)*(point1_shift.position.x - Point2.position.x) + t*t*(point2_shift.position.x - Point2.position.x);
        target_point.position.y = Point2.position.y + (1-t)*(1-t)*(point1_shift.position.y - Point2.position.y) + t*t*(point2_shift.position.y - Point2.position.y);
        target_point.position.z = Point2.position.z + (1-t)*(1-t)*(point1_shift.position.z - Point2.position.z) + t*t*(point2_shift.position.z - Point2.position.z);
//        target_point.orientation = slerp(Point1.orientation, Point2.orientation,t); 
        target_point.orientation = tf2::toMsg(Tg_Pose_Orientation1.slerp(Tg_Pose_Orientation2,t));
        waypoints.poses.push_back(target_point);
    }


    if (norm_2 > d2)
    {
        waypoints.poses.push_back(point2_shift);
    }
}

int ComputePatch(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2, geometry_msgs::PoseArray& waypoints, double res)
{
    /*
    *   Given two points as input returns a series of points with a distance of res beetwen 
    *   the two points included.
    */ 

    ROS_INFO("COMPUTE PATCH");
    geometry_msgs::Pose target_pose;
    geometry_msgs::Point delta;
    tf2::Quaternion Tg_Pose_Orientation1, Tg_Pose_Orientation2;
    double distance;
    double t;                // Used in for cycle €[0,1]
    double step;

    int patch_added = 0;

    // Compute the distance beetween the points with respect to x,y,z
    delta.x = Point2.position.x - Point1.position.x;
    delta.y = Point2.position.y - Point1.position.y;
    delta.z = Point2.position.z - Point1.position.z;

    distance = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);      
    step = distance/res;

    tf2::fromMsg(Point1.orientation, Tg_Pose_Orientation1);
    tf2::fromMsg(Point2.orientation, Tg_Pose_Orientation2);

    for(int i = 1; i < step; i++)
    {
        t = 1/(step)*i;
        target_pose.position.x = Point1.position.x + t*delta.x;
        target_pose.position.y = Point1.position.y + t*delta.y;
        target_pose.position.z = Point1.position.z + t*delta.z;
        target_pose.orientation = Point2.orientation;
        target_pose.orientation = tf2::toMsg(Tg_Pose_Orientation1.slerp(Tg_Pose_Orientation2,t));
        waypoints.poses.push_back(target_pose);
        patch_added++;
    }
    return patch_added;
}

void CornerDetection_test(std::vector<geometry_msgs::PoseArray> waypoints, std::vector<std::string>& id_list)
{
    float det1, det2;
    geometry_msgs::Pose P1, P3;
    int Traj1_size, Traj3_size;

    float AB, AC, BC;

    for(int i = 1; i < (waypoints.size()-1); i++)
    {
       // if(id_list[i] == "pass" && id_list[i-1] != "grasp")
        if(id_list[i] == "pass" && id_list[i-1] != "grasp")
        {
             Traj1_size = waypoints[i-1].poses.size();

            AB = ComputeDistance(waypoints[i].poses[0], waypoints[i-1].poses[Traj1_size]);
            AC = ComputeDistance(waypoints[i+1].poses[0], waypoints[i-1].poses[Traj1_size]);
            BC = ComputeDistance(waypoints[i].poses[0], waypoints[i+1].poses[0]);
            if (AB + BC != AC) // Aggiungere un eventuale valore minimo per cui non è necessario eseguire la curva
            {
                id_list[i] = "corner";
            }
        }
    }

    return;
}

moveit_msgs::RobotTrajectory FactorScaling(moveit_msgs::RobotTrajectory trajectory_cartesian, float scaling_factor, ros::Duration T_offset)
{
    moveit_msgs::RobotTrajectory new_trajectory;
    trajectory_msgs::JointTrajectoryPoint Traj_Point;

    for(size_t i=0; i<trajectory_cartesian.joint_trajectory.points.size(); i++)
    {
        new_trajectory.joint_trajectory.points.push_back(trajectory_cartesian.joint_trajectory.points[i]);
    }
    new_trajectory.joint_trajectory.joint_names.resize(7);
    new_trajectory.joint_trajectory.joint_names = trajectory_cartesian.joint_trajectory.joint_names;

    for (size_t i=0; i<new_trajectory.joint_trajectory.points.size(); i++)
    {
        for(int k=0; k<7; k++)
        {
            new_trajectory.joint_trajectory.points[i].velocities[k] /= scaling_factor;
            new_trajectory.joint_trajectory.points[i].accelerations[k] /= (scaling_factor*scaling_factor);
        }
        new_trajectory.joint_trajectory.points[i].time_from_start *= scaling_factor;
    }


    Traj_Point.time_from_start = ros::Duration(0);
    Traj_Point.positions.resize(7);
    Traj_Point.velocities.resize(7);
    Traj_Point.accelerations.resize(7);

    for(int i=0; i<7; i++)
    {
        Traj_Point.positions[i] = new_trajectory.joint_trajectory.points[0].positions[i];
        Traj_Point.velocities[i] = 0;
        Traj_Point.accelerations[i] = 0;
    }

    for (size_t i=0; i<new_trajectory.joint_trajectory.points.size(); i++)
    {
        new_trajectory.joint_trajectory.points[i].time_from_start += T_offset;
    }

    for(size_t i=1; i<new_trajectory.joint_trajectory.points.size(); i++)
    {
        if(new_trajectory.joint_trajectory.points[i].time_from_start <= new_trajectory.joint_trajectory.points[i-1].time_from_start)
        {
            new_trajectory.joint_trajectory.points[i].time_from_start = new_trajectory.joint_trajectory.points[i-1].time_from_start + ros::Duration(0.00001);
        }
    }

    new_trajectory.joint_trajectory.points.insert(new_trajectory.joint_trajectory.points.begin(), Traj_Point);
    new_trajectory.joint_trajectory.points[new_trajectory.joint_trajectory.points.size()-1].time_from_start += T_offset;

    return new_trajectory;
}

void FromVector2PoseArray(TrajectoryVector waypoints, geometry_msgs::PoseArray &waypoints_final)
{

    for(size_t i=0; i< waypoints.SecondaryTrajectory.size(); i++)
    {
        for (size_t k=0; k< waypoints.SecondaryTrajectory[i].poses.size(); k++)
        {
            waypoints_final.poses.push_back(waypoints.SecondaryTrajectory[i].poses[k]);
        }
    }

}

float ComputeDistance(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2)
{
    float distance;

    distance = sqrt( 
                        pow(Point2.position.x - Point1.position.x, 2) + 
                        pow(Point2.position.y - Point1.position.y, 2) +
                        pow(Point2.position.z - Point1.position.z, 2)) ;

    return distance;
}

float ComputePathLength(geometry_msgs::PoseArray waypoint)
{
    float distance;
    distance = 0.0;

    for(int i=1; i<waypoint.poses.size(); i++)
    {
        distance += ComputeDistance(waypoint.poses[i-1], waypoint.poses[i]);
    }

    return distance;
}

moveit_msgs::RobotTrajectory VelocityScaling(moveit_msgs::RobotTrajectory trajectory_cartesian, geometry_msgs::PoseArray waypoint, float mean_velocity, ros::Duration T_offset)
{
    /*  Functon Input:
    
        trajectory_cartesina = Trajectory to rescale
        waypoint = punti nello spazion cartesiano, da cui calcoare la distanza per avere la T duration totale
        mean_velocity = velocità richiesta
    */


    float distance;
    
    moveit_msgs::RobotTrajectory new_trajectory;
    new_trajectory = trajectory_cartesian;
    distance = ComputePathLength(waypoint);
    ros::Duration overall_time = ros::Duration(distance/mean_velocity);

    float scaling_factor = overall_time.toSec()/(new_trajectory.joint_trajectory.points[(new_trajectory.joint_trajectory.points.size()-1)].time_from_start.toSec());
    new_trajectory = FactorScaling(new_trajectory, scaling_factor, T_offset);

    return new_trajectory;
}

moveit_msgs::RobotTrajectory ValidityCheck(moveit_msgs::RobotTrajectory trajectory_cartesian, ros::Duration T_offset)
{
    moveit_msgs::RobotTrajectory new_trajectory;
    trajectory_msgs::JointTrajectoryPoint Traj_Point;

    for(size_t i=0; i<trajectory_cartesian.joint_trajectory.points.size(); i++)
    {
        new_trajectory.joint_trajectory.points.push_back(trajectory_cartesian.joint_trajectory.points[i]);
    }
    new_trajectory.joint_trajectory.joint_names.resize(7);
    new_trajectory.joint_trajectory.joint_names = trajectory_cartesian.joint_trajectory.joint_names;

    Traj_Point.time_from_start = ros::Duration(0);
    Traj_Point.positions.resize(7);
    Traj_Point.velocities.resize(7);
    Traj_Point.accelerations.resize(7);

    for(int i=0; i<7; i++)
    {
        Traj_Point.positions[i] = new_trajectory.joint_trajectory.points[0].positions[i];
        Traj_Point.velocities[i] = 0;
        Traj_Point.accelerations[i] = 0;
    }

    for (size_t i=0; i<new_trajectory.joint_trajectory.points.size(); i++)
    {
        new_trajectory.joint_trajectory.points[i].time_from_start += T_offset;
    }

    for(size_t i=1; i<new_trajectory.joint_trajectory.points.size(); i++)
    {
        if(new_trajectory.joint_trajectory.points[i].time_from_start <= new_trajectory.joint_trajectory.points[i-1].time_from_start)
        {
            new_trajectory.joint_trajectory.points[i].time_from_start = new_trajectory.joint_trajectory.points[i-1].time_from_start + ros::Duration(0.00001);
        }
    }

    new_trajectory.joint_trajectory.points.insert(new_trajectory.joint_trajectory.points.begin(), Traj_Point);
    new_trajectory.joint_trajectory.points[new_trajectory.joint_trajectory.points.size()-1].time_from_start += T_offset;

    return new_trajectory;
}
