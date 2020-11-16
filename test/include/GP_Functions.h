#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/Pose.h"
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


const std::string Log_Path = "/home/kevin/panda_ws/src/traj_gen/src/";


struct EulerAngles {float x, y, z;};
struct RPY {float roll, pitch, yaw;};

EulerAngles ToEulerAngles(geometry_msgs::Quaternion q);     // Conversion from the quaternion to euler angles struct
geometry_msgs::Quaternion ToQuaternion(EulerAngles e);      // Conversion from Euler Angles to Quaternion

inline float SIGN(float x);                                         // SIGN computation of the input                              
inline float NORM(float a, float b, float c, float d);              // Norm of the function
void CrossProduct(double A[3], double B[3], double C[3]);

geometry_msgs::Quaternion mRot2Quat(float m[3][3]);                 // Get the quaternion associated to the rotation matrix
void getRX(float m[3][3], float degree);                            // get the rotazion matrix of a rotation around x of degree
void getRY(float m[3][3], float degree);
void getRZ(float m[3][3], float degree);
void MatrixMul(float m1[3][3], float m2[3][3], float result[3][3]);
void MatrixMul4(double m1[4][4], double m2[4][4], double result[4][4]); // Matrix multiplication of 2 
RPY Quat2RPY(geometry_msgs::Quaternion q);


geometry_msgs::Quaternion slerp( geometry_msgs::Quaternion v0, geometry_msgs::Quaternion v1, double t);

float ComputeDistance(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2);

int SaveLog( std::string file_name_plan, moveit::planning_interface::MoveGroupInterface::Plan Plan,  std::string file_name_waypoints, std::vector<geometry_msgs::Pose> waypoints_final);


EulerAngles ToEulerAngles(geometry_msgs::Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
geometry_msgs::Quaternion ToQuaternion(EulerAngles e)
{
    // Abbreviations for the various angular functions
    float cy = cos(e.x * 0.5);
    float sy = sin(e.x * 0.5);
    float cp = cos(e.y * 0.5);
    float sp = sin(e.y * 0.5);
    float cr = cos(e.z * 0.5);
    float sr = sin(e.z * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

geometry_msgs::Quaternion mRot2Quat(float m[3][3])
{
    geometry_msgs::Quaternion res;

	float r11 = m[0][0];
	float r12 = m[0][1];
	float r13 = m[0][2];
	float r21 = m[1][0];
	float r22 = m[1][1];
	float r23 = m[1][2];
	float r31 = m[2][0];
	float r32 = m[2][1];
	float r33 = m[2][2];
	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) 
    {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) 
    {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) 
    {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) 
    {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}

	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

    res.w = q0;
    res.x = q1;
    res.y = q2;
    res.z = q3;

    // Mat res = q0 q1, q2, q3);
	return res;
}
void getRX(float m[3][3], float degree)
{

    m[0][0] = 1;
	m[0][1] = 0;
	m[0][2] = 0;
	m[1][0] = 0;
	m[1][1] = cos(degree);
	m[1][2] = -sin(degree);
	m[2][0] = 0;
	m[2][1] = sin(degree);
	m[2][2] = cos(degree);
    
    return;
}
void getRY(float m[3][3], float degree)
{
    
    m[0][0] = cos(degree);
	m[0][1] = 0;
	m[0][2] = sin(degree);
	m[1][0] = 0;
	m[1][1] = 1;
	m[1][2] = 0;
	m[2][0] = -sin(degree);
	m[2][1] = 0;
	m[2][2] = cos(degree);

    return;
}
void getRZ(float m[3][3], float degree)
{

    m[0][0] = cos(degree);
	m[0][1] = -sin(degree);
	m[0][2] = 0;
	m[1][0] = sin(degree);
	m[1][1] = cos(degree);
	m[1][2] = 0;
	m[2][0] = 0;
	m[2][1] = 0;
	m[2][2] = 1;

    return;
}
void MatrixMul(float m1[3][3], float m2[3][3], float result[3][3])
{
    
    float flag[3][3];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            flag[i][j] = 0;
            for (int k = 0; k < 3; k++)
            {
                flag[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = flag[i][j];
        }
    }
    return;
}

void MatrixMul4(double m1[4][4], double m2[4][4], double result[4][4])
{
    
    float flag[4][4];

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            flag[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                flag[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result[i][j] = flag[i][j];
        }
    }
    return;
}

RPY Quat2RPY(geometry_msgs::Quaternion q){

    RPY angles;

 /*    
    angles.roll  = atan2(2*q.y*q.w + 2*q.x*q.z , 1 - 2*q.y*q.y - 2*q.z*q.z);
    angles.pitch = atan2(2*q.x*q.w + 2*q.y*q.z, 1 - 2*q.x*q.x - 2*q.z*q.z);
    angles.yaw   = asin(2*q.x*q.y + 2*q.z*q.w);
 */
    
    angles.roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    angles.pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
    angles.yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);

    return angles;
}

inline float SIGN(float x)
{ 
	return (x >= 0.0f) ? +1.0f : -1.0f; 
}
inline float NORM(float a, float b, float c, float d)
{ 
	return sqrt(a * a + b * b + c * c + d * d); 
}

void CrossProduct(double A[3], double B[3], double C[3])
{
    C[0] = A[1]*B[2] - A[2]*B[1];
    C[1] = -(A[0]*B[2] - A[2]*B[0]);
    C[2] = A[0]*B[1] - A[1]*B[0];
    return ;
}

geometry_msgs::Quaternion slerp( geometry_msgs::Quaternion v0, geometry_msgs::Quaternion v1, double t)
{

    /*
        REPLACED BY SLERP FUNCTION OF TF2 LIBRARY!!
    */
   
    // Only unit quaternions are valid rotations.
    // Normalize to avoid undefined behavior.

    geometry_msgs::Quaternion result;
 /*
    v0.normalize();
    v1.normalize();
 */
    // Compute the cosine of the angle between the two vectors.
    double dot = v0.w*v1.w + v0.x*v1.x + v0.y*v1.y + v0.z*v1.z;

    // If the dot product is negative, slerp won't take
    // the shorter path. Note that v1 and -v1 are equivalent when
    // the negation is applied to all four components. Fix by 
    // reversing one quaternion.
    if (dot < 0.0f)
    {
        v1.w = -v1.w;
        v1.x = -v1.x;
        v1.y = -v1.y;
        v1.z = v1.z;
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // If the inputs are too close for comfort, linearly interpolate
        // and normalize the result.

        result.x = v0.x + t*(v1.x - v0.x);
        result.y = v0.y + t*(v1.y - v0.y);
        result.z = v0.z + t*(v1.z - v0.z);
        result.w = v0.w + t*(v1.w - v0.w);
        dot = result.w*result.w + result.x*result.x + result.y*result.y + result.z*result.z;
        dot = sqrt(dot);

        result.x = result.x/dot;
        result.y = result.y/dot;
        result.z = result.z/dot;
        result.w = result.w/dot;
   
        return result;
    }

    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
    double theta_0 = acos(dot);        // theta_0 = angle between input vectors
    double theta = theta_0*t;          // theta = angle between v0 and result
    double sin_theta = sin(theta);     // compute this value only once
    double sin_theta_0 = sin(theta_0); // compute this value only once

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
    double s1 = sin_theta / sin_theta_0;

    result.x = (s0 * v0.x) + (s1 * v1.x);
    result.y = (s0 * v0.y) + (s1 * v1.y);
    result.z = (s0 * v0.z) + (s1 * v1.z);
    result.w = (s0 * v0.w) + (s1 * v1.w);
    
    return result;
}

/*
float ComputeDistance(geometry_msgs::Pose Point1, geometry_msgs::Pose Point2)
{
    float distance;

    distance = sqrt( 
                        pow(Point2.position.x - Point1.position.x, 2) + 
                        pow(Point2.position.y - Point1.position.y, 2) +
                        pow(Point2.position.z - Point1.position.z, 2)) ;

    return distance;
}

*/

/*
int SaveLog( std::string file_name_plan, moveit::planning_interface::MoveGroupInterface::Plan Plan,  std::string file_name_waypoints, std::vector<geometry_msgs::Pose> waypoints_final)
{
    std::ofstream plan_output_c;
    plan_output_c.open(Log_Path + file_name_plan);
     
    for (size_t i = 0; i < Plan.trajectory_.joint_trajectory.points.size(); i++)
    { 
       
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[0] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[1] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[2]<< "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[3]<< "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[4]<< "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[5]<< "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].positions[6]<< "\n";

        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[0] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[1] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[2] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[3] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[4] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[5] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].velocities[6] << "\n";

        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[0] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[1] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[2] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[3] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[4] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[5] << "\n";
        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].accelerations[6] << "\n";

        plan_output_c << Plan.trajectory_.joint_trajectory.points[i].time_from_start << "\n";                
    }

    plan_output_c.close();

    std::ofstream waypoints_output_c;
    waypoints_output_c.open(Log_Path + file_name_waypoints);

    for (size_t i = 0; i < waypoints_final.size(); i++)
    { 
        waypoints_output_c << waypoints_final[i] << "\n";
    }
    waypoints_output_c.close();

    return 1;

}
*/


