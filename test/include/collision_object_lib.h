

#include "ros/ros.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "Rack";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.7;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.6;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.08;

  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

std::string addRobotBase(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::CollisionObject robot_base;

    robot_base.id = "Robot_Base";
    robot_base.header.frame_id = "panda_link0";

    robot_base.primitives.resize(1);
    robot_base.primitives[0].type = robot_base.primitives[0].BOX;
    robot_base.primitives[0].dimensions.resize(3);
    robot_base.primitives[0].dimensions[0] = 1;
    robot_base.primitives[0].dimensions[1] = 1;
    robot_base.primitives[0].dimensions[2] = 0.1;

    robot_base.primitive_poses.resize(1);
    robot_base.primitive_poses[0].position.x = 0;
    robot_base.primitive_poses[0].position.y = 0;
    robot_base.primitive_poses[0].position.z = -0.05;

    robot_base.operation = robot_base.ADD;
    planning_scene_interface.applyCollisionObject(robot_base);

    return robot_base.id;

}

