#ifndef SHELF_SCAN_H
#define SHELF_SCAN_H

#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>


#include "moveit_lib/move_robot_tf.h"
#include "moveit_lib/move_robot_pose.h"
#include "moveit_lib/move_robot_pose_array.h"
#include "moveit_lib/move_robot_named.h"

#include <string>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

//use this define for enabling debug information with move_group planner and execution
// #define DEBUG

class shelf_scan
{
public:
  //Class Constructor
  shelf_scan(std::string move_group, geometry_msgs::PoseStamped initial_pose);

  void init();
  void reinit(std::string move_group, geometry_msgs::PoseStamped initial_pose);
  bool scanShelf(Eigen::Vector4d start_q_eigen, Eigen::Vector3d delta_xyz, Eigen::Vector3d scan_offset);
  // void createConeTrajectory(geometry_msgs::Pose initial_pose, float radius, float distance, int segments, geometry_msgs::PoseArray &waypoints);
  void generatePath();
  void execute();

  ros::NodeHandle nh_;
  geometry_msgs::PoseStamped shelf_pose;
  ros::Publisher vis_pub;
  ros::Publisher reset_pub;
  ros::Publisher pause_pub;
  ros::Publisher unpause_pub;

  moveit_lib::move_robot_pose pose_srv;
  moveit_lib::move_robot_pose_array pose_array_srv;
  std_msgs::Empty empty_msg;

  ros::ServiceClient pose_client;
  ros::ServiceClient pose_array_client;
  Eigen::Vector4d start_q;
  Eigen::Vector3d delta_xyz,scan_offset;

  geometry_msgs::PoseArray waypoints;
  float radius;
  float distance;
  int segments;

};

#endif // SHELF_SCAN_H
