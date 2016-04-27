#ifndef SHELF_SCAN_H
#define SHELF_SCAN_H

#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>


#include "moveit_lib/move_robot_tf.h"
#include "moveit_lib/move_robot_pose.h"
#include "moveit_lib/move_robot_pose_array.h"
#include "moveit_lib/move_robot_named.h"

#include <string>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

//use this define for enabling debug information with move_group planner and execution
// #define DEBUG

class shelf_scanner
{
public:
  //Class Constructor
  shelf_scanner(std::string move_group, geometry_msgs::PoseStamped initial_pose);

  void init();
  void reinit(std::string move_group, geometry_msgs::PoseStamped initial_pose);
  void createConePath(float radius, float distance, int segments);
  void generatePath();
  void kinfu_callback(const sensor_msgs::PointCloud2& msg);
  void publishKinfuCloud();
  bool execute();

  ros::NodeHandle nh_;
  geometry_msgs::PoseStamped shelf_pose;
  ros::Publisher vis_pub;
  ros::Publisher reset_pub;
  ros::Publisher pause_pub;
  ros::Publisher unpause_pub;
  ros::Publisher kinfu_pub;
  ros::Subscriber kinfu_sub;

  sensor_msgs::PointCloud2 shelf_cloud;

  moveit_lib::move_robot_pose pose_srv;
  moveit_lib::move_robot_pose_array pose_array_srv;
  std_msgs::Empty empty_msg;

  ros::ServiceClient pose_client;
  ros::ServiceClient pose_array_client;

  geometry_msgs::PoseArray waypoints;

  float radius;
  float distance;
  int segments;
  bool scanning;

};

#endif // SHELF_SCAN_H
