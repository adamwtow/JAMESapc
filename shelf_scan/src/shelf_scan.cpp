#include "shelf_scan.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

shelf_scanner::shelf_scanner(std::string move_group, geometry_msgs::PoseStamped initial_pose)
{
  // save shelf initial pose
  shelf_pose = initial_pose;

  // set move_group on moveit_lib messages
  pose_srv.request.move_group.data = move_group;
  pose_array_srv.request.move_group.data = move_group;

  // initialise various
  init();
}

void shelf_scanner::init(){

  //set up moveit_lib service clients
  pose_client = nh_.serviceClient<moveit_lib::move_robot_pose>("moveit_lib/move_robot_pose");
  pose_array_client = nh_.serviceClient<moveit_lib::move_robot_pose_array>("moveit_lib/move_robot_pose_array");

  // setup publisher for scan waypoints, mainly for debugging
  vis_pub = nh_.advertise<geometry_msgs::PoseArray>( "/shelf_scan/waypoints", 0 );

  //setup kinfu advertisers
  reset_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/reset",10);
  pause_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/pause",10);
  unpause_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/unpause",10);

  kinfu_pub = nh_.advertise<sensor_msgs::PointCloud2>("/shelf_scan/shelf_cloud",10);
  kinfu_sub = nh_.subscribe("/ros_kinfu/depth_registered/points",  1, &shelf_scanner::kinfu_callback, this);

  // temporary variables
  radius = 0.10;
  distance = 0.5;
  segments = 36;

  scanning = true;

}

// currently not used, unlikely to be
void shelf_scanner::reinit(std::string move_group, geometry_msgs::PoseStamped initial_pose){

  shelf_pose = initial_pose;
  pose_srv.request.move_group.data = move_group;
  pose_array_srv.request.move_group.data = move_group;

}

// generate desired path, expand later for various different path options
void shelf_scanner::generatePath(){

  createConePath(radius, distance, segments);

}

void shelf_scanner::createConePath(float radius, float distance, int segments) {

  tf::TransformListener tf_listener;

  float arc_theta = 2*M_PI / segments;
  float current_theta = 0.0;

  geometry_msgs::Pose pose;
  waypoints.header.frame_id = shelf_pose.header.frame_id;
  waypoints.header.stamp = ros::Time::now();

  while(current_theta < 2*M_PI){

    pose = shelf_pose.pose;

    float dy = radius*sin(current_theta);
    float dz = radius*cos(current_theta);

    pose.position.y += dy;
    pose.position.z += dz;

    float theta_z = atan2(dz,distance);
    float theta_y = atan2(dy,distance);

    tf::Quaternion quat(theta_z,theta_y,0);

    tf::Quaternion quat2(M_PI/2,0,0);

    quat2 *= quat;
    tf::quaternionTFToMsg(quat2,pose.orientation);

    current_theta += arc_theta;

    // pose.orientation = shelf_pose.pose.orientation;

    waypoints.poses.push_back(pose);

  }

  waypoints.poses.push_back(shelf_pose.pose);

}

void shelf_scanner::kinfu_callback(const sensor_msgs::PointCloud2& msg) {

  if(!scanning) {
    shelf_cloud = msg;
  }

}

void shelf_scanner::publishKinfuCloud() {

  kinfu_pub.publish(shelf_cloud);
  ROS_INFO_STREAM("Scanned shelf cloud published");
  scanning = true;

}

bool shelf_scanner::execute(){

  generatePath();
  // send moveit_lib move to pose command
  pose_srv.request.target_pose = shelf_pose;
  bool success = pose_client.call(pose_srv);

  // if pose not successful, throw exception
  if(!success){ROS_INFO_STREAM(pose_srv.response.success.data);return false;}

  // unpause and reset kinfu
  unpause_pub.publish(empty_msg);
  reset_pub.publish(empty_msg);

  pose_array_srv.request.target_pose_array = waypoints;
  vis_pub.publish(waypoints);
  ROS_INFO_STREAM("Starting to scan shelf.");

  scanning = false;
  success = pose_array_client.call(pose_array_srv);
  if(!success){ROS_INFO_STREAM("Pose array failed");return false;}
  ROS_INFO("Finished scanning shelf.");

  //pause kinfu
  pause_pub.publish(empty_msg);


  //clear waypoints
  waypoints.poses.clear();
  publishKinfuCloud();
  return true;

}
