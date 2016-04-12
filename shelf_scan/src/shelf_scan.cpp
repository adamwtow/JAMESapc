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

  ros::Publisher points_pub;

  // temporary variables
  start_q = {0.0,0.707,0.0,0.707};
  delta_xyz = {0.10, 0.10, 0.10};
  scan_offset = {0.35, 0.0, 0.08};
  radius = 0.10;
  distance = 0.5;
  segments = 36;

}

// currently not used, unlikely to be
void shelf_scanner::reinit(std::string move_group, geometry_msgs::PoseStamped initial_pose){

  shelf_pose = initial_pose;
  pose_srv.request.move_group.data = move_group;
  pose_array_srv.request.move_group.data = move_group;

}

// generate desired path, expand later for various different path options
void shelf_scanner::generatePath(){

  // createSimplePath(start_q, delta_xyz,scan_offset);
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

    float theta_y = atan2(dy,distance);
    float theta_z = atan2(dz,distance);

    pose.position.y += dy;
    pose.position.z += dz;

    tf::Quaternion quat(theta_z,theta_y,0);

    tf::Quaternion quat2(M_PI/2,0,0);

    quat2 *= quat;
    tf::quaternionTFToMsg(quat2,pose.orientation);

    current_theta += arc_theta;

    // pose.orientation = quat;

    waypoints.poses.push_back(pose);
  }
  waypoints.poses.push_back(shelf_pose.pose);

}


//temporary waypoint generator
bool shelf_scanner::createSimplePath(Eigen::Vector4d start_q_eigen, Eigen::Vector3d delta_xyz, Eigen::Vector3d scan_offset){

  Eigen::Vector3d scan_xyz;

  geometry_msgs::Quaternion start_q, q_left,q_right,q_up,q_down;

  geometry_msgs::Pose scan_pose;

  double angle = 8*M_PI/180;

  start_q.x = start_q_eigen[0]; start_q.y = start_q_eigen[1]; start_q.z = start_q_eigen[2]; start_q.w = start_q_eigen[3];

  q_left.x = cos(-angle); q_left.y = sin(-angle); q_left.z = 0.0; q_left.w = 0.0;
  q_up.x = cos(-angle); q_up.y = 0.0; q_up.z = sin(-angle); q_up.w = 0.0;
  q_right.x = cos(angle); q_right.y = sin(angle); q_right.z = 0.0; q_right.w = 0.0;
  q_down.x = cos(angle); q_down.y = 0.0; q_down.z = sin(angle); q_down.w = 0.0;

  scan_pose = shelf_pose.pose;

  waypoints.header.stamp = ros::Time::now();
  waypoints.header.frame_id = "base";
  //Waypoint scan left
  scan_pose.position.y += delta_xyz[1];
  // scan_pose.orientation = q_left;
  waypoints.poses.push_back(scan_pose);

  //Waypoint scan up
  scan_pose.position.y -= delta_xyz[1];
  scan_pose.position.z += delta_xyz[2];
  // scan_pose.orientation = q_up;
  waypoints.poses.push_back(scan_pose);

  //Waypoint scan right
  scan_pose.position.z -= delta_xyz[2];
  scan_pose.position.y -= delta_xyz[1];
  // scan_pose.orientation = q_right;
  waypoints.poses.push_back(scan_pose);

  // scan down
  scan_pose.position.y += delta_xyz[1];
  scan_pose.position.z -= delta_xyz[2];
  // scan_pose.orientation = q_down;
  waypoints.poses.push_back(scan_pose);


  //Waypoint back to start;
  scan_pose.position.z += delta_xyz[2];
  // scan_pose.orientation = start_q;
  waypoints.poses.push_back(scan_pose);

  return true;

}

void shelf_scanner::execute(){

  // send moveit_lib move to pose command
  pose_srv.request.target_pose = shelf_pose;
  bool success = pose_client.call(pose_srv);

  // if pose not successful, throw exception
  if(!success){ROS_INFO_STREAM(pose_srv.response.success.data);throw;}

  // unpause and reset kinfu
  unpause_pub.publish(empty_msg);
  reset_pub.publish(empty_msg);

  pose_array_srv.request.target_pose_array = waypoints;
  vis_pub.publish(waypoints);
  ROS_INFO_STREAM("Starting to scan shelf.");

  success = pose_array_client.call(pose_array_srv);

  if(!success){ROS_INFO_STREAM("Pose array failed");throw;}

  ROS_INFO("Finished scanning shelf.");

  //pause kinfu
  pause_pub.publish(empty_msg);

  //clear waypoints
  waypoints.poses.clear();

}
