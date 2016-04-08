/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/PoseArray.h>

#include <baxter_core_msgs/EndEffectorCommand.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "moveit_lib/move_robot_pose.h"
#include "moveit_lib/move_robot_pose_array.h"
#include "moveit_lib/move_robot_named.h"
#include "build_kinfu/shelf_scan.h"

// #include <shelf_registration.hpp>

#include <string>
#include <vector>


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";

static const double MSG_PULSE_SEC = 0.1;
static const double WAIT_GRIPPER_CLOSE_SEC = 0.1;

// max time to wait for the gripper state to refresh
static const double WAIT_STATE_MSG_SEC = 1;
// Num times to re-send a msg to the end effector for assurance that it arrives
static const double GRIPPER_MSG_RESEND = 10;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
sensor_msgs::Image,
sensor_msgs::CameraInfo,
sensor_msgs::CameraInfo> SyncPolicy;


class shelfScan {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor,
  *subCameraInfoDepth;
  message_filters::Synchronizer<SyncPolicy> *sync;
  ros::Subscriber pointsSub;
  sensor_msgs::PointCloud2 points;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud;
  double velocity_scan;
  Eigen::Vector4d start_q;
  Eigen::Vector3d delta_xyz,scan_offset;

  float object_z_in_kinect_frame = 0.0;
  float object_x_in_kinect_frame = 0.0;
  float object_y_in_kinect_frame = 0.0;

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  tf::TransformListener tf_listener;

  bool pick_fruit;
  bool found_fruit;
  int  pick_fruit_index;
  bool create_lookup;
  std::vector<cv::KeyPoint> fruit_keypoints;

  cv::Mat color;
  cv::Mat depth;

  cv::Mat cameraMatrixColor;
  cv::Mat cameraMatrixDepth;

  bool reset;

  cv::Mat lookupX, lookupY;

  geometry_msgs::PoseStamped object_pose;
  geometry_msgs::PoseStamped start_pose,  arm_pose;

  ros::Publisher vis_pub;
  ros::Publisher reset_pub;
  ros::Publisher pause_pub;

  ros::ServiceServer shelf_scan_service = nh_.advertiseService("/shelf_scan", &scan_self);

  // moveit::planning_interface::MoveGroup::Plan left_plan;
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // moveit::planning_interface::MoveGroup left_arm;
  double wait_time,eef_step_size, jump_threshold;

  ros::ServiceClient pose_client;
  ros::ServiceClient pose_array_client;
  ros::ServiceClient named_client;

  moveit_lib::move_robot_pose pose_srv;
  moveit_lib::move_robot_pose_array pose_array_srv;
  moveit_lib::move_robot_named named_srv;

      std_msgs::Empty reset_msg;


public:
  shelfScan():it_(nh_){ //, left_arm("left_arm") {

    // cv::namedWindow(OPENCV_WINDOW);


    // std::string topicColor = "/kinect2/qhd/image_color";
    // std::string topicDepth = "/kinect2/qhd/image_depth_rect";
    // std::string topicCameraInfoColor = "/kinect2/qhd/camera_info";
    // std::string topicCameraInfoDepth = "/kinect2/qhd/camera_info";


    nh_.param("start_q1",start_q[0],0.0);
    nh_.param("start_q2",start_q[1],0.707);
    nh_.param("start_q3",start_q[2],0.0);
    nh_.param("start_q4",start_q[3],0.707);

    nh_.param("scan_delta_x", delta_xyz[0], 0.10);
    nh_.param("scan_delta_y", delta_xyz[1], 0.10);
    nh_.param("scan_delta_z", delta_xyz[2], 0.10);

    nh_.param("scan_offset_x",scan_offset[0], 0.35);
    nh_.param("scan_offset_y",scan_offset[1], 0.0);
    nh_.param("scan_offset_z",scan_offset[2], 0.08);
    start_q_eigen
    delta_xyz
    scan_offset
    nh_.param("wait_time",wait_time,0.0);

    nh_.param("velocity_scan", velocity_scan, 0.5);
    nh_.param("eef_step_size", eef_step_size,0.01);
    nh_.param("jump_threshold", jump_threshold,0.0);

    // pointsSub = nh_.subscribe("/kinect2/qhd/points", 10, &fruitpicker::pointsCallback, this);
    // subImageColor = new image_transport::SubscriberFilter(it_, topicColor, 4);
    // subImageDepth = new image_transport::SubscriberFilter(it_, topicDepth, 4);
    // subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfoColor, 4);
    // subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfoDepth, 4);

    pose_client = nh_.serviceClient<moveit_lib::move_robot_pose>("moveit_lib/move_robot_pose");
    pose_array_client = nh_.serviceClient<moveit_lib::move_robot_pose_array>("moveit_lib/move_robot_pose_array");
    named_client = nh_.serviceClient<moveit_lib::move_robot_named>("moveit_lib/move_robot_named");

    vis_pub = nh_.advertise<geometry_msgs::PoseArray>( "scan_waypoints", 0 );

    reset_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/reset",10);
    pause_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/pause",10);

    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);

    pick_fruit = false;
    pick_fruit_index = 0;
    found_fruit = false;
    create_lookup = true;

    // sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(4), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
    // sync->registerCallback(boost::bind(&fruitpicker::callback, this, _1, _2, _3, _4));

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  }

  ~fruitpicker() {
    // cv::destroyWindow(OPENCV_WINDOW);

    // std::vector<std::string> object_names = planning_scene_interface.getKnownObjectNames();
    // planning_scene_interface.removeCollisionObjects(object_names);
  }

  void start() {
    float wait_time = 0.0;
    // cv::setMouseCallback(OPENCV_WINDOW, mouse_click, this);
    reset = false;
    ros::Time time;

    // left_arm.setPlannerId("RRTConnectkConfigDefault");
    // left_arm.setPlanningTime(5);
    // left_arm.setMaxVelocityScalingFactor(0.75);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::cout << "Moving to box" << std::endl;

    named_srv.request.move_group.data = "left_arm";
    named_srv.request.named_pose.data = "left_box_pose";

    bool success = named_client.call(named_srv);

    pick_fruit = true;

    while (ros::ok()) {
      if (reset) {

        named_srv.request.move_group.data = "left_arm";
        named_srv.request.named_pose.data = "left_box_pose";
        std::cout << "Reset flagged moving to box" << std::endl;
        bool success = named_client.call(named_srv);

        pick_fruit = false;
        reset = false;
      }

      if (pick_fruit) {
        pick_fruit = false;
        //
        // // arm should reach 5 cm to the left of where it is ending up
        // object_pose.pose.position.z = object_z_in_kinect_frame;
        // // arm should reach 2 cm upwards of where it is ending up
        // object_pose.pose.position.x = object_x_in_kinect_frame;
        // // arm should reach 8 cm further outwards than it is ending up
        // object_pose.pose.position.y = object_y_in_kinect_frame;
        //
        // object_pose.pose.orientation.x = 0.707;
        // object_pose.pose.orientation.y = 0;
        // object_pose.pose.orientation.z = 0.707;
        // object_pose.pose.orientation.w = 0;
        // object_pose.header.frame_id = ("/kinect2_rgb_optical_frame");
        // object_pose.header.stamp = ros::Time::now();

        double init_x_offset = 0.2;
        double gripper_x_offset = 0.0;
        double gripper_y_offset = 0.0;
        double gripper_z_offset = 0.0;

        try {
          // tf_listener.waitForTransform(
          //   "/torso", "/kinect2_rgb_optical_frame",
          //   ros::Time::now(), ros::Duration(10));
          //   tf_listener.transformPose("/torso", object_pose, arm_pose);

          /*
          Add offset to account for the gripper not being in the
          MoveIt model of Baxter.
          */
          // position in front of object
          // arm_pose.pose.position.x -= init_x_offset - gripper_x_offset;
          // arm_pose.pose.position.z -= gripper_z_offset;
          // arm_pose.pose.position.y -= gripper_y_offset;
          arm_pose.pose.position.x = 0.95;
          arm_pose.pose.position.y = 0.05;
          arm_pose.pose.position.z = 0.32;

          arm_pose.pose.orientation.x = 0;
          arm_pose.pose.orientation.y = 0.707;
          arm_pose.pose.orientation.z = 0;
          arm_pose.pose.orientation.w = 0.707;

          arm_pose.header.frame_id = "/base";

          // arm_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2, M_PI/2);

          // Move to init_x_offset from the object
          // if (!moveTo(arm_pose, 2, wait_time)) {throw;}
          pose_srv.request.move_group.data = "left_arm";
          // pose_srv.request.target_frame_id.data = arm_pose.header.frame_id;
          pose_srv.request.target_pose = arm_pose;

          ROS_INFO_STREAM(arm_pose);

          pose_client.call(pose_srv);

          if(!pose_srv.response.success.data){ROS_INFO_STREAM(pose_srv.response.success.data);reset=true;continue;}

          if(scanShelf(arm_pose, start_q, delta_xyz, scan_offset)){reset=true;continue;}

        }
        catch (tf::TransformException ex) {

          reset = true;
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();

        }
      }
    }
  }  // End Start

  bool scanShelf(geometry_msgs::PoseStamped start_pose, Eigen::Vector4d start_q_eigen, Eigen::Vector3d delta_xyz, Eigen::Vector3d scan_offset){

    Eigen::Vector3d scan_xyz;

    geometry_msgs::Quaternion start_q, q_left,q_right,q_up,q_down;
    geometry_msgs::PoseArray waypoints;

    geometry_msgs::Pose scan_pose;
    // geometry_msgs::PoseStamped temp_pose;


    double angle = 8*M_PI/180;
    //angle = atan(delta_xyz[1]/scan_offset[0])/2;

    start_q.x = start_q_eigen[0]; start_q.y = start_q_eigen[1]; start_q.z = start_q_eigen[2]; start_q.w = start_q_eigen[3];

    q_left.x = cos(-angle); q_left.y = sin(-angle); q_left.z = 0.0; q_left.w = 0.0;
    q_up.x = cos(-angle); q_up.y = 0.0; q_up.z = sin(-angle); q_up.w = 0.0;
    q_right.x = cos(angle); q_right.y = sin(angle); q_right.z = 0.0; q_right.w = 0.0;
    q_down.x = cos(angle); q_down.y = 0.0; q_down.z = sin(angle); q_down.w = 0.0;

    // bool tf_success = tf_listener.waitForTransform("base", "left_hand",  ros::Time(0), ros::Duration(1));
    //
    // if (tf_success)   {
    //
    //   tf_listener.lookupTransform("base", "left_hand", ros::Time(0), temp_pose);
    //
    // }
    //       geometry_msgs::PoseStamped target_pose;
    //       Eigen::Affine3d target_pose_eigen;
    //
    //       tf::transformTFToEigen(target_transform, target_pose_eigen);
    //
    //       tf::poseEigenToMsg(target_pose_eigen,target_pose.pose);

    scan_pose = start_pose.pose; //not the best method, would prefer to use left_arm.getCurrentPose()
    ROS_INFO_STREAM(scan_pose);

    ROS_INFO("Starting to scan shelf.");

    reset_pub.publish(reset_msg);
    //Build up multiple views of capsicum
    // left_arm.setMaxVelocityScalingFactor(velocity_scan);

    //Start waypoint
    // waypoints.push_back(scan_pose);
    // pose_array.poses.push_back(scan_pose);
    // waypoints.header = left_arm.getCurrentPose().header;
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
    // createConeTrajectory(scan_pose,0.10,0.2,36,waypoints);
    vis_pub.publish(waypoints);

    pose_array_srv.request.move_group.data = "left_arm";
    pose_array_srv.request.target_pose_array = waypoints;
    ROS_INFO_STREAM("Waypoints generated");
    bool success = pose_array_client.call(pose_array_srv);
    if(!success){ROS_INFO_STREAM("Pose array failed");return false;}

    ROS_INFO("Finished scanning shelf.");
    pause_pub.publish(reset_msg);
    return true;

  }

  // void createConeTrajectory(geometry_msgs::Pose initial_pose, float radius, float distance, int segments, geometry_msgs::PoseArray &waypoints) {
  //
  //   float arc_theta = 2*M_PI / segments;
  //   float current_theta = 0.0;
  //   geometry_msgs::Pose pose;
  //
  //   while(current_theta < 2*M_PI){
  //
  //     pose = initial_pose;
  //
  //     float dy = radius*sin(current_theta);
  //     float dz = radius*cos(current_theta);
  //
  //     float theta_y = atan2(dy,distance);
  //     float theta_z = atan2(dz,distance);
  //
  //     pose.position.y += dy;
  //     pose.position.z += dz;
  //
  //
  //     tf::Quaternion quat(0,theta_y,theta_z);
  //     // double roll, pitch, yaw;
  //     // tf::Matrix3x3(pose.orientation).getRPY(roll, pitch, yaw);
  //
  //     // roll +=
  //     tf::Quaternion currentQuat;
  //     tf::quaternionMsgToTF(pose.orientation,currentQuat);
  //     tf::quaternionTFToMsg(currentQuat.dot(quat),pose.orientation);
  //
  //     current_theta += arc_theta;
  //
  //     waypoints.poses.push_back(pose);
  //   }
  //
  // }

  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::fromROSMsg(*msg, *cloud);
    // points = *msg;
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor,
    const sensor_msgs::Image::ConstPtr imageDepth,
    const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
    const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
      cv::Mat highlighted;
      readImage(imageColor, color);
      cv::imshow(OPENCV_WINDOW, color);
      cv::waitKey(3);
    }


    void readImage(const sensor_msgs::Image::ConstPtr msgImage,
      cv::Mat &image) {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
      }

      void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo,
        cv::Mat &cameraMatrix) {
          double *itC = cameraMatrix.ptr<double>(0, 0);

          for (size_t i = 0; i < 9; ++i, ++itC) {
            *itC = cameraInfo->K[i];
          }
        }

        void mouse_click(int event, int x, int y) {
          switch (event) {
            case CV_EVENT_LBUTTONDOWN:
            {
              int width = cloud->width;

              int bounds = 2;
              float length = 0.0;

              float z_sum = 0.0;
              float x_sum = 0.0;
              float y_sum = 0.0;

              for (int i = x-bounds; i <= x+bounds; i++) {
                for (int j = y-bounds; j <= y+bounds; j++) {
                  if (!isnan(cloud->points[i+j*width].z)) {
                    // std::cout << cloud->points[i+j*width].z << ", ";
                    // Append depths with each valid depth value
                    z_sum += (cloud->points[i+j*width].z);
                    x_sum += (cloud->points[i+j*width].x);
                    y_sum += (cloud->points[i+j*width].y);
                    length = length + 1.0;
                  }
                }
              }

              object_z_in_kinect_frame = z_sum/length;
              object_x_in_kinect_frame = x_sum/length;
              object_y_in_kinect_frame = y_sum/length;

              length = 0;

              pick_fruit = true;
              pick_fruit_index = 0;

              break;
            }
          }
        }

        static void mouse_click(int event, int x, int y, int, void* this_) {
          static_cast<fruitpicker*>(this_)->mouse_click(event, x, y);
        }
      };


      int main(int argc, char** argv) {

        ros::init(argc, argv, "shelfScan");

        shelfScan ss;

        ss.start();

        return 0;
      }
