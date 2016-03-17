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


class fruitpicker {
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
  geometry_msgs::PoseStamped arm_pose;
  std::vector<double> start_joint_values =
  {0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};

  ros::Publisher vis_pub;
  ros::Publisher command_topic_;

  moveit::planning_interface::MoveGroup::Plan left_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup left_arm;
  double wait_time,eef_step_size, jump_threshold;



public:
  fruitpicker():it_(nh_), left_arm("left_arm") {
    ROS_INFO_STREAM("WTF?");
    cv::namedWindow(OPENCV_WINDOW);


    std::string topicColor = "/kinect2/qhd/image_color";
    std::string topicDepth = "/kinect2/qhd/image_depth_rect";
    std::string topicCameraInfoColor = "/kinect2/qhd/camera_info";
    std::string topicCameraInfoDepth = "/kinect2/qhd/camera_info";


    nh_.param("start_q1",start_q[0],0.707);
    nh_.param("start_q2",start_q[1],0.0);
    nh_.param("start_q3",start_q[2],0.707);
    nh_.param("start_q4",start_q[3],0.0);

    nh_.param("scan_delta_x", delta_xyz[0], 0.05);
    nh_.param("scan_delta_y", delta_xyz[1], 0.05);
    nh_.param("scan_delta_z", delta_xyz[2], 0.05);

    nh_.param("scan_offset_x",scan_offset[0], 0.35);
    nh_.param("scan_offset_y",scan_offset[1], 0.0);
    nh_.param("scan_offset_z",scan_offset[2], 0.08);
    nh_.param("wait_time",wait_time,0.0);
    nh_.param("velocity_scan", velocity_scan, 0.5);
    nh_.param("eef_step_size", eef_step_size,0.01);
    nh_.param("jump_threshold", jump_threshold,0.0);

    pointsSub = nh_.subscribe("/kinect2/qhd/points", 10,
    &fruitpicker::pointsCallback, this);
    subImageColor =
    new image_transport::SubscriberFilter(it_, topicColor, 4);
    subImageDepth =
    new image_transport::SubscriberFilter(it_, topicDepth, 4);
    subCameraInfoColor =
    new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfoColor, 4);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfoDepth, 4);

    vis_pub = nh_.advertise<visualization_msgs::Marker>("fruit_marker", 0);
    command_topic_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 10);

    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);

    pick_fruit = false;
    pick_fruit_index = 0;
    found_fruit = false;
    create_lookup = true;


    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(4), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
    sync->registerCallback(boost::bind(&fruitpicker::callback, this, _1, _2, _3, _4));

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  }

  ~fruitpicker() {
    cv::destroyWindow(OPENCV_WINDOW);

    std::vector<std::string> object_names = planning_scene_interface.getKnownObjectNames();
    planning_scene_interface.removeCollisionObjects(object_names);
  }

  void start() {
    float wait_time = 0.0;
    cv::setMouseCallback(OPENCV_WINDOW, mouse_click, this);
    reset = false;
    ros::Time time;

    left_arm.setPlannerId("RRTConnectkConfigDefault");
    left_arm.setPlanningTime(5);
    left_arm.setMaxVelocityScalingFactor(0.75);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::cout << "Moving to box" << std::endl;
    moveToNamed("left_box_pose", 2, 1.0);

    while (ros::ok()) {
      if (reset) {
        std::cout << "Reset flagged moving to box" << std::endl;
        moveToNamed("left_box_pose", 2, wait_time);
        pick_fruit = false;
      }

      if (pick_fruit) {
        pick_fruit = false;

        // arm should reach 5 cm to the left of where it is ending up
        object_pose.pose.position.z = object_z_in_kinect_frame;
        // arm should reach 2 cm upwards of where it is ending up
        object_pose.pose.position.x = object_x_in_kinect_frame;
        // arm should reach 8 cm further outwards than it is ending up
        object_pose.pose.position.y = object_y_in_kinect_frame;

        object_pose.pose.orientation.x = 0.707;
        object_pose.pose.orientation.y = 0;
        object_pose.pose.orientation.z = 0.707;
        object_pose.pose.orientation.w = 0;
        object_pose.header.frame_id = ("/kinect2_rgb_optical_frame");
        object_pose.header.stamp = ros::Time::now();

        double init_x_offset = 0.4;
        double gripper_x_offset = 0.05;
        double gripper_y_offset = 0.035;
        double gripper_z_offset = 0.01;

        try {
          tf_listener.waitForTransform(
            "/torso", "/kinect2_rgb_optical_frame",
            ros::Time::now(), ros::Duration(10));
            tf_listener.transformPose("/torso", object_pose, arm_pose);

            /*
            Add offset to account for the gripper not being in the
            MoveIt model of Baxter.
            */
            // position in front of object
            arm_pose.pose.position.x -= init_x_offset - gripper_x_offset;
            arm_pose.pose.position.z -= gripper_z_offset;
            arm_pose.pose.position.y -= gripper_y_offset;

            arm_pose.pose.orientation.x = 0;
            arm_pose.pose.orientation.y = 0.707;
            arm_pose.pose.orientation.z = 0;
            arm_pose.pose.orientation.w = 0.707;

            // arm_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2, M_PI/2);

            std::cout << "Attempting to grasp fruit at x,y,z: "
            << arm_pose.pose.position.x << ", "
            << arm_pose.pose.position.y << ", "
            << arm_pose.pose.position.z << std::endl;

            // Move to init_x_offset from the object
            if (!moveTo(arm_pose, 2, wait_time)) {throw;}

            if(scanShelf(start_q, delta_xyz, scan_offset)){throw;}
          }
          catch (tf::TransformException ex) {
            reset = true;
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
          }
        }
      }
    }  // End Start

    bool scanShelf(Eigen::Vector4d start_q_eigen, Eigen::Vector3d delta_xyz, Eigen::Vector3d scan_offset){

      Eigen::Vector3d scan_xyz;

      geometry_msgs::Quaternion start_q, q_left,q_right,q_up,q_down;
      std::vector<geometry_msgs::Pose> waypoints;

      geometry_msgs::PoseArray pose_array;
      geometry_msgs::Pose scan_pose;
      pose_array.header.frame_id = "/world";
      pose_array.header.stamp = ros::Time::now();


      double angle = 8*M_PI/180;
      //angle = atan(delta_xyz[1]/scan_offset[0])/2;

      start_q.x = start_q_eigen[0]; start_q.y = start_q_eigen[1]; start_q.z = start_q_eigen[2]; start_q.w = start_q_eigen[3];

      q_left.x = cos(-angle); q_left.y = sin(-angle); q_left.z = 0.0; q_left.w = 0.0;
      q_up.x = cos(-angle); q_up.y = 0.0; q_up.z = sin(-angle); q_up.w = 0.0;
      q_right.x = cos(angle); q_right.y = sin(angle); q_right.z = 0.0; q_right.w = 0.0;
      q_down.x = cos(angle); q_down.y = 0.0; q_down.z = sin(angle); q_down.w = 0.0;

      scan_pose = left_arm.getCurrentPose().pose;
      ROS_INFO_STREAM(left_arm.getCurrentPose().header.frame_id);
      // scan_pose.header.frame_id = "/world";
      ROS_INFO("Starting to scan shelf.");

      //Build up multiple views of capsicum
      left_arm.setMaxVelocityScalingFactor(velocity_scan);

      //Start waypoint
      // waypoints.push_back(scan_pose);
      // pose_array.poses.push_back(scan_pose);

      //Waypoint scan left
      scan_pose.position.y += delta_xyz[1];
      // scan_pose.orientation = q_left;
      waypoints.push_back(scan_pose);
      pose_array.poses.push_back(scan_pose);

      //Waypoint scan up
      scan_pose.position.y -= delta_xyz[1];
      scan_pose.position.z += delta_xyz[2];
      // scan_pose.orientation = q_up;
      waypoints.push_back(scan_pose);
      pose_array.poses.push_back(scan_pose);

      //Waypoint scan right
      scan_pose.position.z -= delta_xyz[2];
      scan_pose.position.y -= delta_xyz[1];
      // scan_pose.orientation = q_right;
      waypoints.push_back(scan_pose);
      pose_array.poses.push_back(scan_pose);

      // scan down
      scan_pose.position.y += delta_xyz[1];
      scan_pose.position.z -= delta_xyz[2];
      // scan_pose.orientation = q_down;
      waypoints.push_back(scan_pose);
      pose_array.poses.push_back(scan_pose);


      //Waypoint back to start;
      scan_pose.position.z += delta_xyz[2];
      // scan_pose.orientation = start_q;
      waypoints.push_back(scan_pose);
      pose_array.poses.push_back(scan_pose);

      vis_pub.publish(pose_array);

      if(!moveToCartesianPath(waypoints,velocity_scan, 20, wait_time, false, 0)){return false;}

      ROS_INFO("Finished scanning shelf.");

      return true;

    }


    bool moveToCartesianPath(std::vector<geometry_msgs::Pose> waypoints, double velocity_scale, int Attempts, float timeout, bool async, int digital_pin){

      moveit_msgs::RobotTrajectory trajectory_msg;       // eef_step // jump_threshold
      double fraction_complete;

      fraction_complete = left_arm.computeCartesianPath(waypoints, eef_step_size,  jump_threshold, trajectory_msg);

      // The trajectory needs to be modified so it will include velocities as well.
      // First to create a RobotTrajectory object
      robot_trajectory::RobotTrajectory rt(left_arm.getCurrentState()->getRobotModel(), "left_arm");

      // Second get a RobotTrajectory from trajectory
      rt.setRobotTrajectoryMsg(*left_arm.getCurrentState(), trajectory_msg);

      // Thrid create a IterativeParabolicTimeParameterization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;

      // Fourth compute computeTimeStamps
      bool success = iptp.computeTimeStamps(rt,velocity_scale);
      ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

      // Get RobotTrajectory_msg from RobotTrajectory
      rt.getRobotTrajectoryMsg(trajectory_msg);

      // Finally plan and execute the trajectory
      left_plan.trajectory_ = trajectory_msg;
      ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction_complete * 100.0);

      try{
        //            #ifdef DEBUG
        //            cout << "Press the ENTER key to move arm or r to RESET";
        //            unsigned char c;
        //            c << cin.get();
        //            //          while( c << cin.get()  ) {
        //            //            if(c == '\n') break;
        //            //          }
        //            #endif

        ROS_INFO("Moving");
        // if(async){
        //
        //   ros::Duration trajectory_time = trajectory_msg.joint_trajectory.points.back().time_from_start;
        //   ros::Duration duration_offset(2.0);
        //   ROS_INFO("Moving asyncronously");
        //   if(!asyncExecuteUntilDigitalIO(digital_pin, trajectory_time + duration_offset, waypoints.back())) return false;
        //
        // }else{
        ROS_INFO("Moving without async");
        left_arm.execute(left_plan);
        // }

        return true;
      }
      catch(moveit::planning_interface::MoveItErrorCode ex){
        std::cout << "Something went wrong. Failed to move to pose" << std::endl;
        return false;
      }
    }

    bool moveToNamed(std::string namedGoal, int Attempts, float timeout) {
      bool ret = false;
      bool success = false;

      while (Attempts > 0) {
        left_arm.setStartStateToCurrentState();
        left_arm.setNamedTarget(namedGoal);

        success = left_arm.plan(left_plan);

        if (success) {
          std::cout << "Successfully planned to goal, moving to goal"
          << std::endl;
          try {
            left_arm.move();
            std::cout << "Moved to Goal planning grasp" << std::endl;
            Attempts = 0;
            ret = true;
          }
          catch(moveit::planning_interface::MoveItErrorCode ex) {
            std::cout << "Something went wrong. Failed to move to Goal"
            << std::endl;
            // reset = true;
            // ROS_ERROR("%s",);
            ret = false;
          }
        } else {
          Attempts--;
          sleep(timeout);
        }
      }
      return ret;
    }

    bool moveTo(geometry_msgs::PoseStamped pose, int Attempts, float timeout) {
      bool ret = false;
      bool success = false;

      while (Attempts > 0) {
        left_arm.setStartStateToCurrentState();
        left_arm.setPoseTarget(arm_pose);

        success = left_arm.plan(left_plan);

        if (success) {
          std::cout << "Successfully planned to goal, moving to goal"
          << std::endl;
          try {
            left_arm.move();
            std::cout << "Moved to Goal planning grasp" << std::endl;
            Attempts = 0;
            ret = true;
          }
          catch (moveit::planning_interface::MoveItErrorCode ex) {
            std::cout << "Something went wrong. Failed to move to Goal"
            << std::endl;
            // reset = true;
            // ROS_ERROR("%s",ex.what());
            ret = false;
          }
        } else {
          Attempts--;
          sleep(timeout);
        }
      }
      return ret;
    }

    void addBoxObject() {
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = left_arm.getPlanningFrame();
      collision_object.id = "box";

      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      // primitive.dimensions[0] = 0.5;
      // primitive.dimensions[1] = 1;
      // primitive.dimensions[2] = 0.63;

      primitive.dimensions[0] = 0.45;
      primitive.dimensions[1] = 0.9;
      primitive.dimensions[2] = 0.3;

      geometry_msgs::Pose object_pose;

      object_pose.position.x = 0.95;
      object_pose.position.y = 0;
      object_pose.position.z = -0.585;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(object_pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);

      ROS_INFO("Add an object into the world");
      planning_scene_interface.addCollisionObjects(collision_objects);
    }


    void createLookup(size_t width, size_t height) {
      const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
      const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
      const float cx = cameraMatrixColor.at<double>(0, 2);
      const float cy = cameraMatrixColor.at<double>(1, 2);
      float *it;

      lookupY = cv::Mat(1, height, CV_32F);
      it = lookupY.ptr<float>();
      for (size_t r = 0; r < height; ++r, ++it) {
        *it = (r - cy) * fy;
      }

      lookupX = cv::Mat(1, width, CV_32F);
      it = lookupX.ptr<float>();
      for (size_t c = 0; c < width; ++c, ++it) {
        *it = (c - cx) * fx;
      }
    }


    bool turnOnVacuumGripper() {
      ROS_INFO_STREAM_NAMED("left", "Opening end effector");

      baxter_core_msgs::EndEffectorCommand command;
      command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
      command.args = "{\"grip_attempt_seconds\": 20.0}";
      command.id = 65537;

      // Send command several times to be safe
      for (std::size_t i = 0; i < GRIPPER_MSG_RESEND; ++i) {
        command_topic_.publish(command);

        ros::Duration(MSG_PULSE_SEC).sleep();
        ros::spinOnce();
      }
      return true;
    }

    bool turnOffVacuumGripper() {
      ROS_INFO_STREAM_NAMED("left", "Opening end effector");

      baxter_core_msgs::EndEffectorCommand command;
      command.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
      command.args = "";
      command.id = 65537;

      // Send command several times to be safe
      for (std::size_t i = 0; i < GRIPPER_MSG_RESEND; ++i) {
        command_topic_.publish(command);
        ros::Duration(MSG_PULSE_SEC).sleep();
        ros::spinOnce();
      }
      return true;
    }

    bool openGripper() {
      ROS_INFO_STREAM_NAMED("left", "Opening end effector");

      baxter_core_msgs::EndEffectorCommand command;
      command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
      command.args = "{\"position\": 100.0}";
      command.id = 65538;

      // Send command several times to be safe
      for (std::size_t i = 0; i < GRIPPER_MSG_RESEND; ++i) {
        command_topic_.publish(command);
        ros::Duration(MSG_PULSE_SEC).sleep();
        ros::spinOnce();
      }
      return true;
    }

    bool closeGripper() {
      ROS_INFO_STREAM_NAMED("left", "Closing end effector");

      baxter_core_msgs::EndEffectorCommand command;
      command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
      command.args = "{\"position\": 0.0}";
      command.id = 65538;
      // Send command several times to be safe
      for (std::size_t i = 0; i < GRIPPER_MSG_RESEND; ++i) {
        command_topic_.publish(command);
        ros::Duration(MSG_PULSE_SEC).sleep();
        ros::spinOnce();
      }

      return true;
    }

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

          ROS_INFO_STREAM("cloud width: " << cloud->width <<
          ", cloud height: " << cloud->height);

          int bounds = 2;
          float length = 0.0;

          float z_sum = 0.0;
          float x_sum = 0.0;
          float y_sum = 0.0;

          for (int i = x-bounds; i <= x+bounds; i++) {
            for (int j = y-bounds; j <= y+bounds; j++) {
              if (!isnan(cloud->points[i+j*width].z)) {
                std::cout << cloud->points[i+j*width].z << ", ";
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

          std::cout <<"You have selected to pick fruit at x,y:" <<
          x << ", " << y << std::endl;
          pick_fruit = true;
          pick_fruit_index = 0;
          std::cout << "YO" << std::endl;

          break;
        }
      }
    }

    static void mouse_click(int event, int x, int y, int, void* this_) {
      static_cast<fruitpicker*>(this_)->mouse_click(event, x, y);
    }
  };


  int main(int argc, char** argv) {

    ros::init(argc, argv, "fruitpicker");

      ROS_INFO_STREAM("WTF?");
    fruitpicker fp;

    fp.start();

    return 0;
  }
