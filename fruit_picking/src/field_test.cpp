
#include <ros/ros.h>
#include <ros/time.h>dr
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "superquadric_fitter/estimateCapsicumPose.h"
#include "capsicum_detection/segmentCapsicum.h"
#include <capsicum_detector.h>
#include <cloud_filtering_tools.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";

static const double MSG_PULSE_SEC = 0.1;
static const double WAIT_GRIPPER_CLOSE_SEC = 0.1;
static const double WAIT_STATE_MSG_SEC = 1; // max time to wait for the gripper state to refresh
static const double GRIPPER_MSG_RESEND = 10; // Number of times to re-send a msg to the end effects for assurance that it arrives

#define DEBUG

class harvey
{

    enum ROBOT_STATE_FLAG {
        RESET,
        START,
        DETECT_INIT_FRUIT,
        SCANNING,
        DETECT_FRUIT,
        ATTACH,
        CUT,
        PLACE
    };

  std::vector<std::string> ROBOT_STATES_NAMES = {
      "RESET",
      "START",
      "DETECT_INIT_FRUIT",
      "SCANNING",
      "DETECT_FRUIT",
      "ATTACH",
      "CUT",
      "PLACE"
  };


  //capsicum detector code
  HSV_model capsicum_hsv_model;
  capsicum_detector capsicumDetector;
  geometry_msgs::PointStamped target_capsicum_centroid;
  std::vector<capsicum> capsicums;

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  tf::TransformListener tf_listener;

  bool pick_fruit, found_fruit, reset_flag;
  int  pick_fruit_index;

  int SUCTION_PIN = 4;
  int CUTTER_PIN = 5;
  int PRESSURE_SWITCH = 0;


  cv::Mat cameraMatrixColor;
  cv::Mat cameraMatrixDepth;
  cv::Mat scene_image;

  cv::Mat lookupX, lookupY;

  geometry_msgs::PoseStamped capsicum_pose;
  geometry_msgs::PoseStamped capsicum_base_pose;
  geometry_msgs::PoseStamped arm_pose;
  Eigen::Affine3d capsicum_pose_eigen;
  Eigen::Vector3d capsicum_model;

  sensor_msgs::PointCloud2 scene_cloud_msg;

  std::vector<double> start_joint_values = {0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber image_sub;

  ros::Subscriber capsicum_cloud_sub, scene_cloud_sub, io_states_sub;
  ros::Publisher wire_cutter_pub, vis_pub, kinfu_reset_pub, estimate_capsicum_pub;

  ros::ServiceClient io_client;
  ros::ServiceClient estimateCapsicumPose_client;
  ros::ServiceClient segmentCapsicum_client;

  std::string gripperPoseTopic, kinfuResetTopic, superquadricTopic, urIoTopic, urIoStateTopic;
  std::string segmentCapsicumTopic, topicCapsicumCloud, topicCapsicumImage, topicKinfuSceneCloud;

  ROBOT_STATE_FLAG robot_state_;

  Eigen::Affine3d grasp_pose_eigen;

//  moveit::planning_interface::MoveGroup::Plan ur5_plan;
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  moveit::planning_interface::MoveGroup ur5;
  double wait_time;
  int attempts;

  std::vector<ur_msgs::Digital> digital_in_states;

  //Starting Position
  Eigen::Vector3d start_xyz, delta_xyz, tool_point, scan_offset;
  Eigen::Vector4d start_q;
  double velocity_scan, velocity_picking,gripper_offset_up,gripper_offset_in, gripper_offset_init, gripper_offset_side;
  double row_depth, focal_x, focal_y;
  double tool_point_x, tool_point_y, tool_point_z, tool_offset_x;

  double eef_step_size, jump_threshold;

public:
  harvey()
    : nh_("~"),it_(nh_) //ur5("manipulator")
    {
        cv::namedWindow(OPENCV_WINDOW);

        nh_.param("gripper_pose_topic",gripperPoseTopic,std::string("gripper_pose"));
        nh_.param("kinfu_reset_topic",kinfuResetTopic,std::string("/ros_kinfu/reset"));
        nh_.param("superquadric_topic",superquadricTopic,std::string("/superquadric_fitter/estimate_capsicum_pose"));
        nh_.param("ur_set_io_topic",urIoTopic,std::string("/set_io"));
        nh_.param("segment_capsicum_topic",segmentCapsicumTopic,std::string("/capsicum_detection/segment_capsicum"));

        vis_pub = nh_.advertise<geometry_msgs::PoseStamped>( gripperPoseTopic, 0 );
        kinfu_reset_pub = nh_.advertise<std_msgs::Empty>( kinfuResetTopic, 0 );
        wire_cutter_pub = nh_.advertise<std_msgs::Float64>("/wire_cutter_controller1/command", 0);
        io_client = nh_.serviceClient<ur_msgs::SetIO>(urIoTopic);
        estimateCapsicumPose_client = nh_.serviceClient<superquadric_fitter::estimateCapsicumPose>(superquadricTopic);
        segmentCapsicum_client = nh_.serviceClient<capsicum_detection::segmentCapsicum>(segmentCapsicumTopic);

        pick_fruit = false;
        pick_fruit_index = 0;
        found_fruit = false;

        nh_.param("velocity_scan_scaling", velocity_scan, 0.1);
        nh_.param("velocity_picking_scaling", velocity_picking,0.25);

        nh_.param("eef_step_size", eef_step_size,0.01);
        nh_.param("jump_threshold", jump_threshold,0.0);

        //nh_.param("scan_offset",scan_offset, 0.45);
        nh_.param("row_depth",row_depth,0.5);
        nh_.param("focal_x",focal_x, 463.888885);
        nh_.param("focal_y",focal_y, 463.888885);

        nh_.param("gripper_offset_x",tool_point[0],0.37);
        nh_.param("gripper_offset_y",tool_point[1],0.0);
        nh_.param("gripper_offset_z",tool_point[2], 0.06);

        nh_.param("tool_offset_x",tool_offset_x,0.2);

        nh_.param("gripper_offset_in",gripper_offset_in,0.13);
        nh_.param("gripper_offset_init",gripper_offset_init,0.47);

        nh_.param("scan_offset_x",scan_offset[0], 0.37);
        nh_.param("scan_offset_y",scan_offset[1], 0.0);
        nh_.param("scan_offset_z",scan_offset[2], 0.1);

        //double delta_z, delta_x, delta_y;
        nh_.param("scan_delta_x", delta_xyz[0], 0.065);
        nh_.param("scan_delta_y", delta_xyz[1], 0.1);
        nh_.param("scan_delta_z", delta_xyz[2], 0.1);

        nh_.param("start_x",start_xyz[0], 0.15);
        nh_.param("start_y",start_xyz[1],-0.52);
        nh_.param("start_z",start_xyz[2], 1.1);

        nh_.param("start_q1",start_q[0],1.0);
        nh_.param("start_q2",start_q[1],0.0);
        nh_.param("start_q3",start_q[2],0.0);
        nh_.param("start_q4",start_q[3],0.0);

        nh_.param("wait_time",wait_time,0.0);
        nh_.param("planning_attempts",attempts,4);

        nh_.param("capsicum_cloud_topic",topicCapsicumCloud,std::string("/capsicum/points"));
        nh_.param("kinfu_cloud_topic",topicKinfuSceneCloud,std::string("/ros_kinfu/depth_registered/points"));
        nh_.param("capsicum_image_topic",topicCapsicumImage,std::string("/camera/rgb/image_raw"));

        ROS_INFO_STREAM("Kinfu Cloud Topic: " << topicKinfuSceneCloud);
        ROS_INFO_STREAM("Capsicum Image Topic: " << topicCapsicumImage);

        scene_cloud_sub = nh_.subscribe(topicKinfuSceneCloud, 1, &harvey::scene_cloud_callback, this);

        //nh_.param("ur_io_states_topic",urIoStateTopic,std::string("/io_states"));
        //io_states_sub = nh_.subscribe(urIoStateTopic, 1, &harvey::io_states_callback, this);
    }

    ~harvey()
    {
        //turnOffIO(SUCTION_PIN);
        cv::destroyWindow(OPENCV_WINDOW);
//        std::vector<std::string> object_names = planning_scene_interface.getKnownObjectNames();
//        planning_scene_interface.removeCollisionObjects(object_names);
    }

    void start(){

//        geometry_msgs::Pose start_pose;

//        //start_pose.header.frame_id = "world";
//        start_pose.orientation.x = start_q[0];
//        start_pose.orientation.y = start_q[1];
//        start_pose.orientation.z = start_q[2];
//        start_pose.orientation.w = start_q[3];

//        start_pose.position.x = start_xyz[0];
//        start_pose.position.y = start_xyz[1];
//        start_pose.position.z = start_xyz[2];

        cv::setMouseCallback(OPENCV_WINDOW, mouse_click, this);

        reset_flag = false;
        ros::Time time;

        updateState(DETECT_FRUIT);

        ros::AsyncSpinner spinner(2);
        spinner.start();

        //std::cout << "Moving to neutral" << std::endl;
        //moveToNamed("neutral",2,wait_time);

        kinfu_reset_pub.publish(std_msgs::Empty());

        while(ros::ok())
        {
            sensor_msgs::PointCloud2 segmentedCapsicum;
            switch (robot_state_) {



            case DETECT_FRUIT:

                if(!segmentCapsicum(scene_cloud_msg,segmentedCapsicum)){
                    updateState(RESET);
                    break;
                }

                if(estimateCapsicum(segmentedCapsicum,capsicum_pose_eigen,capsicum_model)){
                    updateState(ATTACH);
                }else{
                    updateState(RESET);
                }
                break;



//                if(moveToCartesianPath(waypoints, velocity_scan, attempts, wait_time)){
//                    moveToNamed("box_ground",2,wait_time);
//                    updateState(START);
//                }else{
//                    updateState(RESET);
//                }
                break;

            }

        }
    }

    void updateState(ROBOT_STATE_FLAG new_state){
        robot_state_ = new_state;
        ROS_INFO_STREAM("Robot State: " << ROBOT_STATES_NAMES[new_state] );
    }





    void publishPosefromEigen(Eigen::Affine3d pose_eigen, std::string frame_id)
    {
        geometry_msgs::PoseStamped pose;

        tf::poseEigenToMsg(pose_eigen,pose.pose);
        pose.header.frame_id = frame_id;
        pose.header.stamp = ros::Time::now();

        vis_pub.publish(pose);
    }

    bool segmentCapsicum(sensor_msgs::PointCloud2 msg_in, sensor_msgs::PointCloud2 &msg_out){
        bool ret = false;
        capsicum_detection::segmentCapsicum srv;
        srv.request.cloud = msg_in;
        if(segmentCapsicum_client.call(srv))
        {
            ret = true;
            ROS_INFO("Got Segmented Capsicum Response");
            msg_out = srv.response.segmented_cloud;

        }else{
            ROS_ERROR("Did not receive a response from the server");
            ret = false;
        }
        return ret;
    }

    bool estimateCapsicum(sensor_msgs::PointCloud2 cloud, Eigen::Affine3d &capsicum_pose_eigen, Eigen::Vector3d &capsicum_model)
    {
        bool ret = false;
        superquadric_fitter::estimateCapsicumPose srv;
        srv.request.cloud = cloud;
        if(estimateCapsicumPose_client.call(srv))
        {
            ret = true;
            ROS_INFO("Got Capsicum Pose Response");

            tf::transformMsgToEigen(srv.response.transform,capsicum_pose_eigen);

            capsicum_model << srv.response.a, srv.response.b, srv.response.c;
        }else{
            ROS_ERROR("Did not receive a response from the server");
            ret = false;
        }
        return ret;
    }





 void scene_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg){
    scene_cloud_msg = *msg;
 }


 bool detect_init_capsicum_from_image(cv::Mat scene_image, float focal_x, float focal_y, float depth, geometry_msgs::PointStamped &capsicum_point){

     cv::Mat highlighted = capsicumDetector.detect(scene_image,200);

     if(capsicumDetector.nCapsicumFound >= 1){
         found_fruit = true;
         capsicums.clear();

         for( int j = 0; j < capsicumDetector.nCapsicumFound; j++ ) {
             capsicum cap(capsicumDetector.mu[j], capsicumDetector.boundRect[j]);
             capsicums.push_back(cap);
             //capsicums.insert(it,1,cap);
             //it++;
         }
     }else{
        ROS_INFO("Can not find any capsicums in current image");
	return false;
     }

     int image_x = capsicums[0].center_point.x;
     int image_y = capsicums[0].center_point.y;

     //Using planar assumption at depth the relationship is (focal_x/depth = ximage/xcartesian)
     capsicum_point.point.x = image_x*depth/focal_x;
     capsicum_point.point.y = image_y*depth/focal_y;
     capsicum_point.point.z = depth;

     cv::imshow("capsicums", highlighted);
     cv::waitKey(1);

     return true;

 }

 bool detect_init_capsicum(geometry_msgs::PointStamped &capsicum_centroid){

     sensor_msgs::PointCloud2 segmented_capsicum;

     if(!segmentCapsicum(scene_cloud_msg,segmented_capsicum)){
        return false;
     }

     if(segmented_capsicum.height*segmented_capsicum.width > 0)
     {
        PointCloud::Ptr segmented_capsicum_pcl(new PointCloud);
        Eigen::Vector4f centroid;
        pcl::fromROSMsg(segmented_capsicum,*segmented_capsicum_pcl);
        pcl::compute3DCentroid(*segmented_capsicum_pcl, centroid);


        try{

            geometry_msgs::PointStamped centroid_point;
            centroid_point.header.frame_id = "kf_world";
            centroid_point.point.x = centroid[0];
            centroid_point.point.y = centroid[1];
            centroid_point.point.z = centroid[2];

            //target_capsicum_centroid.header.frame_id = "world";
            tf_listener.transformPoint("world",centroid_point,capsicum_centroid);
            return true;

        }catch(tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          return false;
        }

     }else{
         return false;
     }


 }

 void find_fruit_callback(const sensor_msgs::PointCloud2ConstPtr msg){


     while(!found_fruit && (msg->height*msg->width > 0))
     {
        PointCloud::Ptr cloud(new PointCloud);
        PointCloud::Ptr capsicum_cloud(new PointCloud);
        Eigen::Vector4f centroid;
        pcl::fromROSMsg(*msg,*capsicum_cloud);
        pcl::compute3DCentroid(*capsicum_cloud, centroid);


        try{

            geometry_msgs::PointStamped centroid_point, world_point;
            centroid_point.header.frame_id = "kf_world";
            centroid_point.point.x = centroid[0];
            centroid_point.point.y = centroid[1];
            centroid_point.point.z = centroid[2];

            //target_capsicum_centroid.header.frame_id = "world";
            tf_listener.transformPoint("world",centroid_point,target_capsicum_centroid);
            found_fruit = true;

        }catch(tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        //pcl::PointXYZ centre_point(centroid[0],centroid[1],centroid[2]);

     }


 }

 void image_callback(const sensor_msgs::ImageConstPtr& msg){

     readImage(msg,scene_image);

 }

 void image_callback_detect(const sensor_msgs::ImageConstPtr& msg){

     cv::Mat rgb_image_;
     readImage(msg,rgb_image_);

     cv::Mat highlighted = capsicumDetector.detect(rgb_image_,200);

     if(capsicumDetector.nCapsicumFound >= 1){
         found_fruit = true;
         capsicums.clear();

         for( int j = 0; j < capsicumDetector.nCapsicumFound; j++ ) {
             capsicum cap(capsicumDetector.mu[j], capsicumDetector.boundRect[j]);
             capsicums.push_back(cap);
             //capsicums.insert(it,1,cap);
             //it++;
         }
     }


     cv::imshow(OPENCV_WINDOW, highlighted);
     cv::waitKey(1);

 }


  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }


  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix)
  {
      double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
        *itC = cameraInfo->K[i];
      }
  }

  void mouse_click(int event, int x, int y)
  {

      switch(event)
      {
          case CV_EVENT_LBUTTONDOWN:
          {
              std::cout<<"Mouse Pressed at x:"<< x << std::endl;
              std::cout<<"Mouse Pressed at y:"<< y << std::endl;
              if(found_fruit){
                  for (int i=0; i<capsicumDetector.nCapsicumFound; i++){
                      if((capsicums[i].center_point.x < x + 50)&&(capsicums[i].center_point.x > x-50)&&
                              (capsicums[i].center_point.y < y + 50)&&(capsicums[i].center_point.y > y-50)){
                                std::cout<<"You have selected to pick fruit at x,y:"<< x << ", " << y << std::endl;
                                pick_fruit = true;
                                pick_fruit_index = i;

                      }
                  }
              }

              break;
          }
      }

  }

  static void mouse_click(int event, int x, int y, int, void* this_) {
    static_cast<harvey*>(this_)->mouse_click(event, x, y);
  }

};



int main(int argc, char** argv)
{
  std::vector<capsicum> caps; //= new std::vector<capsicum>();

  ros::init(argc, argv, "field_trial_node");
  harvey fp;

  fp.start();

  return 0;
}
