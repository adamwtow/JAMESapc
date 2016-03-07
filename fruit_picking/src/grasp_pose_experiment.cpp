
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <ur_msgs/SetIO.h>
#include <ur_msgs/SetIO.h>

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

//#include <baxter_core_msgs/EndEffectorCommand.h>

#include <capsicum_detector.h>
#include <cloud_filtering_tools.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";

static const double MSG_PULSE_SEC = 0.1;
static const double WAIT_GRIPPER_CLOSE_SEC = 0.1;
static const double WAIT_STATE_MSG_SEC = 1; // max time to wait for the gripper state to refresh
static const double GRIPPER_MSG_RESEND = 10; // Number of times to re-send a msg to the end effects for assurance that it arrives

//#define NAIVE
class fruitpicker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber pcl_sub;
  image_transport::Subscriber image_sub;

  //capsicum detector code
  HSV_model capsicum_model;
  capsicum_detector capsicumDetector;
  capsicum target_capsicum;
  std::vector<capsicum> capsicums;

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

  cv::Mat lookupX, lookupY;

  geometry_msgs::PoseStamped capsicum_pose;
  geometry_msgs::PoseStamped capsicum_base_pose;
  geometry_msgs::PoseStamped arm_pose;
  std::vector<double> start_joint_values = {0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};

  ros::Publisher vis_pub, kinfu_reset_pub, estimate_capsicum_pub;

  ros::ServiceClient io_client;
  ros::ServiceClient estimateCapsicumPose_client;


  moveit::planning_interface::MoveGroup::Plan ur5_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup ur5;

  //Starting Position
  double start_z,start_x, start_y, delta_z, delta_x, delta_y;
  double velocity_scan, velocity_picking,gripper_offset_up,gripper_offset_in ;

  double start_q[4] = {1.0, 0, 0, 0};
//  double q1[4] = {0.987,-0.152,0.0,0.0};
//  double q2[4] = {0.964, 0, -0.266, 0};
//  double q3[4] = {0.951, 0.305, 0.0,0.0};

//  double q1[4] = {0.987,-0.152,0.0,0.0};
//  double q2[4] = {0.964, 0, -0.266, 0};
//  double q3[4] = {0.951, 0.305, 0.0,0.0};


public:
  fruitpicker()
    : it_(nh_), ur5("manipulator")
    {
        cv::namedWindow(OPENCV_WINDOW);

        vis_pub = nh_.advertise<geometry_msgs::PoseStamped>( "gripper_pose", 0 );
        kinfu_reset_pub = nh_.advertise<std_msgs::Empty>( "/ros_kinfu/reset", 0 );
        estimate_capsicum_pub = nh_.advertise<std_msgs::Empty>( "/superquadric/estimate_pose", 0 );
        io_client = nh_.serviceClient<ur_msgs::SetIO>("set_io");
        estimateCapsicumPose_client = nh_.serviceClient<superquadric_fitter::estimateCapsicumPose>("/superquadric_fitter/estimate_capsicum_pose");

        pick_fruit = false;
        pick_fruit_index = 0;
        found_fruit = false;


#ifdef NAIVE
        //Starting Position
        start_z = 0.97;
        start_y = -0.42;
        start_x = 0.318;
        delta_z = 0.08;
        delta_x = 0.18;
        delta_y = 0.06;
#else
        start_z = 0.97;
        start_y = -0.42;
        start_x = 0.318;
        delta_z = 0.08;
        delta_x = 0.065;
        delta_y = 0.065;

//        start_z = 0.97;
//        start_y = -0.42;
//        start_x = 0.318;
//        delta_z = 0.08;
//        delta_x = 0.18;
//        delta_y = 0.06;
#endif

        velocity_scan = 0.05;
        velocity_picking = 0.1;

        gripper_offset_up = 0.05;
        gripper_offset_in = 0.15;

        std::string topicCapsicumCloud, topicCapsicumImage;
        if (!nh_.getParam("capsicum_cloud_topic", topicCapsicumCloud)) topicCapsicumCloud = "/capsicum/points";
        if (!nh_.getParam("capsicum_image_topic", topicCapsicumImage)) topicCapsicumImage = "/camera/rgb/image_raw";

        //image_sub = it_.subscribe(topicCapsicumImage, 1, &fruitpicker::image_callback, this);
        //pcl_sub = nh_.subscribe(topicCapsicumCloud, 1, &fruitpicker::pcl_callback, this);

    }

    ~fruitpicker()
    {
        turnOffSuction(4);
        cv::destroyWindow(OPENCV_WINDOW);
//        std::vector<std::string> object_names = planning_scene_interface.getKnownObjectNames();
//        planning_scene_interface.removeCollisionObjects(object_names);
    }

    void start(){

        float wait_time = 0.0;
        cv::setMouseCallback(OPENCV_WINDOW, mouse_click, this);
        bool success = false;
		bool capsicumAttached = false;
        bool reset = false;
        ros::Time time;

        ur5.setPlannerId("RRTConnectkConfigDefault");
        //ur5.setPlannerId("KPIECEkConfigDefault");
        ur5.setPlanningTime(10);
        ur5.setMaxVelocityScalingFactor(velocity_picking);

        ros::AsyncSpinner spinner(2);
        spinner.start();

        std::cout << "Moving to neutral" << std::endl;
//        moveToNamed("neutral",2,wait_time);
        if(!moveTo("world", start_x, -0.42, start_z, start_q, 2, wait_time)){reset = true; }


        while(ros::ok())
        {
            if(reset)
            {
                std::cout << "Reset flagged moving to neutral" << std::endl;
                //moveToNamed("neutral",2,wait_time);
                if(!moveTo("world", start_x, -0.42, start_z, start_q, 2, wait_time)){reset = true; }
                reset = false;
                pick_fruit = false;
                if(capsicumAttached){
                    dettachCapsicumObject();
                }
                turnOffSuction(4);
                removeCapsicumObject();
            }

            cout << "Press the ENTER key to pick fruit";

            if (cin.get() == '\n'){
                 cout << "Picking Fruit." << endl;

#ifdef NAIVE
                 if(!moveTo("world", start_x, start_y, start_z, start_q, 2, wait_time)){reset = true; }

                 if(!moveTo("world", start_x, start_y, start_z + delta_z, start_q, 2, wait_time)){reset = true; }

                 if(!turnOnSuction(4)) std::cout << "Couldn't Turn on Suction" << std::endl;
                 if(!moveTo("world", start_x + delta_x, start_y, start_z + delta_z, start_q, 2, wait_time)){reset = true; }

                 if(!moveTo("world", start_x, start_y, start_z + delta_z, start_q, 2, wait_time)){reset = true; }
                 turnOffSuction(4);

                 if(!moveTo("world", start_x, start_y, start_z, start_q, 2, wait_time)){reset = true; }
#else
                 kinfu_reset_pub.publish(std_msgs::Empty());

                //Build up multiple views of capsicum
                 ur5.setMaxVelocityScalingFactor(velocity_scan);
                 if(!moveTo("world", start_x, start_y + delta_y, start_z, start_q, 2, wait_time)){reset = true; }
                 if(!moveTo("world", start_x, start_y, start_z + delta_z, start_q, 2, wait_time)){reset = true; }
                 if(!moveTo("world", start_x, start_y - delta_y, start_z, start_q, 2, wait_time)){reset = true; }
                 if(!moveTo("world", start_x, start_y, start_z, start_q, 2, wait_time)){reset = true; }


                    Eigen::Affine3d capsicum_transform;
                    Eigen::Affine3d capsicum_grasp_translation;

                    superquadric_fitter::estimateCapsicumPose srv;
                    srv.request.empty = std_msgs::Empty();
                    if(estimateCapsicumPose_client.call(srv))
                    {

                        ROS_INFO("Got Capsicum Pose Response");

                        tf::transformMsgToEigen(srv.response.transform,capsicum_transform);

                    //bool tf_success = tf_listener.waitForTransform("/world", "/capsicum_frame",  ros::Time(0), ros::Duration(10));
                    //if (tf_success)
                    //{
                        //tf_listener.lookupTransform("/world", "/capsicum_frame", ros::Time(0), capsicum_stamped_transform);
                        //tf::transformTFToEigen(capsicum_stamped_transform, capsicum_transform);

                        Eigen::Affine3d capsicum_grasp_transform;

                        //Eigen::Vector3d model_abc(srv.response.a,srv.response.b,srv.response.c);
                        Eigen::Vector3d grasp_offset(0.25,0,gripper_offset_up);

                        createGraspTransform(capsicum_transform,capsicum_grasp_transform, grasp_offset);

                        geometry_msgs::PoseStamped capsicum_grasp_pose;
                        tf::poseEigenToMsg(capsicum_grasp_transform,capsicum_grasp_pose.pose);
                        capsicum_grasp_pose.header.frame_id = ("/world");
                        capsicum_grasp_pose.header.stamp = ros::Time::now();
                        vis_pub.publish(capsicum_grasp_pose);

                        ur5.setMaxVelocityScalingFactor(velocity_picking);

                        if(!moveTo(capsicum_grasp_pose,2,wait_time)){reset = true; continue;}


                        capsicum_grasp_translation = Eigen::Translation3d(Eigen::Vector3d(gripper_offset_in,0,0));
                        capsicum_grasp_transform = capsicum_grasp_transform*capsicum_grasp_translation;
                        tf::poseEigenToMsg(capsicum_grasp_transform,capsicum_grasp_pose.pose);
                        capsicum_grasp_pose.header.stamp = ros::Time::now();
                        vis_pub.publish(capsicum_grasp_pose);

                        if(!turnOnSuction(4)) std::cout << "Couldn't Turn on Suction" << std::endl;
                        if(!moveTo(capsicum_grasp_pose,2,wait_time)){reset = true; continue;}

                        capsicum_grasp_translation = Eigen::Translation3d(Eigen::Vector3d(-gripper_offset_in,0,0));
                        capsicum_grasp_transform = capsicum_grasp_transform*capsicum_grasp_translation;
                        tf::poseEigenToMsg(capsicum_grasp_transform,capsicum_grasp_pose.pose);
                        capsicum_grasp_pose.header.stamp = ros::Time::now();
                        vis_pub.publish(capsicum_grasp_pose);

                        if(!moveTo(capsicum_grasp_pose,2,wait_time)){reset = true; continue;}


                        if(!moveTo("world", start_x, start_y, start_z, start_q, 2, wait_time)){reset = true; }
                        if(!turnOffSuction(4)) std::cout << "Couldn't Turn off  Suction" << std::endl;




                    }else{
                        ROS_ERROR("Did not receive a response from server");
                        reset = true; continue;

                    }
	#endif
            }
        }
    }

  bool moveToNamed(std::string namedGoal, int Attempts, float timeout){
	  
	bool ret = false;
    bool success = false;

	while(Attempts > 0){
        ur5.setStartStateToCurrentState();
        ur5.setNamedTarget(namedGoal);

        success = ur5.plan(ur5_plan);
	
		if(success){
            std::cout << "Successfully planned to: " << namedGoal << std::endl;
			try{
                ur5.move();
                std::cout << "Moved to: " << namedGoal << std::endl;
				Attempts = 0;
				ret = true;
			}
			catch(moveit::planning_interface::MoveItErrorCode ex){
                std::cout << "Something went wrong. Failed to move to pose" << std::endl;
                ret = false;
			}
		}else{
            std::cout << "Failed to plan, trying to replan" << std::endl;
			Attempts--;
            sleep(timeout);
		}
	}
	return ret;
	  
  }
  
  bool moveTo(geometry_msgs::PoseStamped pose, int Attempts, float timeout){
	bool ret = false;
    bool success = false;
    
	while(Attempts > 0){
        ur5.setStartStateToCurrentState();
        pose.header.stamp = ros::Time::now();
        ur5.setPoseTarget(pose);

        success = ur5.plan(ur5_plan);
	
		if(success){
			std::cout << "Successfully planned to goal, moving to goal" << std::endl;
			try{
                ur5.move();
                std::cout << "Moved to pose" << std::endl;
				Attempts = 0;
				ret = true;
			}
			catch(moveit::planning_interface::MoveItErrorCode ex){
                std::cout << "Something went wrong. Failed to move to pose" << std::endl;
                //ROS_ERROR("%s",ex.what());
                ret = false;
			}
		}else{
			Attempts--;
            sleep(timeout);
		}
	}
	return ret;
  }

  bool moveTo(std::string frame_id, double x, double y,double z, double* q, int Attempts, float timeout){
    bool ret = false;
    bool success = false;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = q[0];
    pose.pose.orientation.y = q[1];
    pose.pose.orientation.z = q[2];
    pose.pose.orientation.w = q[3];

    while(Attempts > 0){
        ur5.setStartStateToCurrentState();
        pose.header.stamp = ros::Time::now();
        ur5.setPoseTarget(pose);

        success = ur5.plan(ur5_plan);

        if(success){
            std::cout << "Successfully planned to goal, moving to goal" << std::endl;
            try{
                ur5.move();
                std::cout << "Moved to pose" << std::endl;
                Attempts = 0;
                ret = true;
            }
            catch(moveit::planning_interface::MoveItErrorCode ex){
                std::cout << "Something went wrong. Failed to move to pose" << std::endl;
                //ROS_ERROR("%s",ex.what());
                ret = false;
            }
        }else{
            Attempts--;
            sleep(timeout);
        }
    }
    return ret;
  }


  void publishTFfromPose(Eigen::Affine3d pose, std::string parent_frame, std::string child_frame)
  {
      static tf::TransformBroadcaster tf_broadcaster;

      tf::Transform transform;
      tf::transformEigenToTF(pose,transform);

      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));

  }

  void createGraspTransform(Eigen::Affine3d &capsicum_transform, Eigen::Affine3d &grasp_transform, Eigen::Vector3d &grasp_offset){

    Eigen::Matrix3d capsicum_rotation = capsicum_transform.rotation();
    Eigen::Matrix3d grasp_pose_rotation;
    Eigen::Vector3d grasp_pose_translation;

    int maxIndex[3];

    int frontAxis, sideAxis, stemAxis;
    capsicum_rotation.row(0).cwiseAbs().maxCoeff(&frontAxis);
    capsicum_rotation.row(1).cwiseAbs().maxCoeff(&sideAxis);
    capsicum_rotation.row(2).cwiseAbs().maxCoeff(&stemAxis);

    Eigen::Vector3d frontVector = capsicum_rotation.col(frontAxis);
    Eigen::Vector3d stemVector = capsicum_rotation.col(stemAxis);
    Eigen::Vector3d sidevector = capsicum_rotation.col(sideAxis);

    double angleFrontAxis = acos(frontVector[0]/frontVector.norm());
    double angleSideAxis = acos(sidevector[1]/sidevector.norm());
    double angleStemAxis = acos(stemVector[2]/stemVector.norm());

    ROS_INFO_STREAM("Capsicum Rotation Matrix: "<< capsicum_rotation);
    ROS_INFO_STREAM("Grasp Pose Rotation Matrix: "<< grasp_pose_rotation);


    std::vector<std::string> Axis = {"x","y","z"};
    ROS_INFO_STREAM("Angle to front face: " << angleFrontAxis*180/M_PI << " Capsicum front axis: " << Axis[frontAxis]);
    ROS_INFO_STREAM("Angle to side face: " << angleSideAxis*180/M_PI << " Capsicum side axis: " << Axis[sideAxis]);
    ROS_INFO_STREAM("Angle to stem face: " << angleStemAxis*180/M_PI << " Capsicum stem axis: " << Axis[stemAxis]);

    if(angleFrontAxis < M_PI/2) grasp_pose_rotation.col(0) = capsicum_rotation.col(frontAxis);
    else grasp_pose_rotation.col(0) = -1*capsicum_rotation.col(frontAxis);

    if(angleStemAxis > M_PI/2) grasp_pose_rotation.col(2) = capsicum_rotation.col(stemAxis);
    else grasp_pose_rotation.col(2) = -1*capsicum_rotation.col(stemAxis);

    grasp_pose_rotation.col(1) = -1*(grasp_pose_rotation.col(0).cross(grasp_pose_rotation.col(2)));

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = grasp_pose_rotation; //grasp_pose_rotation;
    transform.block<3,1>(0,3) = capsicum_transform.translation();
    Eigen::Affine3d grasp_transform_eigen(transform);

    //grasp_offset *= -1;
    //grasp_offset[frontAxis] = -1*grasp_offset[frontAxis] - model_abc[frontAxis];
    translation.block<3,1>(0,3) = -1*grasp_offset ;
    Eigen::Affine3d transform_translation(translation);

    publishTFfromPose(grasp_transform_eigen*transform_translation, "/world", "/gripper_frame");

    grasp_transform = grasp_transform_eigen*transform_translation;

    //tf::poseEigenToMsg(grasp_transform*transform2,grasp_pose);

//    ROS_INFO_STREAM("Capsicum Rotation Matrix: "<< capsicum_rotation);
//    ROS_INFO_STREAM("Grasp Pose Rotation Matrix: "<< grasp_pose_rotation);

  }


  void attachCapsicumObject(){
      ur5.attachObject("capsicum","ee_link");
  }

  void dettachCapsicumObject(){
      ur5.detachObject("capsicum");
  }

  void removeCapsicumObject(){
      std::vector<std::string> object_names = {"capsicum"};
      planning_scene_interface.removeCollisionObjects(object_names);
  }

  void addBoxObject(){

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = ur5.getPlanningFrame();
      collision_object.id = "box";

      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);

      primitive.dimensions[0] = 0.45;
      primitive.dimensions[1] = 0.9;
      primitive.dimensions[2] = 0.55;

      geometry_msgs::Pose object_pose;


      object_pose.position.x = 0.95;
      object_pose.position.y = 0;
      object_pose.position.z = -0.575;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(object_pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);

      ROS_INFO("Add an object into the world");
      planning_scene_interface.addCollisionObjects(collision_objects);

  }

  void addCapsicumObject(geometry_msgs::Pose object_pose){

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = "/world";
      collision_object.id = "capsicum";

      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[0] = 0.08;
      primitive.dimensions[1] = 0.04;

      object_pose.orientation.x = 3.14;
      object_pose.orientation.y = 0;
      object_pose.orientation.z = 0;


      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(object_pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);

      ROS_INFO("Add an object into the world");
      planning_scene_interface.addCollisionObjects(collision_objects);

  }


  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }


 bool turnOnSuction(int8_t pin){

     ur_msgs::SetIO io_srv;
     bool success = false;

     io_srv.request.fun = 1;
     io_srv.request.pin = pin;
     io_srv.request.state = true;


     if(io_client.call(io_srv)){
         success = io_srv.response.success;
     }

     return success;
 }

 bool turnOffSuction(int8_t pin){

     ur_msgs::SetIO io_srv;
     bool success = false;

     io_srv.request.fun = 1;
     io_srv.request.pin = pin;
     io_srv.request.state = false;


     if(io_client.call(io_srv)){
         success = io_srv.response.success;
     }

     return success;
 }


 void pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg){

     PointCloud::Ptr cloud(new PointCloud);
     PointCloud::Ptr capsicum_cloud(new PointCloud);
     Eigen::Vector4f centroid;

     pcl::fromROSMsg(*msg,*cloud);
     pcl::compute3DCentroid(*capsicum_cloud, centroid);
     pcl::PointXYZ centre_point(centroid[0],centroid[1],centroid[2]);
     target_capsicum.centroid = centre_point;

     found_fruit = true;

 }

 void image_callback(const sensor_msgs::ImageConstPtr& msg){

     cv::Mat rgb_image_;
     readImage(msg,rgb_image_);

     cv::Mat highlighted = capsicumDetector.detect(rgb_image_,200);

     if(capsicumDetector.nCapsicumFound >= 1){
         found_fruit = true;
         capsicums.clear();

         for( unsigned int j = 0; j < capsicumDetector.nCapsicumFound; j++ ) {
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
                  for (unsigned int i=0; i<capsicumDetector.nCapsicumFound; i++){
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
    static_cast<fruitpicker*>(this_)->mouse_click(event, x, y);
  }

};



int main(int argc, char** argv)
{
  std::vector<capsicum> caps; //= new std::vector<capsicum>();

  ros::init(argc, argv, "grasp_node");
  fruitpicker fp;

  fp.start();

  return 0;
}
