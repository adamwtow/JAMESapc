
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
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


//typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;

class fruitpicker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfo;
  message_filters::Synchronizer<SyncPolicy> *sync;

  //capsicum detector code
  HSV_model capsicum_model;
  capsicum_detector capsicumDetector;

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

  geometry_msgs::PoseStamped capsicum_kinect_pose;
  geometry_msgs::PoseStamped capsicum_base_pose;
  geometry_msgs::PoseStamped arm_pose;
  std::vector<double> start_joint_values = {0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};

  ros::Publisher vis_pub;
  ros::Publisher command_topic_;

  ros::ServiceClient io_client;


  moveit::planning_interface::MoveGroup::Plan ur5_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup ur5;



public:
  fruitpicker()
    : it_(nh_), ur5("manipulator")
    {
        cv::namedWindow(OPENCV_WINDOW);
       // cv::namedWindow(OPENCV_WINDOW2);

        std::string topicColor = "/kinect2/hd/image_color_rect";
        std::string topicDepth = "/kinect2/hd/image_depth_rect";
        std::string topicCameraInfo = "/kinect2/hd/camera_info";
        //std::string topicCameraInfoDepth = "/kinect2_head/depth_lowres/camera_info";

        subImageColor = new image_transport::SubscriberFilter(it_, topicColor, 3);
        subImageDepth = new image_transport::SubscriberFilter(it_, topicDepth, 3);
        subCameraInfo = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfo, 3);
        //subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfoDepth, 4);

        vis_pub = nh_.advertise<geometry_msgs::PoseStamped>( "gripper_pose", 0 );
        io_client = nh_.serviceClient<ur_msgs::SetIO>("set_io");


        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);

        pick_fruit = false;
        pick_fruit_index = 0;
        found_fruit = false;
        create_lookup = true;

        capsicum_model.hue_mean = 84;
        capsicum_model.saturation_mean = 221;
        capsicum_model.value_mean = 121;
        capsicum_model.hue_var = 20.2;
        capsicum_model.saturation_var = 1368;
        capsicum_model.value_var = 753;

        //mean = (cv::Mat_<double>(1,3) << 43.14, 170.5, 66.17);
        //stdev = (cv::Mat_<double>(1,3) << 72.87, 52.84, 25.6);

        capsicumDetector = capsicum_detector(capsicum_model);

        //sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(4), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), *subImageColor, *subImageDepth, *subCameraInfo);
        sync->registerCallback(boost::bind(&fruitpicker::callback, this, _1, _2, _3));

    }

    ~fruitpicker()
    {
        cv::destroyWindow(OPENCV_WINDOW);

        std::vector<std::string> object_names = planning_scene_interface.getKnownObjectNames();
        planning_scene_interface.removeCollisionObjects(object_names);
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

        //openGripper();
        //addBoxObject();

        ros::AsyncSpinner spinner(2);
        spinner.start();

        //geometry_msgs::PoseStamped pose = ur5.getCurrentPose();
        //std::cout << pose << std::endl;

        std::cout << ur5.getPlanningFrame() << std::endl;

        std::cout << "Moving to box" << std::endl;
        moveToNamed("box_ground",2,wait_time);

        while(ros::ok()){

		if(reset){
			std::cout << "Reset flagged moving to box" << std::endl;
            moveToNamed("box_ground",2,wait_time);
            reset = false;
			pick_fruit = false;
            if(capsicumAttached){
					//openGripper();
                dettachCapsicumObject();
			}
            removeCapsicumObject();
		}
        if(pick_fruit){
            pick_fruit = false;
            int col = capsicums[pick_fruit_index].center_point.x;
            int row = capsicums[pick_fruit_index].center_point.y;
            cv::Rect capsicum_box = capsicums[pick_fruit_index].bounding_box;
            //const uint16_t Depth_val = depth.at<uint16_t>(row,col);
            double depth_val_avg = 0;
            int count = 0;


            for(int i = row - capsicum_box.height/4; i < row + capsicum_box.height/4; i++){
                for(int j = col - capsicum_box.width/4; j < col + capsicum_box.width/4; j++){
                    uint16_t depth_val = depth.at<uint16_t>(i,j);
                    if(!std::isnan(depth_val) && depth_val > 0){
                        //std::cout << depth_val << std::endl;
                        depth_val_avg += depth_val;
                        count++;
                    }
                }
            }

            //std::cout << depthValue << std::endl;
            const float depthValue = ((float)depth_val_avg/(float)count) / 1000.0f;
            //std::cout << depth_val_avg << std::endl;
            //std::cout << count << std::endl;

            const float y = lookupY.at<float>(0, row);
            const float x = lookupX.at<float>(0,col);

            capsicum_kinect_pose.pose.position.x = x * depthValue;
            capsicum_kinect_pose.pose.position.y = y * depthValue;
            capsicum_kinect_pose.pose.position.z = depthValue + 0.025;
            capsicum_kinect_pose.pose.orientation.x = 0.707;
            capsicum_kinect_pose.pose.orientation.y = 0.0;
            capsicum_kinect_pose.pose.orientation.z = 0.707;
            capsicum_kinect_pose.pose.orientation.w = 0.0;


            capsicum_kinect_pose.header.frame_id = ("/kinect2_link");
            capsicum_kinect_pose.header.stamp = ros::Time::now();



            try{
                tf_listener.waitForTransform("/world", "/kinect2_link",  time, ros::Duration(10));
                tf_listener.transformPose("/world",capsicum_kinect_pose,capsicum_base_pose);

				//add offset for first arm pose
                arm_pose.header.frame_id = ("/world");
                arm_pose.header.stamp = ros::Time::now();

                arm_pose.pose = capsicum_base_pose.pose;

                arm_pose.pose.position.x -= 0.4;
                arm_pose.pose.position.y += 0.025;
                arm_pose.pose.position.z += 0.047;
                arm_pose.pose.orientation.x = -1.0;
                arm_pose.pose.orientation.y = 0.0;
                arm_pose.pose.orientation.z = 0.0;
                arm_pose.pose.orientation.w = 0.0;

                //vis_pub.publish(arm_pose);

                //arm_pose = ur5.getCurrentPose();
                //arm_pose.pose.position.y += 0.2;
                //fruit_pose.pose = arm_pose.pose;
                //fruit_pose.pose.position.y -= 0.2;

                capsicum_base_pose.header.frame_id = "/world";
                capsicum_base_pose.header.stamp = ros::Time::now();
                //addCapsicumObject(capsicum_base_pose.pose);

                std::cout << "Attempting to grasp fruit at x,y,z: "
                          << capsicum_base_pose.pose.position.x << ", " << capsicum_base_pose.pose.position.y << ", " << capsicum_base_pose.pose.position.z << std::endl;

                if(!moveTo(arm_pose,2,wait_time)){reset = true; continue;}

                arm_pose.pose.position.x += 0.27;
                if(!moveTo(arm_pose,2,wait_time)){reset = true; continue;}
                if(!turnOnSuction(4)) std::cout << "Couldn't Turn on Suction" << std::endl;

                arm_pose.pose.position.x -= 0.27;
                if(!moveTo(arm_pose,2,wait_time)){reset = true; continue;}
                //attachCapsicumObject();

                if(!moveToNamed("box_ground",2,wait_time)){reset = true; continue;}
                if(!turnOffSuction(4)) std::cout << "Couldn't Turn off  Suction" << std::endl;
                //dettachCapsicumObject();

                //if(!moveToNamed("harvey_home_alt",2,wait_time)){continue; reset = true;}
                //removeCapsicumObject();

                /*
				//Plan and got to capsicum
                arm_pose.pose.position.x += 0.157; //
                if(!moveTo(arm_pose,2,wait_time)){continue; reset = true;}
				//closeGripper();
                std::cout << "Got Capsicum!" << std::endl;

				capsicumAttached = true;
				
				//Got capsicum so plan backwards to box
				arm_pose.pose.position.x -= 0.2;
                if(!moveTo(arm_pose,2,wait_time)){continue; reset = true;}
                if(!moveToNamed("neutral",2,wait_time)){continue; reset = true;}
				
				//release capsicum and remove object from scene
                //openGripper();*/



            }
            catch (tf::TransformException ex){
                reset = true;
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
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

  void attachCapsicumObject(){
//      ur5.attachObject("capsicum","right_gripper",{"right_gripper_l_finger","right_gripper_r_finger"});
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
      //primitive.dimensions[0] = 0.5;
      //primitive.dimensions[1] = 1;
      //primitive.dimensions[2] = 0.63;

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
      //collision_object.header.frame_id = ur5.getPlanningFrame();
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

/*bool openGripper()
{
  ROS_INFO_STREAM_NAMED("right","Opening end effector");

  //baxter_core_msgs::EndEffectorCommand command;
  //command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 100.0}";
  command.id = 65538;

  // Send command several times to be safe
  for (std::size_t i = 0; i < GRIPPER_MSG_RESEND; ++i)
  {
    command_topic_.publish(command);

    ros::Duration(MSG_PULSE_SEC).sleep();
    ros::spinOnce();
  }


  return true;
}*/

/*bool closeGripper()
{
  ROS_INFO_STREAM_NAMED("right","Closing end effector");

  //baxter_core_msgs::EndEffectorCommand command;
  //command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 0.0}";
  command.id = 65538;
  // Send command several times to be safe
  for (std::size_t i = 0; i < GRIPPER_MSG_RESEND; ++i)
  {
    command_topic_.publish(command);
    ros::Duration(MSG_PULSE_SEC).sleep();
    ros::spinOnce();
  }

  
  return true;
}*/


//void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
//              const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
//{

void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfo)
{
    cv::Mat highlighted;

    readImage(imageColor, color);
    readImage(imageDepth, depth);

    if(create_lookup == true){
        readCameraInfo(cameraInfo, cameraMatrixColor);
        readCameraInfo(cameraInfo, cameraMatrixDepth);
        createLookup(this->color.cols, this->color.rows);
        create_lookup = false;
    }


    int minArea = 200;

    highlighted = capsicumDetector.detect(color,depth,minArea);

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

    //segmentBlobs(color,depth,segmented);
    //segmentContours(color,depth,segmented);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, highlighted);
    cv::waitKey(3);


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

  ros::init(argc, argv, "fruit_picker");
  fruitpicker fp;

  fp.start();

  return 0;
}
