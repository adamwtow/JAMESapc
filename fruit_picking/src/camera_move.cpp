
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

//#include <ur_msgs/SetIO.h>

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


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";

static const double MSG_PULSE_SEC = 0.1;
static const double WAIT_GRIPPER_CLOSE_SEC = 0.1;
static const double WAIT_STATE_MSG_SEC = 1; // max time to wait for the gripper state to refresh
static const double GRIPPER_MSG_RESEND = 10; // Number of times to re-send a msg to the end effects for assurance that it arrives


class camera_move
{
  ros::NodeHandle nh_;

  geometry_msgs::PoseStamped camera_pose;

  moveit::planning_interface::MoveGroup::Plan ur5_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup ur5;



public:
  camera_move(): ur5("manipulator")
    {

    }

    ~camera_move()
    {
    }

    void start(){

        float wait_time = 0.0;
        bool success = false;
		bool capsicumAttached = false;
        bool reset = false;
        ros::Time time;

        ur5.setPlannerId("RRTConnectkConfigDefault");
        ur5.setPlanningTime(10);
        ur5.setMaxVelocityScalingFactor(0.05);

        ros::AsyncSpinner spinner(2);
        spinner.start();

        //while(ros::ok()){
        for(int i = 0; i < 5; i++)
              try{
                camera_pose.pose.position.x = 0.5;
                camera_pose.pose.position.y = 0.15;
                camera_pose.pose.position.z = 0.9;
                camera_pose.pose.orientation.x = 1;
                camera_pose.pose.orientation.y = 0.0;
                camera_pose.pose.orientation.z = 0.0;
                camera_pose.pose.orientation.w = 0.0;

                camera_pose.header.frame_id = ("/world");
                camera_pose.header.stamp = ros::Time::now();

                if(!moveTo(camera_pose,2,wait_time)){reset = true;}

                sleep(1.0);

                camera_pose.pose.position.x = 0.7;
                camera_pose.pose.position.y = 0.125;
                camera_pose.header.frame_id = ("/world");
                camera_pose.header.stamp = ros::Time::now();

                if(!moveTo(camera_pose,2,wait_time)){reset = true;}

                sleep(1.0);

                camera_pose.pose.position.x = 0.5;
                camera_pose.pose.position.y = 0.15;
                camera_pose.header.frame_id = ("/world");
                camera_pose.header.stamp = ros::Time::now();

                if(!moveTo(camera_pose,2,wait_time)){reset = true;}



            }
            catch (tf::TransformException ex){
                reset = true;
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

      //}
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
                //ur5.move();
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

 };



int main(int argc, char** argv)
{

  ros::init(argc, argv, "camera_move");
  camera_move fp;

  fp.start();

  return 0;
}
