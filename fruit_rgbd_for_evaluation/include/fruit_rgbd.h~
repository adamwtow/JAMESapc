#ifndef FRUIT_RGBD
#define FRUIT_RGBD

#include <capsicum_detector.h>
#include <cloud_filtering_tools.h>
#include <cloud_registration.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <deque>
#include <vector>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#define ICP
//#define NDT


//convenient typedefs
typedef pcl::PointXYZ PointT_;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT_> PointCloud_;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class fruit_rgbd_filter
{

public:

	fruit_rgbd_filter();
	~fruit_rgbd_filter();

	void pcl_callback(const sensor_msgs::PointCloud2 msg);
	void start();
	PointCloud::Ptr segment_capsicum(PointCloud::Ptr input_cloud);
	void image_callback(const sensor_msgs::Image::ConstPtr imageColor);
	void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud);
	void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);

	ros::NodeHandle nh_;

	//capsicum detector code
	HSV_model capsicum_model;
	capsicum_detector capsicumDetector;
	std::vector<capsicum> capsicums;

	PointCloud_::Ptr result, source,target;

	//Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	std::deque<PointCloud::Ptr> output_clouds;

	pcl::visualization::PCLVisualizer *visualizer;
	int vp_1,vp_2;
	//pcl::visualization::CloudViewer visualizer;

	tf::TransformListener listener;

	ros::Publisher pcl_pub;
	ros::Subscriber pcl_sub;
	ros::Subscriber image_sub;
	ros::Publisher pcl_merged_pub;
	bool first;

	bool init_flag_pc_;
	PointCloud_::Ptr init_pc_;
	Eigen::Matrix4f GlobalTransform;
	Eigen::Matrix4f pairTransform;
	ros::Publisher gt_pose_pub;
	#ifdef ICP
	ros::Publisher icp_pose_pub;
	#endif
	#ifdef NDT
	ros::Publisher ndt_pose_pub;
	#endif

	geometry_msgs::Pose gt_pose;
	geometry_msgs::Pose icp_pose;
	tf::StampedTransform gt_pose_tf;
	tf::StampedTransform icp_pose_tf;
	PointCloud_::Ptr merged_pc_;
	pcl::visualization::CloudViewer viewer_;
	


	

};

#endif // FRUIT_RGBD

