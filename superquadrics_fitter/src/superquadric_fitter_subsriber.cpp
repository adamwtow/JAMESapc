#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>


#include "tabletop_segmentation.h"
#include "impl/tabletop_segmentation.hpp"

#include "fit_superquadric_ceres.h"
#include "sample_superquadric_uniform.h"

#include <pcl/console/time.h>

/////////////////////////////////////////////////
//ROS Includes

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include "superquadric_fitter/estimateCapsicumPose.h"

using namespace pcl;

ros::Subscriber cloud_sub;
ros::Subscriber startFittingSub;
ros::Publisher cloud_pub;
ros::Publisher superquadric_pub;
std::string capsicum_frame_id;
std::string capsicumCloudTopic, capsicumModelTopic;
bool start_fitting_ = false;

boost::mutex cloud_input_mutex;
PointCloud<PointXYZ>::Ptr cloud_input;

//ros::Publisher pose_pub;

// Create the filtering object: downsample the dataset using a leaf size of 1cm
void downSample(pcl::PointCloud<PointXYZ>::Ptr input_cloud, pcl::PointCloud<PointXYZ>::Ptr output_cloud, float leafSize){

    //std::cout << "PointCloud before filtering has: " << input_cloud->points.size ()  << " data points." << std::endl; //*
    pcl::VoxelGrid<PointXYZ> vg;
//    output_cloud = PointCloud(new PointCloud);

    vg.setInputCloud (input_cloud);
    vg.setLeafSize (leafSize, leafSize, leafSize);
    vg.filter (*output_cloud);
    //std::cout << "PointCloud after filtering has: " << output_cloud->points.size ()  << " data points." << std::endl; //*
}

void publishTFfromPose(Eigen::Affine3d pose)
{
    static tf::TransformBroadcaster tf_broadcaster;

    tf::Transform transform;
    tf::transformEigenToTF(pose,transform);

    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", capsicum_frame_id));

}

void publishTFfromPose(Eigen::Affine3d pose, Eigen::Affine3d pose_pre_align)
{
    static tf::TransformBroadcaster tf_broadcaster;

    tf::Transform transform, transform_pre_align;
    tf::transformEigenToTF(pose,transform);
    tf::transformEigenToTF(pose_pre_align,transform_pre_align);

    tf_broadcaster.sendTransform(tf::StampedTransform(transform_pre_align*transform, ros::Time::now(), "/world", capsicum_frame_id));
}

void writeCloudtoMsg(PointCloud<PointXYZ>::Ptr cloud, sensor_msgs::PointCloud2::Ptr msg){

      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(*cloud, pcl_pc);
      pcl_conversions::fromPCL(pcl_pc,*msg);
}

void startFittingcallback(const std_msgs::Empty &){
    start_fitting_ = true;
}

bool estimateCapsicumPose(superquadric_fitter::estimateCapsicumPose::Request  &req,
                          superquadric_fitter::estimateCapsicumPose::Response &res){

    ROS_INFO_STREAM("Got Request, Starting to Estimate Capsicum Model, Input Cloud has " << cloud_input->size () << " points" << std::endl);

    pcl::PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    pcl::fromROSMsg(req.cloud,*cloud);

    //pcl::PointCloud<PointXYZ>::Ptr cloud_local(new PointCloud<PointXYZ>);

//    {
//        boost::lock_guard<boost::mutex> lock(cloud_input_mutex);
//        pcl::copyPointCloud(*cloud_input,*cloud);
//    }

    //downSample(cloud, cloud, 0.006);
    downSample(cloud, cloud, 0.006);

    pcl::console::TicToc timer;
    timer.tic ();

    double time = timer.toc ();

    bool preAlign = true;

    pcl::console::TicToc timer_fitting;
    timer_fitting.tic ();

    double min_fit = std::numeric_limits<double>::max ();
    sq::SuperquadricParameters<double> min_params;

    for (size_t i = 0; i < 1; ++i)
    {
        sq::SuperquadricFittingCeres<PointXYZ> fitting;
        fitting.setInputCloud (cloud);
        fitting.setPreAlign (preAlign, i);

        sq::SuperquadricParameters<double> params;
        double fit = fitting.fit (params);
        printf ("pre_align axis %d, fit %f\n", i, fit);

        if (fit < min_fit)
        {
            min_fit = fit;
            min_params = params;
        }
    }

    time = timer.toc ();
    PCL_ERROR ("1 fitted: %f\n", time);

    if (min_fit > std::numeric_limits<double>::max () - 100.){;;}
    else
    {

        sq::SuperquadricSampling<PointXYZ, double> sampling;
        sampling.setParameters (min_params);

        PolygonMesh mesh;
        sampling.generateMesh (mesh);
        ROS_INFO ("--- Displaying superquadric with error: %f and params: -e1 %f -e2 %f -a %f -b %f -c %f\n",
                min_fit, min_params.e1, min_params.e2, min_params.a, min_params.b, min_params.c);


        time = timer_fitting.toc ();
        ROS_ERROR ("Total fitting time: %f\n", time);

        ROS_INFO ("Superquadric fitting done. Press 'q' to quit.\n");

        tf::StampedTransform stamped_transform;
        Eigen::Affine3d kf_world_transform;
        static tf::TransformListener tf_listener;
        try{
            bool tf_success = tf_listener.waitForTransform("/world", "/kf_world",  ros::Time(0), ros::Duration(1));
            if (tf_success)   {
                tf_listener.lookupTransform("/world", "/kf_world", ros::Time(0), stamped_transform);
                tf::transformTFToEigen(stamped_transform, kf_world_transform);
            }

            sensor_msgs::PointCloud2 msg_out;
            pcl_conversions::fromPCL(mesh.cloud,msg_out);
            msg_out.header.stamp = ros::Time::now();
            msg_out.header.frame_id = "/kf_world";
            superquadric_pub.publish(msg_out);

            Eigen::Matrix<double,4,4> transform;
            if(preAlign) transform = kf_world_transform*min_params.pre_align_transform.inverse()*min_params.transform.inverse();
            else  transform = kf_world_transform*min_params.transform.inverse();

            Eigen::Affine3d pose(transform);
            geometry_msgs::Transform transform_msg;

            publishTFfromPose(pose);
            tf::transformEigenToMsg(pose,transform_msg);

            res.transform = transform_msg;
            res.a = min_params.a;
            res.b = min_params.b;
            res.c = min_params.c;
            res.e1 = min_params.e1;
            res.e2 = min_params.e2;

            ROS_INFO("Estimated Capsicum Model Sending Response");
            return true;

        }catch(tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          return false;
        }
    }

}


void pcl_callback(const sensor_msgs::PointCloud2 msg){

    {
        boost::lock_guard<boost::mutex> lock(cloud_input_mutex);
        pcl::fromROSMsg(msg,*cloud_input);
    }


//    if(start_fitting_ && cloud_input->size() > 3){

//    start_fitting_ = false;

//    ROS_INFO_STREAM("Starting to Estimate Capsicum Model, Input Cloud has " << cloud_input->size () << " points" << std::endl);

//    downSample(cloud_input, cloud_input, 0.006);

//    pcl::console::TicToc timer;
//    timer.tic ();

//    double time = timer.toc ();

//    bool preAlign = true;

//    pcl::console::TicToc timer_fitting;
//    timer_fitting.tic ();

//    double min_fit = std::numeric_limits<double>::max ();
//    sq::SuperquadricParameters<double> min_params;

//    for (size_t i = 0; i < 1; ++i)
//    {
//        sq::SuperquadricFittingCeres<PointXYZ> fitting;
//        fitting.setInputCloud (cloud_input);
//        fitting.setPreAlign (preAlign, i);

//        sq::SuperquadricParameters<double> params;
//        double fit = fitting.fit (params);
//        printf ("pre_align axis %d, fit %f\n", i, fit);

//        if (fit < min_fit)
//        {
//            min_fit = fit;
//            min_params = params;
//        }
//    }

//    time = timer.toc ();
//    PCL_ERROR ("1 fitted: %f\n", time);

//    if (min_fit > std::numeric_limits<double>::max () - 100.){;;}
//    else
//    {

//        sq::SuperquadricSampling<PointXYZ, double> sampling;
//        sampling.setParameters (min_params);

//        PolygonMesh mesh;
//        sampling.generateMesh (mesh);
//        ROS_INFO ("--- Displaying superquadric with error: %f and params: -e1 %f -e2 %f -a %f -b %f -c %f\n",
//                min_fit, min_params.e1, min_params.e2, min_params.a, min_params.b, min_params.c);

//        time = timer_fitting.toc ();
//        ROS_ERROR ("Total fitting time: %f\n", time);

//        ROS_INFO ("Superquadric fitting done. Press 'q' to quit.\n");

//        tf::StampedTransform stamped_transform;
//        Eigen::Affine3d kf_world_transform;
//        static tf::TransformListener tf_listener;
//        try{
//            bool tf_success = tf_listener.waitForTransform("/world", "/kf_world",  ros::Time(0), ros::Duration(1));
//            if (tf_success)   {
//                tf_listener.lookupTransform("/world", "/kf_world", ros::Time(0), stamped_transform);
//                tf::transformTFToEigen(stamped_transform, kf_world_transform);
//            }

//            sensor_msgs::PointCloud2 msg_out;
//            pcl_conversions::fromPCL(mesh.cloud,msg_out);
//            msg_out.header.stamp = ros::Time::now();
//            msg_out.header.frame_id = "/kf_world";
//            superquadric_pub.publish(msg_out);


//            Eigen::Matrix<double,4,4> transform;
//            if(preAlign) transform = kf_world_transform*min_params.pre_align_transform.inverse()*min_params.transform.inverse();
//            else  transform = kf_world_transform*min_params.transform.inverse();

//            Eigen::Affine3d pose(transform);
//            publishTFfromPose(pose);

//        }catch(tf::TransformException ex){
//          ROS_ERROR("%s",ex.what());
//          ros::Duration(1.0).sleep();
//        }
//    }

        //            sensor_msgs::PointCloud2::Ptr msg_out2(new sensor_msgs::PointCloud2);
        //            writeCloudtoMsg(cloud_input, msg_out2);
        //            msg_out2->header.stamp = ros::Time::now();
        //            msg_out2->header.frame_id = "/kf_world";
        //            cloud_pub.publish(msg_out2);
//  }

}



int
main (int argc, char **argv)
{
  ros::init(argc, argv, "superquadric_fitter");

  ros::NodeHandle nh("~");

  cloud_input = pcl::PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);


  if (!nh.getParam("capsicum_cloud_topic", capsicumCloudTopic))
  {
      capsicumCloudTopic = "/capsicum/points";
      ROS_INFO_STREAM("Listening for capsicum points on topic: " << capsicumCloudTopic);
  }
  if (!nh.getParam("capsicum_model_topic", capsicumModelTopic))
  {
      capsicumModelTopic = "/capsicum/model/points";
      ROS_INFO_STREAM("Publishing capsicum model on topic: " << capsicumModelTopic);
  }

  if (!nh.getParam("capsicum_frame_id", capsicum_frame_id)) capsicum_frame_id = "/capsicum_frame";

  std::string startFittingTopic;
  if (!nh.getParam("start_fitting_topic", startFittingTopic)) startFittingTopic = "/superquadric/estimate_pose";

  ros::ServiceServer service = nh.advertiseService("estimate_capsicum_pose", estimateCapsicumPose);

  cloud_sub = nh.subscribe(capsicumCloudTopic, 1, &pcl_callback);
  startFittingSub = nh.subscribe(startFittingTopic, 1, &startFittingcallback);
  superquadric_pub = nh.advertise<sensor_msgs::PointCloud2>(capsicumModelTopic, 1);
//  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/capsicum/pose",1);
  ros::spin();

  return (0);
}
