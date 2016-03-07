


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <vector>

#include "capsicum_segmentation.h"

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> SyncPolicyCloud;


//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

static const std::string OPENCV_WINDOW = "Image window";
const std::string cloudName = "Output";

  std::vector<ros::Publisher> pose_publishers;

HSV_model capsicum_model;
capsicum_detector capsicumDetector;

void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud){

      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  }


void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }

  sensor_msgs::ImagePtr cvImagetoMsg(cv::Mat &image,std::string encoding, std::string frame){

    static int header_sequence_id = 0;

    cv_bridge::CvImage* cv_msg_ptr = new cv_bridge::CvImage();
    std_msgs::Header header;

    header.seq = header_sequence_id++;
    header.stamp = ros::Time::now();
    header.frame_id = frame;

    cv_msg_ptr = new cv_bridge::CvImage(header,encoding,image);

    return cv_msg_ptr->toImageMsg();
}


//  void callback_cloud(const sensor_msgs::PointCloud2ConstPtr msgCloud){

//      PointCloud::Ptr cloud(new PointCloud);
//      readCloud(msgCloud,cloud);
//      cv::Mat rgb_image(cloud->height,cloud->width,CV_8UC3);
//      cv::Mat depth_image(cloud->height,cloud->width,CV_16UC1);

//      cv::Mat depthCropped, rgbCropped;

//      std::string frame_id = msgCloud->header.frame_id;




//      //depthCropped = depth_image(cv::Rect(0,40,640,400));
//      //rgbCropped = rgb_image(cv::Rect(0,40,640,400));

//      rgb_pub.publish(cvImagetoMsg(rgb_image,sensor_msgs::image_encodings::BGR8,"camera_rgb_optical_frame"));
//      depth_pub.publish(cvImagetoMsg(depth_image,sensor_msgs::image_encodings::TYPE_16UC1,frame_id));

//  }

//void callback_image(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth){

//    cv::Mat color,colorResized, colorCropped, colorRegistered, depth, depthCropped;

//    std::string depth_frame_id = imageDepth->header.frame_id;
//    std::string color_frame_id = imageColor->header.frame_id;

//    readImage(imageDepth, depth);
//    readImage(imageColor, color);

//    colorCropped = color(cv::Rect(150,0,1920-150,1080));
//    resize(colorCropped, colorResized, cv::Size(640,400), 0, 0, cv::INTER_LANCZOS4);
//    //colorRegistered = colorResized(cv::Rect(0,10,640,440));

//    depthCropped = depth(cv::Rect(0,40,640,400));

//    rgb_pub.publish(cvImagetoMsg(colorResized,sensor_msgs::image_encodings::BGR8,color_frame_id));
//    depth_pub.publish(cvImagetoMsg(depthCropped,sensor_msgs::image_encodings::TYPE_16UC1,depth_frame_id));
//}

  void addPosetoMarkerArray(ros::NodeHandle nh_, std::vector<visualization_msgs::MarkerArray> &marker_array_msgs, geometry_msgs::Pose pose, int id){

          visualization_msgs::Marker marker;
          visualization_msgs::MarkerArray marker_array;
          //static int i = 0;
          static int color_count = 0;
          static bool first = true;

          if(id == 0 || first){
              first = false;
              marker_array_msgs.push_back(marker_array);
              std::stringstream topicss;
              topicss << color_count;
              std::string pose_topic = "/icra/gripper_pose" + topicss.str();
              pose_publishers.push_back(nh_.advertise<visualization_msgs::MarkerArray>(pose_topic, 1 ));
          }
          else if(id%5 == 0){
            color_count++;
            marker_array_msgs.push_back(marker_array);
            std::stringstream topicss;
            topicss << color_count;
            std::string pose_topic = "/icra/gripper_pose" + topicss.str();
            pose_publishers.push_back(nh_.advertise<visualization_msgs::MarkerArray>(pose_topic, 1 ));
          }


          double colors[8][3] = {{1.0,0.0,0.0},
                                 {0.0,1.0,0.0},
                                 {0.0,0.0,1.0},
                                 {1.0,1.0,0.0},
                                 {0.0,1.0,1.0},
                                 {1.0,0.0,1.0},
                                 {1.0,0.0,0.0},
                                 {0.0,1.0,0.0}};


          marker.header.frame_id = "/world";
          marker.header.stamp = ros::Time();
          marker.ns = "grasp_poses";
          marker.id = id;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose = pose;
          marker.scale.x = 0.04;
          marker.scale.y = 0.0025;
          marker.scale.z = 0.0025;
          marker.color.a = 1.0;
          marker.color.r = colors[color_count][0];
          marker.color.g = colors[color_count][1];
          marker.color.b = colors[color_count][2];

//          marker.color.r = 0.0;
//          marker.color.g = 1.0;
//          marker.color.b = 1.0;

          marker_array_msgs.back().markers.push_back(marker);

  }


  bool extractData(const std::string &filename,
                   geometry_msgs::TransformStamped &kf_world_transform, geometry_msgs::TransformStamped &capsicum_transform,
                   geometry_msgs::TransformStamped &gripper_transform, sensor_msgs::PointCloud2 &capsicumModelCloud,
                   sensor_msgs::PointCloud2 &kinfuCloud)
  {
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);

    std::string kinfu_topic = "/ros_kinfu/depth_registered/points";
    std::string capsicum_topic = "/capsicum/model/points";

    bool found_capsicum_transform = false;
    bool found_capsicum_cloud = false;
    bool found_kinfu_cloud = false;
    bool found_gripper_transform = false;
    bool found_kf_world_transform = false;

    // Image topics to load
    std::vector<std::string> topics;
    topics.push_back(kinfu_topic);
    topics.push_back("/tf");
    topics.push_back(capsicum_topic);


    rosbag::View view(bag, rosbag::TopicQuery(topics));

       // Load all messages into our  dataset
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        if(m.getTopic() == "/tf"){
            tf2_msgs::TFMessage::ConstPtr transform = m.instantiate<tf2_msgs::TFMessage>();

            for(unsigned int i = 0; i < transform->transforms.size(); i++){

                if (transform->transforms[i].child_frame_id == "/capsicum_frame"){
                  capsicum_transform = transform->transforms[i];
                  found_capsicum_transform = true;
                }

                if (transform->transforms[i].child_frame_id == "/gripper_frame"){
                  gripper_transform = transform->transforms[i];
                  found_gripper_transform = true;
                }

                if (transform->transforms[i].child_frame_id == "kf_world"){
                  kf_world_transform = transform->transforms[i];
                  found_kf_world_transform = true;
                }
            }
        }


        if(m.getTopic() == capsicum_topic)
        {
            sensor_msgs::PointCloud2::ConstPtr capsicumCloud = m.instantiate<sensor_msgs::PointCloud2>();
            if (capsicumCloud != NULL) {
                found_capsicum_cloud = true;
                capsicumModelCloud = *capsicumCloud;
            }
        }


        if (m.getTopic() == kinfu_topic)
        {
            sensor_msgs::PointCloud2::ConstPtr kinfuCloudPtr = m.instantiate<sensor_msgs::PointCloud2>();
            if (kinfuCloudPtr != NULL){
                kinfuCloud = *kinfuCloudPtr;
                found_kinfu_cloud = true;
            }

        }

        if(found_capsicum_transform && found_capsicum_cloud && found_kinfu_cloud
                && found_gripper_transform && found_kf_world_transform)
        break;


    }


    return (found_capsicum_transform && found_capsicum_cloud && found_kinfu_cloud
        && found_gripper_transform && found_kf_world_transform);

    bag.close();
  }

  void writeCloudtoMsg(PointCloud::Ptr cloud, sensor_msgs::PointCloud2 &msg){

        pcl::PCLPointCloud2 pcl_pc;
        pcl::toPCLPointCloud2(*cloud, pcl_pc);
        pcl_conversions::fromPCL(pcl_pc,msg);
  }


  void segment_capsicum(sensor_msgs::PointCloud2 msg, sensor_msgs::PointCloud2 &msg_out){

      PointCloud::Ptr cloud(new PointCloud);
      PointCloud::Ptr cloud_segmented(new PointCloud);
      PointCloud::Ptr capsicum_cloud(new PointCloud);

      pcl::fromROSMsg(msg,*cloud);
      std::string input_frame_id = msg.header.frame_id;

      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
      capsicumDetector.segmentCloud(capsicum_model, cloud, cloud_segmented, 10.0, false);
      pcl_filter::downSample(cloud_segmented, cloud_segmented, 0.0025);
      std::vector<PointCloud::Ptr> clusters = pcl_filter::euclideanClusterRemoval(cloud_segmented, 0.003, 100, 250000);

      //Assume biggest cluster is capsicum
      int id = 0;
      for (std::vector<PointCloud::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it){
          std::string str_id = "cloud_" + id++;
          PointCloud::Ptr cloud = *it;
          if(cloud->size() > capsicum_cloud->size()) capsicum_cloud = cloud;
      }

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_capsicum_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl_filter::smooth_cloud(capsicum_cloud, smoothed_capsicum_cloud, 0.01);

      PointCloud::Ptr output_cloud(new PointCloud);
      copyPointCloud(*smoothed_capsicum_cloud,*output_cloud);

      //sensor_msgs::PointCloud2::Ptr msg_out(new sensor_msgs::PointCloud2);
      writeCloudtoMsg(output_cloud, msg_out);
      msg_out.header.stamp = ros::Time::now();
      msg_out.header.frame_id = input_frame_id;
      //pcl_pub.publish(msg_out);
  }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "icra_processing");
  ros::NodeHandle nh_("~");

  capsicum_model.hue_mean = 180.0; //183.198
  capsicum_model.saturation_mean = 1.0;//0.8323;//32.6;
  capsicum_model.value_mean = 0.3907;//115;
  capsicum_model.hue_var = 10*pow(4.3659,2); //650
  capsicum_model.saturation_var = 5*pow(0.1600,2); //412;
  capsicum_model.value_var = pow(0.1299,2); //641;

  capsicumDetector = capsicum_detector(capsicum_model);

  std::vector<sensor_msgs::PointCloud2> kinfuClouds, capsicumModelClouds, capsicumClouds;
  std::vector<geometry_msgs::TransformStamped> kf_world_transforms, capsicum_transforms, gripper_transforms;
  std::vector<geometry_msgs::PoseStamped> gripper_poses;

  std::vector<ros::Publisher> kinfu_pubs, capsicumModel_pubs, capsicumCloud_pubs;

  //ros::Publisher capsicumModel_pubs, capsicumCloud_pubs, pose_publisher;

  std::string kinfu_topic, capsicum_topic, capsicumModel_topic, pose_topic;


  std::vector<visualization_msgs::MarkerArray> marker_array_msgs;

  const char* args[] = {"-30_roll","30_roll", "-30_pitch","30_pitch", "-30_yaw","30_yaw"};
  std::vector<std::string> pose_type_str(args, args + 6);

  int count = 0;
  for(int pose_type = 0; pose_type < 6; pose_type++)
  {
    for(int trial = 1; trial < 6; trial++)
    {

        char filename[50];
        geometry_msgs::TransformStamped nextKfWorldTransform;
        geometry_msgs::TransformStamped nextCapsicumTransform;
        geometry_msgs::TransformStamped nextGripperTransform;
        geometry_msgs::PoseStamped nextGripperPose;

        sensor_msgs::PointCloud2 nextCapsicumModelCloud;
        sensor_msgs::PointCloud2 nextKinfuCloud;
        sensor_msgs::PointCloud2 nextcapsicumCloud;


//        sprintf(filename,"/home/chris/icra2016_bags/trial%d_%s.bag",trial,pose_type_str[pose_type].c_str());

        sprintf(filename,"/home/harvey/grasp_experiment/trials/trial%d_%s.bag",trial,pose_type_str[pose_type].c_str());

        ROS_INFO_STREAM("Processing Bag File: " << filename);

        if(extractData(filename, nextKfWorldTransform, nextCapsicumTransform, nextGripperTransform, nextCapsicumModelCloud, nextKinfuCloud))
        {

          ROS_INFO_STREAM("Success, found all messages");

          std::stringstream ss;
          ss << count;

          nextKfWorldTransform.child_frame_id += ss.str();
          //nextCapsicumTransform.header.frame_id += ss.str();
          nextCapsicumTransform.child_frame_id += ss.str();
          nextGripperTransform.child_frame_id += ss.str();
          nextCapsicumModelCloud.header.frame_id += ss.str();
          nextKinfuCloud.header.frame_id += ss.str();




          tf::Transform translation, translation2, transform, newGripperTransform;
          translation.setIdentity(); translation2.setIdentity();

          tf::Vector3 offset(0.17, 0, 0.05); tf::Vector3 offset2(-0.01, 0.04, 0.0);
          translation.setOrigin(offset); translation2.setOrigin(offset2);
          tf::transformMsgToTF(nextGripperTransform.transform,transform);

          //if((pose_type_str[pose_type] == "-30_yaw") && (trial == 2 || trial >= 4) ){
          if((pose_type_str[pose_type] == "-30_yaw") && (trial >= 4) ){
                newGripperTransform = translation2*transform*translation;
                nextKfWorldTransform.transform.translation.y += 0.04;
                nextCapsicumTransform.transform.translation.y += 0.04;
                nextGripperTransform.transform.translation.y += 0.04;

                nextKfWorldTransform.transform.translation.x -= 0.01;
                nextCapsicumTransform.transform.translation.x -= 0.01;
                nextGripperTransform.transform.translation.x -= 0.01;

          }else{
                newGripperTransform = transform*translation;
          }

          nextGripperPose.header.frame_id = nextGripperTransform.header.frame_id;
          tf::quaternionTFToMsg(newGripperTransform.getRotation(),nextGripperPose.pose.orientation);
          nextGripperPose.pose.position.x = newGripperTransform.getOrigin().x();
          nextGripperPose.pose.position.y = newGripperTransform.getOrigin().y();
          nextGripperPose.pose.position.z = newGripperTransform.getOrigin().z();


          kf_world_transforms.push_back(nextKfWorldTransform);
          capsicum_transforms.push_back(nextCapsicumTransform);
          gripper_transforms.push_back(nextGripperTransform);
          capsicumModelClouds.push_back(nextCapsicumModelCloud);
          kinfuClouds.push_back(nextKinfuCloud);

          segment_capsicum(nextKinfuCloud, nextcapsicumCloud);
          capsicumClouds.push_back(nextcapsicumCloud);


  //        capsicum_grasp_translation = Eigen::Translation3d(Eigen::Vector3d(-gripper_offset_in,0,0));
  //        capsicum_grasp_transform = capsicum_grasp_transform*capsicum_grasp_translation;
  //        tf::poseEigenToMsg(capsicum_grasp_transform,capsicum_grasp_pose.pose);

          addPosetoMarkerArray(nh_, marker_array_msgs,nextGripperPose.pose, count);
          gripper_poses.push_back(nextGripperPose);

          std::stringstream topicss;
          topicss << count;

          kinfu_topic = "/icra/kinfu/points" + topicss.str();
          capsicum_topic = "/icra/capsicum/points"  + topicss.str();
          capsicumModel_topic = "/icra/capsicum/model/points"  + topicss.str();


          kinfu_pubs.push_back(nh_.advertise<sensor_msgs::PointCloud2>(kinfu_topic, 1 ));
          capsicumModel_pubs.push_back(nh_.advertise<sensor_msgs::PointCloud2>(capsicumModel_topic, 1 ));
          capsicumCloud_pubs.push_back(nh_.advertise<sensor_msgs::PointCloud2>(capsicum_topic, 1 ));
          //pose_publishers.push_back(nh_.advertise<geometry_msgs::PoseStamped>(pose_topic, 1 ));

          //kinfu_pubs.push_back(nh_.advertise<sensor_msgs::PointCloud2>(kinfu_topic, 1 ));



        }else{
            ROS_ERROR_STREAM("Fail, did not find all messages for bag file: " << filename);
        }

        count++;
    }
  }

//  capsicum_topic = "/icra/capsicum/points";
//  capsicumModel_topic = "/icra/capsicum/model/points";
//  pose_topic = "/icra/gripper_pose";


//  capsicumModel_pubs = nh_.advertise<sensor_msgs::PointCloud2>(capsicumModel_topic, 1 );
//  capsicumCloud_pubs = nh_.advertise<sensor_msgs::PointCloud2>(capsicum_topic, 1 );
//  pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic, 1 );

  tf::TransformBroadcaster tf;
  ros::Rate loop_rate(10);



  while(ros::ok())
  {
      for(unsigned int i = 0; i < kf_world_transforms.size(); i++){

          capsicumModelClouds[i].header.stamp = ros::Time::now();
          kinfuClouds[i].header.stamp = ros::Time::now();
          capsicumClouds[i].header.stamp = ros::Time::now();

          gripper_transforms[i].header.stamp = ros::Time::now();
          capsicum_transforms[i].header.stamp = ros::Time::now();
          kf_world_transforms[i].header.stamp = ros::Time::now();
          gripper_poses[i].header.stamp = ros::Time::now();


          tf.sendTransform(gripper_transforms[i]);
          tf.sendTransform(capsicum_transforms[i]);
          tf.sendTransform(kf_world_transforms[i]);


          kinfu_pubs[i].publish(kinfuClouds[i]);
          capsicumModel_pubs[i].publish(capsicumModelClouds[i]);
          capsicumCloud_pubs[i].publish(capsicumClouds[i]);
          //pose_publishers[i].publish(gripper_poses[i]);
          //pose_publishers.publish(marker_array_msgs);
      }

      for(unsigned int i = 0; i < marker_array_msgs.size(); i++)
        pose_publishers[i].publish(marker_array_msgs[i]);



      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
