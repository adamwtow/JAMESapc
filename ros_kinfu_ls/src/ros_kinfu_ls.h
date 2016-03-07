#ifndef ROS_KINFU_LS_H
#define ROS_KINFU_LS_H

#define _CRT_SECURE_NO_DEPRECATE


///////////////////////////////////////////////////////////
// ROS



#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Empty.h>
#include <ros/spinner.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/subscriber_filter.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>

///////////////////////////////////////////////////////////////////
//Standard

#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>

///////////////////////////////////////////////////////////////////
//PCL

#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/exceptions.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/angles.h>


#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
//#include <pcl/gpu/kinfu_large_scale/tsdf_volume.hpp>
//#include <pcl/gpu/kinfu_large_scale/camera_pose.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu_large_scale/internal.h>

/////////////////////////////////////////////////////////////////////
//OPENCV

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>

/////////////////////////////////////////////////////////////////////
//LOCAL

#include "kinfu_ls_viz_tools.h"
#include "kinfu_ls_tools.h"
#include "ros_kinfu_ls_publisher.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu::kinfuLS;
using namespace Eigen;

//typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
//typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZI PointTSDF;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointTSDF> PointCloudTSDF;

//static void keyboard_callback (const pcl::visualization::KeyboardEvent &e, void *cookie);

class ros_kinfu_ls
{
public:
    enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };


    ros_kinfu_ls();
    ~ros_kinfu_ls();

    void initCurrentFrameView ();
    void initRegistration ();
    void setDepthIntrinsics();
    void setDepthIntrinsics(std::vector<float> depth_intrinsics);

    void toggleColorIntegration();
    void enableTruncationScaling();
    void toggleIndependentCamera();
    void togglePublisher();

    void resetCallback(const std_msgs::Empty & /*msg*/);

    void execute(const sensor_msgs::Image::ConstPtr rgb,const sensor_msgs::Image::ConstPtr depth, const sensor_msgs::CameraInfo::ConstPtr cameraInfo);
    void execute(const sensor_msgs::Image::ConstPtr rgb,const sensor_msgs::Image::ConstPtr depth);
    void execute(const sensor_msgs::Image::ConstPtr& depth);
    void startMainLoop();

    void writeCloud (int format) const;
    void writeMesh(int format) const;
    void printHelp ();

    void publishTSDFCloud();
    void publishCameraPose();

    void readImageRGB(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //Kinfu Member Variables

    bool exit_, scan_,scan_mesh_,scan_volume_, independent_camera_, registration_,
    integrate_colors_, pcd_source_;

    bool use_hints_, update_kinect_world_;
    bool was_lost_;

    KinfuTracker *kinfu_;
    KinfuTracker::DepthMap depth_device_;
    KinfuTracker::DepthMap generated_depth_;

    SceneCloudView scene_cloud_view_;
    ImageView image_view_;
    boost::shared_ptr<CurrentFrameCloudView> current_frame_cloud_view_;

    //pcl::gpu::kinfuLS::TsdfVolume<float, short> tsdf_volume_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    std::vector<KinfuTracker::PixelRGB> source_image_data_;
    std::vector<unsigned short> source_depth_data_;

    int time_ms_, icp_, viz_;

    //Depth Intrinsic parameters
    double fx_,fy_,cx_,cy_;

    //boost::shared_ptr<CameraPoseProcessor> pose_processor_;

    Eigen::Affine3f delta_lost_pose_;
///////////////////////////////////////////////////////////////////////////////////////////////////

    //ROS Member Variables
    bool subsriber_start_, publish_;

    std::string topicDepth, topicColor, topicCameraInfo,topicReset ;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter *colorImageSubscriberFilter, *depthImageSubscriberFilter;
    image_transport::Subscriber depthImageSubscriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *cameraInfoSubscriber;
    message_filters::Synchronizer<SyncPolicy> *sync;
    ros::Subscriber resetSubscriber;

    Affine3f reverse_initial_pose;
    tf::TransformListener tf_listener;

    ros_kinfu_ls_publisher kinfu_publisher;

    bool reset_command_ = false;

    static void keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
    {
      cout << "Got Keyboard Key" << endl;
      ros_kinfu_ls* app = reinterpret_cast<ros_kinfu_ls*> (cookie);

      int key = e.getKeyCode ();

      if (e.keyUp ())
        switch (key)
        {
        case 27: app->exit_ = true; break;
        case (int)'t': case (int)'T': app->scan_ = true; break;
        case (int)'a': case (int)'A': app->scan_mesh_ = true; break;
        case (int)'h': case (int)'H': app->printHelp (); break;
        case (int)'m': case (int)'M': app->scene_cloud_view_.toggleExtractionMode (); break;
        case (int)'n': case (int)'N': app->scene_cloud_view_.toggleNormals (); break;
        case (int)'c': case (int)'C': app->scene_cloud_view_.clearClouds (true); break;
        case (int)'i': case (int)'I': app->toggleIndependentCamera (); break;
        case (int)'b': case (int)'B': app->scene_cloud_view_.toggleCube(app->kinfu_->volume().getSize()); break;
        case (int)'l': case (int)'L': app->kinfu_->performLastScan (); break;
        case (int)'s': case (int)'S': app->kinfu_->extractAndSaveWorld (); break;
        case (int)'r': case (int)'R': app->kinfu_->reset(); app->scene_cloud_view_.clearClouds(); break;
        //case (int)'7': case (int)'8': app->writeMesh (key - (int)'0'); break;
        //case (int)'1': case (int)'2': case (int)'3': app->writeCloud (key - (int)'0'); break;
        case '*': app->image_view_.toggleImagePaint (); break;

        case (int)'p': case (int)'P': app->kinfu_->setDisableICP(); break;

        case (int)'x': case (int)'X':
          app->scan_volume_ = !app->scan_volume_;
          cout << endl << "Volume scan: " << (app->scan_volume_ ? "enabled" : "disabled") << endl << endl;
          break;
        case (int)'v': case (int)'V':
          cout << "Saving TSDF volume to tsdf_volume.dat ... " << flush;
          // app->tsdf_volume_.save ("tsdf_volume.dat", true);
          app->kinfu_->volume ().save ("tsdf_volume.dat", true);
          // cout << "done [" << app->tsdf_volume_.size () << " voxels]" << endl;
          cout << "done [" << app->kinfu_->volume ().size () << " voxels]" << endl;
          cout << "Saving TSDF volume cloud to tsdf_cloud.pcd ... " << flush;
          pcl::io::savePCDFile<pcl::PointXYZI> ("tsdf_cloud.pcd", *app->tsdf_cloud_ptr_, true);
          cout << "done [" << app->tsdf_cloud_ptr_->size () << " points]" << endl;
          break;
        default:
          break;
        }
    }



};

#endif // ROS_KINFU_H
