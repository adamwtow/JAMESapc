#include "ros_kinfu_ls.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu::kinfuLS;
using pcl::gpu::DeviceArray;
using pcl::gpu::DeviceArray2D;
using pcl::gpu::PtrStepSz;

using pcl::device::kinfuLS::device_cast;

using namespace Eigen;
namespace pc = pcl::console;

  ros_kinfu_ls::ros_kinfu_ls() : it_(nh_), exit_ (false), scan_ (false), scan_mesh_(false),
      scan_volume_ (false), scene_cloud_view_(), was_lost_(false), image_view_(), independent_camera_ (false), pcd_source_ (false), time_ms_(0), kinfu_publisher(nh_)

  {
    float volume_size = pcl::device::kinfuLS::VOLUME_SIZE;
    float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
    int snapshot_rate = pcl::device::kinfuLS::SNAPSHOT_RATE;

    Eigen::Vector3f volume_size_vector = Vector3f::Constant (volume_size/*meters*/);

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size_vector, shift_distance, 480, 640);

    if (!nh_.getParam("depth_image_topic", topicDepth))
    {
        topicDepth = "/camera/depth/image_registered";
        ROS_INFO_STREAM("Listening for depth on topic: " << topicDepth);
    }
    if (!nh_.getParam("rgb_image_topic", topicColor))
    {
        topicColor = "/camera/rgb/image_registered";
        ROS_INFO_STREAM("Listening for colour on topic: " << topicColor);
    }

    if (!nh_.getParam("camera_info_topic", topicCameraInfo))
    topicCameraInfo = "/camera/depth/camera_info";

    if (!nh_.getParam("kinfu_reset_topic", topicReset))
    topicReset = "/ros_kinfu/reset";


    if (!nh_.getParam("publish_points", publish_)) publish_ = false;
    if (!nh_.getParam("visualise", viz_)) viz_ = true;
    if (!nh_.getParam("use_hints", use_hints_)) use_hints_ = false;
    if (!nh_.getParam("registration", registration_)) registration_ = false;
    if (!nh_.getParam("integrate_colors", integrate_colors_)) integrate_colors_ = true;
    if (!nh_.getParam("update_kinect_world_frame", update_kinect_world_)) update_kinect_world_ = false;


    double vsz;
    if (!nh_.getParam("volume_size", vsz)) vsz = 0.5;

    if (!nh_.getParam("camera_fx", fx_)) fx_ =  463.888885f;
    if (!nh_.getParam("camera_fy", fy_)) fy_ =  463.888885f;
    if (!nh_.getParam("camera_cx", cx_)) cx_ = 320.0f;
    if (!nh_.getParam("camera_cy", cy_)) cy_ = 240.0f;

    double tsdf_trunc_,icp_weight,cam_move_threshold;
    if (!nh_.getParam("tsdf_trunc", tsdf_trunc_)) tsdf_trunc_ = 0.005f;
    if (!nh_.getParam("icp_weight", icp_weight)) icp_weight = 0.01f;
    if (!nh_.getParam("camera_movement_threshold", cam_move_threshold)) cam_move_threshold = 0.001f;

    int height, width;
    if (!nh_.getParam("image_height", height)) height = 480;
    if (!nh_.getParam("image_width", width)) width = 640;

    int device;
    if (!nh_.getParam("gpu_device", device)) device = 0;

    //kinfu_ = new KinfuTracker(height,width);
//    scene_cloud_view_ = new SceneCloudView(viz_);
//    image_view_= new ImageView(viz_);

    pcl::gpu::setDevice (device);
    pcl::gpu::printShortCudaDeviceInfo (device);


    //Init Kinfu Tracker
    //Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);
    //kinfu_->volume().setSize (volume_size);
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();
    Eigen::Vector3f t = volume_size_vector * 0.5f - Vector3f (0, 0, volume_size_vector (2) / 2);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    Eigen::Matrix3f Rid = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f T = Vector3f (0, 0, -volume_size_vector(0)*1.5f);
    delta_lost_pose_ = Eigen::Translation3f (T) * Eigen::AngleAxisf (Rid);


    kinfu_->setInitialCameraPose (pose);

    kinfu_->volume().setTsdfTruncDist (tsdf_trunc_/*meters*/);
    kinfu_->setIcpCorespFilteringParams (icp_weight/*meters*/, sin ( pcl::deg2rad(20.f) ));
    kinfu_->setCameraMovementThreshold(cam_move_threshold);

//    if (!icp)
//      kinfu_->disableIcp();

    //Init KinfuApp
    tsdf_cloud_ptr_ = PointCloudTSDF::Ptr (new PointCloudTSDF);
    image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_->rows (), kinfu_->cols (), fx_, fy_, cx_, cy_) );

    if (viz_)
    {
        //scene_cloud_view_.cloud_viewer_.registerKeyboardCallback (&ros_kinfu_ls::keyboard_callback, *this, (void*)this);
        //image_view_.viewerScene_.registerKeyboardCallback (&ros_kinfu_ls::keyboard_callback, *this, (void*)this);
        //image_view_.viewerDepth_.registerKeyboardCallback (&ros_kinfu_ls::keyboard_callback, *this, (void*)this);
        //image_view_.viewerDepth_->registerKeyboardCallback (keyboard_callback, (void*)this);
        scene_cloud_view_.toggleCube(volume_size_vector);
    }

    kinfu_->setDepthIntrinsics(fx_, fy_, cx_, cy_);

    if(integrate_colors_){
      const int max_color_integration_weight = 2;
      kinfu_->initColorIntegration(max_color_integration_weight);
    }

    colorImageSubscriberFilter = new image_transport::SubscriberFilter(it_, topicColor, 1);
    depthImageSubscriberFilter = new image_transport::SubscriberFilter(it_, topicDepth, 1);
    cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, topicCameraInfo, 1);

    resetSubscriber = nh_.subscribe(topicReset, 1,&ros_kinfu_ls::resetCallback,this);

    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(2), *colorImageSubscriberFilter, *depthImageSubscriberFilter);
  }


  ros_kinfu_ls::~ros_kinfu_ls()
  {
    delete &kinfu_;
    delete &depth_device_;
    delete &generated_depth_;
    delete &scene_cloud_view_;
    delete &image_view_;
    //delete &tsdf_volume_;
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void ros_kinfu_ls::initCurrentFrameView ()
  {
    current_frame_cloud_view_ = boost::shared_ptr<CurrentFrameCloudView>(new CurrentFrameCloudView ());
    //current_frame_cloud_view_->cloud_viewer_.registerKeyboardCallback (&ros_kinfu_ls::keyboard_callback, *this, (void*)this);
    current_frame_cloud_view_->setViewerPose (kinfu_->getCameraPose ());
  }

  void ros_kinfu_ls::initRegistration ()
  {
    registration_ = true;
  }

  void ros_kinfu_ls::setDepthIntrinsics(){

      kinfu_->setDepthIntrinsics(fx_, fy_, cx_, cy_);
      cout << "Depth intrinsics changed to fx="<< fx_ << " fy=" << fy_ << " cx=" << cx_ << " cy=" << cy_ << endl;
  }

  void ros_kinfu_ls::setDepthIntrinsics(std::vector<float> depth_intrinsics)
  {
    float fx = depth_intrinsics[0];
    float fy = depth_intrinsics[1];

    if (depth_intrinsics.size() == 4)
    {
        float cx = depth_intrinsics[2];
        float cy = depth_intrinsics[3];
        kinfu_->setDepthIntrinsics(fx, fy, cx, cy);
        cout << "Depth intrinsics changed to fx="<< fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << endl;
    }
    else {
        kinfu_->setDepthIntrinsics(fx, fy);
        cout << "Depth intrinsics changed to fx="<< fx << " fy=" << fy << endl;
    }
  }

  void ros_kinfu_ls::toggleColorIntegration()
  {
    if(registration_)
    {
        const int max_color_integration_weight = 2;
        kinfu_->initColorIntegration(max_color_integration_weight);
        integrate_colors_ = true;
    }
    cout << "Color integration: " << (integrate_colors_ ? "On" : "Off ( requires registration mode )") << endl;
  }

  void ros_kinfu_ls::enableTruncationScaling()
  {
    kinfu_->volume().setTsdfTruncDist (kinfu_->volume().getSize()(0) / 100.0f);
  }

  void ros_kinfu_ls::toggleIndependentCamera()
  {
    independent_camera_ = !independent_camera_;
    cout << "Camera mode: " << (independent_camera_ ?  "Independent" : "Bound to Kinect pose") << endl;
  }

  void ros_kinfu_ls::togglePublisher()
  {
    publish_ = !publish_;
    cout << "TSDF Publisher: " << (publish_ ?  "TSDF Publisher On" : "TSDF Publisher Off ") << endl;
  }


void ros_kinfu_ls::resetCallback(const std_msgs::Empty & /*msg*/)
{
  reset_command_ = true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  void ros_kinfu_ls::execute(const sensor_msgs::Image::ConstPtr bgr, const sensor_msgs::Image::ConstPtr depth)
  {
    bool has_image = false;
    bool tf_success = false;
    Affine3f *pose_hint = NULL;
    cv::Mat rgb;

    was_lost_ = kinfu_->icpIsLost();

    if(reset_command_){
        kinfu_->reset();
        reset_command_ = false;
        ROS_INFO("KinFu was reset.");
    }

    depth_device_.upload (&(depth->data[0]), depth->step, depth->height, depth->width);

    if (integrate_colors_){

        readImageRGB(bgr,rgb);
        image_view_.colors_device_.upload (rgb.data, rgb.step, rgb.rows, rgb.cols);
    }

    tf::StampedTransform stamped_transform, kinect_transform;
    if(use_hints_){
        try{
            cout << "Waiting for Transform" << endl;
            tf_success = tf_listener.waitForTransform("/world", "/camera_depth_optical_frame",  ros::Time(0), ros::Duration(0.1));
            if (tf_success)   {
                tf_listener.lookupTransform("/world", "/camera_depth_optical_frame", ros::Time(0), stamped_transform);
                tf_listener.lookupTransform(kinfu_publisher.kf_world_frame, "/camera_depth_optical_frame", ros::Time(0), kinect_transform);
                Affine3d kinect_camera_pose_hint;
                //tf::Transform kinect_camera_transform;
                //kinect_camera_transform = stamped_transform.inverse()*kinfu_publisher.kf_world_transform;
                tf::transformTFToEigen(kinect_transform,kinect_camera_pose_hint);

                pose_hint = new Affine3f(kinect_camera_pose_hint);

                cout << "Found Pose" << endl;
            }
                //kinfu_publisher.updateKinectWorldTransform(stamped_transform,kinfu_->getCameraPose());
        }catch(tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        //run kinfu algorithm
        if (integrate_colors_)
            ;//has_image = kinfu_->operator()(depth_device_, image_view_.colors_device_, pose_hint);
        else
            has_image = kinfu_->operator()(depth_device_);

    }else{

        //cout << "Not using hints" << endl;

        //run kinfu algorithm
        if (integrate_colors_)
            has_image = kinfu_->operator()(depth_device_, image_view_.colors_device_);
        else
            has_image = kinfu_->operator()(depth_device_);

    }

    if(viz_){
        image_view_.showDepth(depth);

        if (current_frame_cloud_view_)
          current_frame_cloud_view_->show (*kinfu_);


        // if ICP is lost, we show the world from a farther view point
        if(kinfu_->icpIsLost())
        {
          kinfu_viz::setViewerPose (scene_cloud_view_.cloud_viewer_, kinfu_->getCameraPose () * delta_lost_pose_);
        }
        else
        if (!independent_camera_)
          kinfu_viz::setViewerPose (scene_cloud_view_.cloud_viewer_, kinfu_->getCameraPose ());

//        if (enable_texture_extraction_ && !kinfu_->icpIsLost ()) {
//          if ( (frame_counter_  % snapshot_rate_) == 0 )   // Should be defined as a parameter. Done.
//          {
//            screenshot_manager_.saveImage (kinfu_->getCameraPose (), rgb24);
//          }
//        }
        // display ICP state
        scene_cloud_view_.displayICPState (*kinfu_, was_lost_);


    }

    // process camera pose
    //if (pose_processor_)
    //{
    //    pose_processor_->processPose (kinfu_->getCameraPose ());
    //}

    // update kinect world transform
    if(update_kinect_world_) kinfu_publisher.updateKinectWorldTransform(stamped_transform,kinfu_->getCameraPose());



    if(publish_){
        //if(new_tsdf_) kinfu_publisher.publishTSDFCloud(tsdf_cloud_ptr_);
        kinfu_publisher.publishCameraPose(kinfu_->getCameraPose());
        kinfu_publisher.publishTFfromPose(kinfu_->getCameraPose());
        kinfu_publisher.publishKfWorldTransform();
        //image_view_.generateDepth(kinfu_, kinfu_->getCameraPose(),generated_depth_);
        //kinfu_publisher.publishDepth(generated_depth_);

        //scene_cloud_view_.generateCloud(kinfu_,integrate_colors_);
        if(integrate_colors_){
            kinfu_publisher.publishCloud(scene_cloud_view_.cloud_ptr_);
            //kinfu_publisher.publishColorCloud(scene_cloud_view_.combined_color_ptr_);
        }else{
            kinfu_publisher.publishCloud(scene_cloud_view_.cloud_ptr_);
        }

    }

  }


  void ros_kinfu_ls::readImageRGB(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, sensor_msgs::image_encodings::RGB8);
        pCvImage->image.copyTo(image);
    }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void ros_kinfu_ls::startMainLoop ()
    {

        tf::StampedTransform stamped_transform;
        try{
            bool success = tf_listener.waitForTransform("/world", "/camera_depth_optical_frame",  ros::Time(0), ros::Duration(1));
            if (success)   {
              tf_listener.lookupTransform("/world", "/camera_depth_optical_frame", ros::Time(0), stamped_transform);
              kinfu_publisher.updateKinectWorldTransform(stamped_transform,kinfu_->getCameraPose());
              kinfu_publisher.publishKfWorldTransform();
            }

        }catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        sync->registerCallback(boost::bind(&ros_kinfu_ls::execute, this, _1, _2));  // Start subscriber

        bool scene_view_not_stopped= viz_ ? !scene_cloud_view_.cloud_viewer_.wasStopped () : true;
        bool image_view_not_stopped= viz_ ? !image_view_.viewerScene_.wasStopped () : true;

        while (!exit_ && scene_view_not_stopped && image_view_not_stopped)
        {
            if (viz_)
                scene_cloud_view_.cloud_viewer_.spinOnce (3);
            ros::spin();
        }

  }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  void ros_kinfu_ls::printHelp ()
//  {
//    cout << endl;
//    cout << "KinFu app hotkeys" << endl;
//    cout << "=================" << endl;
//    cout << "    H    : print this help" << endl;
//    cout << "   Esc   : exit" << endl;
//    cout << "    T    : take cloud" << endl;
//    cout << "    A    : take mesh" << endl;
//    cout << "    M    : toggle cloud exctraction mode" << endl;
//    cout << "    N    : toggle normals exctraction" << endl;
//    cout << "    I    : toggle independent camera mode" << endl;
//    cout << "    B    : toggle volume bounds" << endl;
//    cout << "    *    : toggle scene view painting ( requires registration mode )" << endl;
//    cout << "    C    : clear clouds" << endl;
//    cout << "   1,2,3 : save cloud to PCD(binary), PCD(ASCII), PLY(ASCII)" << endl;
//    cout << "    7,8  : save mesh to PLY, VTK" << endl;
//    cout << "   X, V  : TSDF volume utility" << endl;
//    cout << endl;
//  }
    void ros_kinfu_ls::printHelp ()
    {
      cout << endl;
      cout << "KinFu app hotkeys" << endl;
      cout << "=================" << endl;
      cout << "    H    : print this help" << endl;
      cout << "   Esc   : exit" << endl;
      cout << "    T    : take cloud" << endl;
      cout << "    A    : take mesh" << endl;
      cout << "    M    : toggle cloud exctraction mode" << endl;
      cout << "    N    : toggle normals exctraction" << endl;
      cout << "    I    : toggle independent camera mode" << endl;
      cout << "    B    : toggle volume bounds" << endl;
      cout << "    *    : toggle scene view painting ( requires registration mode )" << endl;
      cout << "    C    : clear clouds" << endl;
      cout << "   1,2,3 : save cloud to PCD(binary), PCD(ASCII), PLY(ASCII)" << endl;
      cout << "    7,8  : save mesh to PLY, VTK" << endl;
      cout << "   X, V  : TSDF volume utility" << endl;
      cout << "   L, l  : On the next shift, KinFu will extract the whole current cube, extract the world and stop" << endl;
      cout << "   S, s  : On the next shift, KinFu will extract the world and stop" << endl;
      cout << endl;
    }




//  void keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
//  {
//    ros_kinfu_ls* app = reinterpret_cast<ros_kinfu_ls*> (cookie);

//    int key = e.getKeyCode ();

//    if (e.keyUp ())
//      switch (key)
//      {
//      case 27: app->exit_ = true; break;
//      case (int)'t': case (int)'T': app->scan_ = true; break;
//      case (int)'a': case (int)'A': app->scan_mesh_ = true; break;
//      case (int)'h': case (int)'H': app->printHelp (); break;
//      case (int)'m': case (int)'M': app->scene_cloud_view_.toggleExtractionMode (); break;
//      case (int)'n': case (int)'N': app->scene_cloud_view_.toggleNormals (); break;
//      case (int)'c': case (int)'C': app->scene_cloud_view_.clearClouds (true); break;
//      case (int)'i': case (int)'I': app->toggleIndependentCamera (); break;
//      case (int)'b': case (int)'B': app->scene_cloud_view_.toggleCube(app->kinfu_->volume().getSize()); break;
//      case (int)'p': case (int)'P': app->togglePublisher(); break;

////      case (int)'7': case (int)'8': app->writeMesh (key - (int)'0'); break;
////      case (int)'1': case (int)'2': case (int)'3': app->writeCloud (key - (int)'0'); break;
//      case '*': app->image_view_.toggleImagePaint (); break;

//      case (int)'x': case (int)'X':
//        app->scan_volume_ = !app->scan_volume_;
//        cout << endl << "Volume scan: " << (app->scan_volume_ ? "enabled" : "disabled") << endl << endl;
//        break;
//      case (int)'v': case (int)'V':
//        cout << "Saving TSDF volume to tsdf_volume.dat ... " << flush;
//        //app->tsdf_volume_.save ("tsdf_volume.dat", true);
//        //cout << "done [" << app->tsdf_volume_.size () << " voxels]" << endl;
//        cout << "Saving TSDF volume cloud to tsdf_cloud.pcd ... " << flush;
//        pcl::io::savePCDFile<pcl::PointXYZI> ("tsdf_cloud.pcd", *app->tsdf_cloud_ptr_, true);
//        cout << "done [" << app->tsdf_cloud_ptr_->size () << " points]" << endl;
//        break;

//      default:
//        break;
//      }
//  }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ros_kinfu_ls");
  //ros::NodeHandle nh("~");

  //boost::shared_ptr<CameraPoseProcessor> pose_processor;

  ros_kinfu_ls ros_kinfu_app;

  //ros_kinfu_->initRegistration();
  //ros_kinfu_->setDepthIntrinsics();
  //ros_kinfu_->toggleColorIntegration();
  // executing
  try { ros_kinfu_app.startMainLoop(); }
  catch (const pcl::PCLException& /*e*/) { cout << "PCLException" << endl; }
  catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
  catch (const std::exception& /*e*/) { cout << "Exception" << endl; }


  delete &ros_kinfu_app;

  return 0;
}

