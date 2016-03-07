#include "kinfu_ls_viz_tools.h"
#include "kinfu_ls_tools.h"

using namespace pcl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace kinfu_viz
{

    vector<string> getPcdFilesInDir(const string& directory)
    {
      namespace fs = boost::filesystem;
      fs::path dir(directory);

      std::cout << "path: " << directory << std::endl;
      if (directory.empty() || !fs::exists(dir) || !fs::is_directory(dir))
        PCL_THROW_EXCEPTION (pcl::IOException, "No valid PCD directory given!\n");

      vector<string> result;
      fs::directory_iterator pos(dir);
      fs::directory_iterator end;

      for(; pos != end ; ++pos)
        if (fs::is_regular_file(pos->status()) )
          if (fs::extension(*pos) == ".pcd")
          {
    #if BOOST_FILESYSTEM_VERSION == 3
            result.push_back (pos->path ().string ());
    #else
            result.push_back (pos->path ());
    #endif
            cout << "added: " << result.back() << endl;
          }

      return result;
    }

    void setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
    {
        Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
        Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
        Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
        viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                                  look_at_vector[0], look_at_vector[1], look_at_vector[2],
                                  up_vector[0], up_vector[1], up_vector[2]);
    }

    Eigen::Affine3f getViewerPose (visualization::PCLVisualizer& viewer)
    {
      Eigen::Affine3f pose = viewer.getViewerPose();
      Eigen::Matrix3f rotation = pose.linear();

      Matrix3f axis_reorder;
      axis_reorder << 0,  0,  1,
                     -1,  0,  0,
                      0, -1,  0;

      rotation = rotation * axis_reorder;
      pose.linear() = rotation;
      return pose;
    }

    boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const DeviceArray<PointXYZ>& triangles)
    {
      if (triangles.empty())
              return boost::shared_ptr<pcl::PolygonMesh>();

      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.width  = (int)triangles.size();
      cloud.height = 1;
      triangles.download(cloud.points);

      boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() );
      pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

      mesh_ptr->polygons.resize (triangles.size() / 3);
      for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
      {
        pcl::Vertices v;
        v.vertices.push_back(i*3+0);
        v.vertices.push_back(i*3+2);
        v.vertices.push_back(i*3+1);
        mesh_ptr->polygons[i] = v;
      }
      return mesh_ptr;
    }


}

 /////////////////////////////////////////////////////////////////////////
//CurrentFrameCloudView Class definitions



CurrentFrameCloudView::CurrentFrameCloudView() : cloud_device_ (480, 640), cloud_viewer_ ("Frame Cloud Viewer")
{
    cloud_ptr_ = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);

    cloud_viewer_.setBackgroundColor (0, 0, 0.15);
    cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    cloud_viewer_.addCoordinateSystem (1.0, "global");
    cloud_viewer_.initCameraParameters ();
    cloud_viewer_.setPosition (0, 500);
    cloud_viewer_.setSize (640, 480);
    cloud_viewer_.setCameraClipDistances (0.001, 2.01);
}

void CurrentFrameCloudView::show (const KinfuTracker& kinfu)
{
    kinfu.getLastFrameCloud (cloud_device_);

    int c;
    cloud_device_.download (cloud_ptr_->points, c);
    cloud_ptr_->width = cloud_device_.cols ();
    cloud_ptr_->height = cloud_device_.rows ();
    cloud_ptr_->is_dense = false;

    cloud_viewer_.removeAllPointClouds ();
    cloud_viewer_.addPointCloud<PointXYZ>(cloud_ptr_);
    cloud_viewer_.spinOnce ();
}

void CurrentFrameCloudView::setViewerPose (const Eigen::Affine3f& viewer_pose)
{
    kinfu_viz::setViewerPose (cloud_viewer_, viewer_pose);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
//ImageView Class definitions

ImageView::ImageView() : paint_image_ (false), accumulate_views_ (false)
{
  viewerScene_.setWindowTitle ("View3D from ray tracing");
  viewerScene_.setPosition (0, 0);
  viewerDepth_.setWindowTitle ("Kinect Depth stream");
  viewerDepth_.setPosition (640, 0);
  //viewerColor_.setWindowTitle ("Kinect RGB stream");
}

void ImageView::showScene (KinfuTracker& kinfu, const PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB>& rgb24, bool registration, Eigen::Affine3f* pose_ptr)
{
  if (pose_ptr)
  {
    raycaster_ptr_->run ( kinfu.volume (), *pose_ptr, kinfu.getCyclicalBufferStructure () ); //says in cmake it does not know it
    raycaster_ptr_->generateSceneView(view_device_);
  }
  else
  {
    kinfu.getImage (view_device_);
  }

  if (paint_image_ && registration && !pose_ptr)
  {
    colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
    paint3DView (colors_device_, view_device_);
  }

  int cols;
  view_device_.download (view_host_, cols);
  viewerScene_.showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows ());

        //viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);
#ifdef HAVE_OPENCV
  if (accumulate_views_)
  {
    views_.push_back (cv::Mat ());
    cv::cvtColor (cv::Mat (480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back (), CV_RGB2GRAY);
    //cv::copy(cv::Mat(480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back());
  }
#endif
}

//void ImageView::showDepth (const PtrStepSz<const unsigned short>& depth)
//{
//  viewerDepth_.showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true);
//}

void ImageView::showDepth (const sensor_msgs::Image::ConstPtr depth)
  {
    const unsigned short *image = reinterpret_cast<const unsigned short*>(&(depth->data[0]));

    viewerDepth_.showShortImage (image, depth->width, depth->height, 0, 1000, true);
  }

void ImageView::showGeneratedDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose)
{
  raycaster_ptr_->run(kinfu.volume(), pose, kinfu.getCyclicalBufferStructure ());
  raycaster_ptr_->generateDepthImage(generated_depth_);

  int c;
  vector<unsigned short> data;
  generated_depth_.download(data, c);

  viewerDepth_.showShortImage (&data[0], generated_depth_.cols(), generated_depth_.rows(), 0, 5000, true);
}

void ImageView::toggleImagePaint()
{
  paint_image_ = !paint_image_;
  cout << "Paint image: " << (paint_image_ ? "On   (requires registration mode)" : "Off") << endl;
}


//ImageView::ImageView(int viz) : viz_(viz), paint_image_ (true), accumulate_views_ (false)
//  {
//    if (viz_)
//    {
//        viewerScene_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
//        //viewerDepth_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);

//        viewerScene_->setWindowTitle ("View3D from ray tracing");
//        viewerScene_->setPosition (0, 0);
//        //viewerDepth_->setWindowTitle ("Kinect Depth stream");
//        //viewerDepth_->setPosition (640, 0);
//        //viewerColor_.setWindowTitle ("Kinect RGB stream");
//    }
//  }

//void ImageView::showScene (KinfuTracker& kinfu, cv::Mat &rgb, bool registration, Eigen::Affine3f* pose_ptr)
//  {
//    if (pose_ptr)
//    {
//        raycaster_ptr_->run( kinfu.volume (), *pose_ptr, kinfu.getCyclicalBufferStructure () );
//        raycaster_ptr_->generateSceneView(view_device_);
//    }
//    else
//      kinfu.getImage (view_device_);

//    if (paint_image_ && registration && !pose_ptr)
//    {
//      //colors_device_.upload (rgb->data, rgb24.step, rgb24.rows, rgb24.cols);
//      colors_device_.upload (&(rgb.data[0]), rgb.step, rgb.rows, rgb.cols);

//      paint3DView (colors_device_, view_device_);
//    }


//    int cols;
//    view_device_.download (view_host_, cols);
//    if (viz_)
//        viewerScene_->showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows ());

//    //viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);

//    if (accumulate_views_)
//    {
//      views_.push_back (cv::Mat ());
//      cv::cvtColor (cv::Mat (480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back (), CV_RGB2GRAY);
//      //cv::copy(cv::Mat(480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back());
//    }

//  }

//void ImageView::showScene (KinfuTracker& kinfu, bool registration, Eigen::Affine3f* pose_ptr)
//  {
//    if (pose_ptr)
//    {
//        raycaster_ptr_->run( kinfu.volume (), *pose_ptr, kinfu.getCyclicalBufferStructure () );
//        raycaster_ptr_->generateSceneView(view_device_);
//    }
//    else
//      kinfu.getImage (view_device_);

//    int cols;
//    view_device_.download (view_host_, cols);
//    if (viz_)
//        viewerScene_->showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows ());

//    //viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);

//    if (accumulate_views_)
//    {
//      views_.push_back (cv::Mat ());
//      cv::cvtColor (cv::Mat (480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back (), CV_RGB2GRAY);
//      //cv::copy(cv::Mat(480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back());
//    }

//  }


//void ImageView::showDepth (const sensor_msgs::Image::ConstPtr depth)
//  {
//    const unsigned short *image = reinterpret_cast<const unsigned short*>(&(depth->data[0]));

//     if (viz_)
//       viewerDepth_->showShortImage (image, depth->width, depth->height, 0, 1000, true);
//  }

//void ImageView::generateDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose, KinfuTracker::DepthMap &generated_depth_)
//  {
//    raycaster_ptr_->run( kinfu.volume (), pose, kinfu.getCyclicalBufferStructure () );
//    raycaster_ptr_->generateDepthImage(generated_depth_);

//    //if (viz_)
//      //  viewerDepth_->showShortImage (&data[0], generated_depth_.cols(), generated_depth_.rows(), 0, 1000, true);
//  }

//void ImageView::toggleImagePaint()
//  {
//    paint_image_ = !paint_image_;
//    cout << "Paint image: " << (paint_image_ ? "On   (requires registration mode)" : "Off") << endl;
//  }

//void ConvertImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
//{
//    cv_bridge::CvImageConstPtr pCvImage;
//    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
//    pCvImage->image.copyTo(image);
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////


SceneCloudView::SceneCloudView() : extraction_mode_ (GPU_Connected6), compute_normals_ (false), valid_combined_ (false), cube_added_(false), cloud_viewer_ ("Scene Cloud Viewer")
{
  cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
  normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
  combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
  point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);

  cloud_viewer_.setBackgroundColor (0, 0, 0);
  cloud_viewer_.addCoordinateSystem (1.0, "global");
  cloud_viewer_.initCameraParameters ();
  cloud_viewer_.setPosition (0, 500);
  cloud_viewer_.setSize (640, 480);
  cloud_viewer_.setCameraClipDistances (0.01, 10.01);

  cloud_viewer_.addText ("H: print help", 2, 15, 20, 34, 135, 246);
  cloud_viewer_.addText ("ICP State: ", 450, 55, 20, 0.0, 1.0, 0.0, "icp");
  cloud_viewer_.addText ("Press 'S' to save the current world", 450, 35, 10, 0.0, 1.0, 0.0, "icp_save");
  cloud_viewer_.addText ("Press 'R' to reset the system", 450, 15, 10, 0.0, 1.0, 0.0, "icp_reset");
}

inline void SceneCloudView::drawCamera (Eigen::Affine3f& pose, const string& name, double r, double g, double b)
{
  double focal = 575;
  double height = 480;
  double width = 640;

  // create a 5-point visual for each camera
  pcl::PointXYZ p1, p2, p3, p4, p5;
  p1.x=0; p1.y=0; p1.z=0;
  double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
  double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
  double dist = 0.75;
  double minX, minY, maxX, maxY;
  maxX = dist*tan (atan (width / (2.0*focal)));
  minX = -maxX;
  maxY = dist*tan (atan (height / (2.0*focal)));
  minY = -maxY;
  p2.x=minX; p2.y=minY; p2.z=dist;
  p3.x=maxX; p3.y=minY; p3.z=dist;
  p4.x=maxX; p4.y=maxY; p4.z=dist;
  p5.x=minX; p5.y=maxY; p5.z=dist;
  p1=pcl::transformPoint (p1, pose);
  p2=pcl::transformPoint (p2, pose);
  p3=pcl::transformPoint (p3, pose);
  p4=pcl::transformPoint (p4, pose);
  p5=pcl::transformPoint (p5, pose);
  std::stringstream ss;
  ss.str ("");
  ss << name << "_line1";
  cloud_viewer_.addLine (p1, p2, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line2";
  cloud_viewer_.addLine (p1, p3, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line3";
  cloud_viewer_.addLine (p1, p4, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line4";
  cloud_viewer_.addLine (p1, p5, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line5";
  cloud_viewer_.addLine (p2, p5, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line6";
  cloud_viewer_.addLine (p5, p4, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line7";
  cloud_viewer_.addLine (p4, p3, r, g, b, ss.str ());
  ss.str ("");
  ss << name << "_line8";
  cloud_viewer_.addLine (p3, p2, r, g, b, ss.str ());
}

inline void SceneCloudView::removeCamera (const string& name)
{
  cloud_viewer_.removeShape (name);
  std::stringstream ss;
  ss.str ("");
  ss << name << "_line1";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line2";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line3";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line4";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line5";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line6";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line7";
  cloud_viewer_.removeShape (ss.str ());
  ss.str ("");
  ss << name << "_line8";
  cloud_viewer_.removeShape (ss.str ());
}

void SceneCloudView::displayICPState (KinfuTracker& kinfu, bool was_lost_)
{
  string name = "last_good_track";
  string name_estimate = "last_good_estimate";
  if (was_lost_ && !kinfu.icpIsLost ()) //execute only when ICP just recovered (i.e. was_lost_ == true && icpIsLost == false)
  {
    removeCamera(name);
    removeCamera(name_estimate);
    clearClouds(false);
    cloud_viewer_.updateText ("ICP State: OK", 450, 55, 20, 0.0, 1.0, 0.0, "icp");
    cloud_viewer_.updateText ("Press 'S' to save the current world", 450, 35, 10, 0.0, 1.0, 0.0, "icp_save");
    cloud_viewer_.updateText ("Press 'R' to reset the system", 450, 15, 10, 0.0, 1.0, 0.0, "icp_reset");
  }
  else if (!was_lost_ && kinfu.icpIsLost()) //execute only when we just lost ourselves (i.e. was_lost_ = false && icpIsLost == true)
  {
    // draw position of the last good track
    Eigen::Affine3f last_pose = kinfu.getCameraPose();
    drawCamera(last_pose, name, 0.0, 1.0, 0.0);



    cloud_viewer_.updateText ("ICP State: LOST", 450, 55, 20, 1.0, 0.0, 0.0, "icp");
    cloud_viewer_.updateText ("Press 'S' to save the current world", 450, 35, 10, 1.0, 0.0, 0.0, "icp_save");
    cloud_viewer_.updateText ("Press 'R' to reset the system", 450, 15, 10, 1.0, 0.0, 0.0, "icp_reset");
  }


  if( kinfu.icpIsLost() )
  {
    removeCamera(name_estimate);
     // draw current camera estimate
    Eigen::Affine3f last_pose_estimate = kinfu.getLastEstimatedPose();
    drawCamera(last_pose_estimate, name_estimate, 1.0, 0.0, 0.0);
  }



//       cout << "current estimated pose: " << kinfu.getLastEstimatedPose().translation() << std::endl << kinfu.getLastEstimatedPose().linear() << std::endl;
//
}

void SceneCloudView::show (KinfuTracker& kinfu, bool integrate_colors)
{
  viewer_pose_ = kinfu.getCameraPose();

  ScopeTimeT time ("PointCloud Extraction");
  cout << "\nGetting cloud... " << flush;

  valid_combined_ = false;

  if (extraction_mode_ != GPU_Connected6)     // So use CPU
  {
    kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
  }
  else
  {
    DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);

    if (compute_normals_)
    {
      kinfu.volume().fetchNormals (extracted, normals_device_);
      pcl::gpu::kinfuLS::mergePointNormal (extracted, normals_device_, combined_device_);
      combined_device_.download (combined_ptr_->points);
      combined_ptr_->width = (int)combined_ptr_->points.size ();
      combined_ptr_->height = 1;

      valid_combined_ = true;
    }
    else
    {
      extracted.download (cloud_ptr_->points);
      cloud_ptr_->width = (int)cloud_ptr_->points.size ();
      cloud_ptr_->height = 1;
    }

    if (integrate_colors)
    {
      kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
      point_colors_device_.download(point_colors_ptr_->points);
      point_colors_ptr_->width = (int)point_colors_ptr_->points.size ();
      point_colors_ptr_->height = 1;
    }
    else
      point_colors_ptr_->points.clear();
  }
  size_t points_size = valid_combined_ ? combined_ptr_->points.size () : cloud_ptr_->points.size ();
  cout << "Done.  Cloud size: " << points_size / 1000 << "K" << endl;

  cloud_viewer_.removeAllPointClouds ();
  if (valid_combined_)
  {
    visualization::PointCloudColorHandlerRGBHack<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
    cloud_viewer_.addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
    cloud_viewer_.addPointCloudNormals<PointNormal>(combined_ptr_, 50);
  }
  else
  {
    visualization::PointCloudColorHandlerRGBHack<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
    cloud_viewer_.addPointCloud<PointXYZ> (cloud_ptr_, rgb);
  }
}

void SceneCloudView::toggleCube(const Eigen::Vector3f& size)
{
  if (cube_added_)
    cloud_viewer_.removeShape("cube");
  else
    cloud_viewer_.addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

  cube_added_ = !cube_added_;
}

void SceneCloudView::toggleExtractionMode ()
{
  extraction_mode_ = (extraction_mode_ + 1) % 3;
  switch (extraction_mode_)
  {
    case 0: cout << "Cloud extraction mode: GPU, Connected-6" << endl; break;
    case 1: cout << "Cloud extraction mode: CPU, Connected-6    (requires a lot of memory)" << endl; break;
    case 2: cout << "Cloud extraction mode: CPU, Connected-26   (requires a lot of memory)" << endl; break;
  }
}

void SceneCloudView::toggleNormals ()
{
  compute_normals_ = !compute_normals_;
  cout << "Compute normals: " << (compute_normals_ ? "On" : "Off") << endl;
}

void SceneCloudView::clearClouds (bool print_message)
{
  cloud_viewer_.removeAllPointClouds ();
  cloud_ptr_->points.clear ();
  normals_ptr_->points.clear ();
  if (print_message)
    cout << "Clouds/Meshes were cleared" << endl;
}

void SceneCloudView::showMesh(KinfuTracker& kinfu, bool /*integrate_colors*/)
{
  ScopeTimeT time ("Mesh Extraction");
  cout << "\nGetting mesh... " << flush;

  if (!marching_cubes_)
    marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

  DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);
  mesh_ptr_ = kinfu_viz::convertToMesh(triangles_device);

  cloud_viewer_.removeAllPointClouds ();
  if (mesh_ptr_)
    cloud_viewer_.addPolygonMesh(*mesh_ptr_);

  cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << endl;
}


//SceneCloudView::SceneCloudView(int viz) : viz_(viz), extraction_mode_ (GPU_Connected6), compute_normals_ (false), valid_combined_ (false), cube_added_(false)
//{
//  cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
//  normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
//  combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
//  point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);
//  combined_color_ptr_ = PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB>);

//  if (viz_)
//  {
//      cloud_viewer_ = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer("Scene Cloud Viewer") );

//      cloud_viewer_->setBackgroundColor (0, 0, 0);
//      cloud_viewer_->addCoordinateSystem (1.0, "global");
//      cloud_viewer_->initCameraParameters ();
//      cloud_viewer_->setPosition (0, 500);
//      //cloud_viewer_->setSize (640, 480);
//      cloud_viewer_->setSize (640, 440);
//      cloud_viewer_->setCameraClipDistances (0.001, 10.01);

//      cloud_viewer_->addText ("H: print help", 2, 15, 20, 34, 135, 246);
//  }
//}


//void SceneCloudView::generateCloud(KinfuTracker& kinfu, bool integrate_colors)
//{
//  viewer_pose_ = kinfu.getCameraPose();

//  ScopeTimeT time ("PointCloud Extraction");
//  cout << "\nGetting cloud... " << flush;

//  valid_combined_ = false;
//  bool valid_extracted_ = false;

//  if (extraction_mode_ != GPU_Connected6)     // So use CPU
//  {
//    kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
//  }
//  elsea
//  {
//    DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);

//    if(extracted.size() > 0){
//        valid_extracted_ = true;

//        extracted.download (cloud_ptr_->points);
//        cloud_ptr_->width = (int)cloud_ptr_->points.size ();
//        cloud_ptr_->height = 1;

//        if (integrate_colors)
//        {
//          kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
//          point_colors_device_.download(point_colors_ptr_->points);
//          point_colors_ptr_->width = (int)point_colors_ptr_->points.size ();
//          point_colors_ptr_->height = 1;
//          //pcl::gpu::mergePointRGB(extracted, point_colors_device_, combined_color_device_);
//          //combined_color_device_.download (combined_color_ptr_->points);
//        }
//        else
//          point_colors_ptr_->points.clear();
//        combined_color_ptr_->clear();
//        generateXYZRGB(cloud_ptr_, point_colors_ptr_, combined_color_ptr_);

//    }else{
//        valid_extracted_ = false;
//        cout << "Failed to Extract Cloud " << endl;

//    }
//  }

//  cout << "Done.  Cloud size: " << cloud_ptr_->points.size () / 1000 << "K" << endl;

//}

//void SceneCloudView::generateXYZRGB(PointCloud<PointXYZ>::Ptr cloud_ptr, PointCloud<RGB>::Ptr rgb_ptr_, PointCloud<PointXYZRGB>::Ptr output){

//    for(int i = 0; i < cloud_ptr->width; i++){
//        pcl::PointXYZRGB point;

//        point.x = (*cloud_ptr)[i].x;
//        point.y = (*cloud_ptr)[i].y;
//        point.z = (*cloud_ptr)[i].z;

//        point.b = (*rgb_ptr_)[i].b;
//        point.g = (*rgb_ptr_)[i].g;
//        point.r = (*rgb_ptr_)[i].r;

//        output->push_back(point);

//    }

//}


//void SceneCloudView::show (KinfuTracker& kinfu, bool integrate_colors)
//{
//  viewer_pose_ = kinfu.getCameraPose();

//  ScopeTimeT time ("PointCloud Extraction");
//  cout << "\nGetting cloud... " << flush;

//  valid_combined_ = false;

//  if (extraction_mode_ != GPU_Connected6)     // So use CPU
//  {
//    kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
//  }
//  else
//  {
//    DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);


//    if (compute_normals_)
//    {
//      kinfu.volume().fetchNormals (extracted, normals_device_);
//      pcl::gpu::kinfuLS::mergePointNormal (extracted, normals_device_, combined_device_);
//      combined_device_.download (combined_ptr_->points);
//      combined_ptr_->width = (int)combined_ptr_->points.size ();
//      combined_ptr_->height = 1;

//      valid_combined_ = true;
//    }
//    else
//    {
//      extracted.download (cloud_ptr_->points);
//      cloud_ptr_->width = (int)cloud_ptr_->points.size ();
//      cloud_ptr_->height = 1;
//    }

//    if (integrate_colors)
//    {
//      kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
//      point_colors_device_.download(point_colors_ptr_->points);
//      point_colors_ptr_->width = (int)point_colors_ptr_->points.size ();
//      point_colors_ptr_->height = 1;
//    }
//    else
//      point_colors_ptr_->points.clear();
//  }
//  size_t points_size = valid_combined_ ? combined_ptr_->points.size () : cloud_ptr_->points.size ();
//  cout << "Done.  Cloud size: " << points_size / 1000 << "K" << endl;

//  if (viz_)
//  {
//      cloud_viewer_->removeAllPointClouds ();
//      if (valid_combined_)
//      {
//        visualization::PointCloudColorHandlerRGBCloud<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
//        cloud_viewer_->addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
//        cloud_viewer_->addPointCloudNormals<PointNormal>(combined_ptr_, 50);
//      }
//      else
//      {
//        visualization::PointCloudColorHandlerRGBCloud<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
//        cloud_viewer_->addPointCloud<PointXYZ> (cloud_ptr_, rgb);
//      }
//  }
//}

//void SceneCloudView::toggleCube(const Eigen::Vector3f& size)
//{
//    if (!viz_)
//        return;

//    if (cube_added_)
//        cloud_viewer_->removeShape("cube");
//    else
//      cloud_viewer_->addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

//    cube_added_ = !cube_added_;
//}

//void SceneCloudView::toggleExtractionMode ()
//{
//  extraction_mode_ = (extraction_mode_ + 1) % 3;

//  switch (extraction_mode_)
//  {
//  case 0: cout << "Cloud exctraction mode: GPU, Connected-6" << endl; break;
//  case 1: cout << "Cloud exctraction mode: CPU, Connected-6    (requires a lot of memory)" << endl; break;
//  case 2: cout << "Cloud exctraction mode: CPU, Connected-26   (requires a lot of memory)" << endl; break;
//  }
//  ;
//}

//void SceneCloudView::toggleNormals ()
//{
//  compute_normals_ = !compute_normals_;
//  cout << "Compute normals: " << (compute_normals_ ? "On" : "Off") << endl;
//}

//void SceneCloudView::clearClouds (bool print_message)
//{
//  if (!viz_)
//      return;

//  cloud_viewer_->removeAllPointClouds ();
//  cloud_ptr_->points.clear ();
//  normals_ptr_->points.clear ();
//  if (print_message)
//    cout << "Clouds/Meshes were cleared" << endl;
//}

//void SceneCloudView::showMesh(KinfuTracker& kinfu, bool /*integrate_colors*/)
//    {
//      if (!viz_)
//         return;

//      ScopeTimeT time ("Mesh Extraction");
//      cout << "\nGetting mesh... " << flush;

//      if (!marching_cubes_)
//        marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

//      DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);
//      mesh_ptr_ = kinfu_ls_tools::convertToMesh(triangles_device);

//      cloud_viewer_->removeAllPointClouds ();
//      if (mesh_ptr_)
//        cloud_viewer_->addPolygonMesh(*mesh_ptr_);

//      cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << endl;
//    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

SampledScopeTime::SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}

SampledScopeTime::~SampledScopeTime()
{
  static int i_ = 0;
  static boost::posix_time::ptime starttime_ = boost::posix_time::microsec_clock::local_time();
  time_ms_ += getTime ();
  if (i_ % EACH == 0 && i_)
  {
    boost::posix_time::ptime endtime_ = boost::posix_time::microsec_clock::local_time();
    cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )"
         << "( real: " << 1000.f * EACH / (endtime_-starttime_).total_milliseconds() << "fps )"  << endl;
    time_ms_ = 0;
    starttime_ = endtime_;
  }
  ++i_;
}
