
#include <fruit_rgbd.h>


//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

static const std::string OPENCV_WINDOW = "Image window";
const std::string cloudName = "Output";



using namespace std;
fruit_rgbd_filter::fruit_rgbd_filter():result(new PointCloud_), source(new PointCloud_),target(new PointCloud_),init_flag_pc_(0),merged_pc_(new PointCloud_),viewer_("Merged Cloud Viewer")
{
	cv::namedWindow(OPENCV_WINDOW);
	// cv::namedWindow(OPENCV_WINDOW2);

	first = true;

	std::string topicPointCloud = "/camera/depth_registered/points";

	pcl_sub = nh_.subscribe(topicPointCloud, 1, &fruit_rgbd_filter::pcl_callback,this);
	//pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("/fruit_rgbd/points", 1 );
	pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("/fruit_rgbd/segmented_points", 1 );
	pcl_merged_pub = nh_.advertise<sensor_msgs::PointCloud2>("/fruit_rgbd/merged_points", 1 );

	gt_pose_pub = nh_.advertise<geometry_msgs::Pose>("/fruit_rgbd/gt_pose", 1 );

	#ifdef NDT
	ndt_pose_pub = nh_.advertise<geometry_msgs::Pose>("/fruit_rgbd/ndt_pose", 1 );
	#endif

	#ifdef ICP
	icp_pose_pub = nh_.advertise<geometry_msgs::Pose>("/fruit_rgbd/icp_pose", 1 );
	#endif

	//image_sub = nh_.subscribe("/camera/image/rgb_raw", 10, &fruit_rgbd_filter::image_callback,this);
	//ros::Subscriber pcl_sub = nh_.subscribe<sensor_msgs::PointCloud2>(topicPointCloud, 1, &fruit_rgbd_filter::pcl_callback,this);


	/*capsicum_model.hue_mean = 84;
	capsicum_model.saturation_mean = 0.866;//221;
	capsicum_model.value_mean = 0.47;//121;
	capsicum_model.hue_var = 20.2;
	capsicum_model.saturation_var = 5.43; //1368;
	capsicum_model.value_var = 2.95; //753;*/

	/*
	capsicum_model.hue_mean = 183.198;
	capsicum_model.saturation_mean = 0.8323;//32.6;
	capsicum_model.value_mean = 0.3907;//115;
	capsicum_model.hue_var = 4.3659;//pow(4.3659,2); //650
	capsicum_model.saturation_var = 0.1600; //412;
	capsicum_model.value_var = pow(0.1299,2); //641;
	*/

	capsicum_model.hue_mean = 183.198;
	capsicum_model.saturation_mean = 1.8323;//32.6;
	capsicum_model.value_mean = 0.3907;//115;
	capsicum_model.hue_var = 4.3659;//pow(4.3659,2); //650
	capsicum_model.saturation_var = 0.1600; //412;
	capsicum_model.value_var = pow(0.1299,2); //641;

	capsicumDetector = capsicum_detector(capsicum_model);

	//visualizer = new pcl::visualization::PCLVisualizer ("PCL Visualisation");

	//visualizer->setBackgroundColor(0,0,0);
	//visualizer->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	//visualizer->createViewPort (0.0, 0, 1.0, 1.0, vp_1);
	//visualizer->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	//init_flag_pc_=0;
	GlobalTransform=Eigen::Matrix4f::Identity();
	cout<<"Here"<<endl;


}

fruit_rgbd_filter::~fruit_rgbd_filter()
{
    cv::destroyWindow(OPENCV_WINDOW);

}

void fruit_rgbd_filter::start(){

    //float wait_time = 0.0;
    //cv::setMouseCallback(OPENCV_WINDOW, mouse_click, this);
    //bool success = false;
    //bool capsicumAttached = false;
    //bool reset = false;

    ros::spin();

    //ros::AsyncSpinner spinner(2);
    //spinner.start();


}

void fruit_rgbd_filter::pcl_callback(const sensor_msgs::PointCloud2 msg)
{
	//listener.waitForTransform("base_link","camera_depth_optical_frame",ros::Time(0), ros::Duration(10));

	PointCloud::Ptr cloud(new PointCloud);
	PointCloud::Ptr target_demean (new PointCloud);
	PointCloud::ConstPtr const_cloud_filtered;
	PointCloud::Ptr cloud_filtered(new PointCloud);

	std::vector<pcl::PointXYZ> pc_line1(2);
	std::vector<pcl::PointXYZ> pc_line2(2);
	std::vector<pcl::PointXYZ> pc_line3(2);

	pcl::PCA<PointT> pca_analysis;

	Eigen::Vector4f mean, centroid;
	Eigen::Matrix3f eig_vec;
	Eigen::Vector3f eig_val;
	Eigen::Vector3f plane;

	pcl::fromROSMsg(msg,*cloud);

	

	//std::cout<<"frame="<<msg.header.frame_id<<std::endl;

	std::vector<int> indices;

	pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

	//    capsicumDetector.segmentCloud(cloud, cloud_filtered, 0.0, true);
	capsicumDetector.segmentCloud(capsicum_model, cloud, cloud_filtered, 1.0, true);

	cloud_filtered = filter_cloud(cloud_filtered);

	//cloud_filtered->header.stamp=ros::Time::now();

	if(init_flag_pc_==0 && cloud_filtered->size()>0)
	{
		pcl::copyPointCloud (*cloud_filtered, *target);
		pcl::copyPointCloud (*cloud_filtered, *source);
		//init_pc_=cloud_filtered;
		init_flag_pc_=1;
		//pcl::copyPointCloud (*cloud_filtered, *init_pc_);

		#ifdef ICP
			pairAlignICP (source, target,merged_pc_,pairTransform,0);
		#endif

		#ifdef NDT
			pairAlignNDT(source, target,merged_pc_,pairTransform);
		#endif

		static tf::TransformBroadcaster br;
		tf::Transform temp_tf;
		temp_tf = tfFromEigen(GlobalTransform);
		br.sendTransform(tf::StampedTransform(temp_tf, ros::Time::now(), "camera_depth_optical_frame", "GlobalTransform_frame"));

		//tf, world to camera_depth_optical_frame
		//listener.waitForTransform("base_link","camera_depth_optical_frame",ros::Time(0), ros::Duration(5.0));
		listener.waitForTransform("base_link","camera_depth_optical_frame",ros::Time(0), ros::Duration(5.0));
				
		std::cout<<"Obtain the initial pc"<<std::endl;
	}
	
	
	if(init_flag_pc_ && cloud_filtered->size()>0)
	{
		pcl::copyPointCloud (*cloud_filtered, *target);
		static tf::TransformBroadcaster br;
		tf::Transform temp_tf;
		PointCloud_::Ptr temp_pc(new PointCloud_);
		
		#ifdef ICP
			pairAlignICP (source, target,temp_pc,pairTransform,0);
		#endif

		#ifdef NDT
			pairAlignNDT(source, target,temp_pc,pairTransform);
		#endif
	
		//transform current pair into the global transform
    		pcl::transformPointCloud (*temp_pc, *merged_pc_, GlobalTransform);

				
		//cout<<pairTransform<<endl;
		//cout<<temp_tf<<endl;
		//update the global transform
    		GlobalTransform = GlobalTransform * pairTransform;
		//cout<<GlobalTransform<<endl;

		temp_tf = tfFromEigen(GlobalTransform);
		br.sendTransform(tf::StampedTransform(temp_tf, ros::Time::now(), "camera_depth_optical_frame", "GlobalTransform_frame"));

		sensor_msgs::PointCloud2 pc2msg,pcMergedmsg;
		pcl::toROSMsg(*cloud_filtered,pc2msg);
		pc2msg.header.stamp=ros::Time::now();
		pc2msg.header.frame_id = "camera_depth_optical_frame";
		pcl_pub.publish(pc2msg);
		pcl::copyPointCloud (*target, *source);

		
		pcl::toROSMsg(*merged_pc_,pcMergedmsg);
		pcMergedmsg.header.stamp=ros::Time::now();
		pcMergedmsg.header.frame_id = "camera_depth_optical_frame";
		pcl_merged_pub.publish(pcMergedmsg);
		viewer_.showCloud(merged_pc_);
		//visualizer->addPointCloud(merged_pc_,"merged_pc_");
		//visualizer->spinOnce(1);
		listener.waitForTransform("base_link","camera_depth_optical_frame",ros::Time(0), ros::Duration(10.0));
		listener.waitForTransform("base_link","GlobalTransform_frame",ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("base_link","camera_depth_optical_frame",ros::Time(0), gt_pose_tf);
		listener.lookupTransform("base_link","GlobalTransform_frame",ros::Time(0), icp_pose_tf);
		
		
		gt_pose.position.x=gt_pose_tf.getOrigin().getX();
		gt_pose.position.y=gt_pose_tf.getOrigin().getY();
		gt_pose.position.z=gt_pose_tf.getOrigin().getZ();
		gt_pose.orientation.x=gt_pose_tf.getRotation().getX();
		gt_pose.orientation.y=gt_pose_tf.getRotation().getY();
		gt_pose.orientation.z=gt_pose_tf.getRotation().getZ();
		gt_pose.orientation.w=gt_pose_tf.getRotation().getW();

		icp_pose.position.x=icp_pose_tf.getOrigin().getX();
		icp_pose.position.y=icp_pose_tf.getOrigin().getY();
		icp_pose.position.z=icp_pose_tf.getOrigin().getZ();
		icp_pose.orientation.x=icp_pose_tf.getRotation().getX();
		icp_pose.orientation.y=icp_pose_tf.getRotation().getY();
		icp_pose.orientation.z=icp_pose_tf.getRotation().getZ();
		icp_pose.orientation.w=icp_pose_tf.getRotation().getW();

		

		gt_pose_pub.publish(gt_pose);
		icp_pose_pub.publish(icp_pose);
		//source=target;
	}

	/*
	float scale = 0.01;
	float scale2 = 0.001;
	// your point cloud

	if(cloud->size() > 3 )
	{

	pcl::compute3DCentroid(*cloud_filtered, centroid);

	pcl::demeanPointCloud<PointT> (*cloud_filtered, centroid, *cloud_filtered);
	pcl::demeanPointCloud<PointT> (*cloud, centroid, *cloud);

	const_cloud_filtered = cloud_filtered;
	pca_analysis.setInputCloud(const_cloud_filtered);

	eig_vec = pca_analysis.getEigenVectors();
	eig_val = pca_analysis.getEigenValues();

	pc_line1.at(0).x = 0;
	pc_line1.at(0).y = 0;
	pc_line1.at(0).z = 0;

	pc_line1.at(1).x = eig_vec(0,0)*sqrt(eig_val[0])*scale;
	pc_line1.at(1).y = eig_vec(1,0)*sqrt(eig_val[0])*scale;
	pc_line1.at(1).z = eig_vec(2,0)*sqrt(eig_val[0])*scale;


	pc_line2.at(0).x = 0;
	pc_line2.at(0).y = 0;
	pc_line2.at(0).z = 0;

	pc_line2.at(1).x = eig_vec(0,1)*sqrt(eig_val[1])*scale;
	pc_line2.at(1).y = eig_vec(1,1)*sqrt(eig_val[1])*scale;
	pc_line2.at(1).z = eig_vec(2,1)*sqrt(eig_val[1])*scale;

	pc_line3.at(0).x = 0;
	pc_line3.at(0).y = 0;
	pc_line3.at(0).z = 0;

	pc_line3.at(1).x = eig_vec(0,2)*sqrt(eig_val[2])*scale;
	pc_line3.at(1).y = eig_vec(1,2)*sqrt(eig_val[2])*scale;
	pc_line3.at(1).z = eig_vec(2,2)*sqrt(eig_val[2])*scale;

	Eigen::Vector3f eig_vec1 = eig_vec.block(0, 0, 3, 1);
	Eigen::Vector3f eig_vec2 = eig_vec.block(0, 1, 3, 1);

	plane = eig_vec1.cross(eig_vec2);


	// draw the cloud and the box
	visualizer->removePointCloud("cloud_filtered");
	visualizer->removePointCloud("cloud");
	visualizer->removeShape("pc1");
	visualizer->removeShape("pc2");
	visualizer->removeShape("pc3");
	visualizer->removeShape("plane");
	//visualizer->removeShape("cube");
	visualizer->addPointCloud(cloud_filtered,"cloud_filtered");
	visualizer->addPointCloud(cloud,"cloud");


	pcl::ModelCoefficients plane_coeff;
	plane_coeff.values.resize (4);    // We need 4 values
	plane_coeff.values[0] = plane[0]*scale2;
	plane_coeff.values[1] = plane[1]*scale2;
	plane_coeff.values[2] = plane[2]*scale2;
	plane_coeff.values[3] = 0;

	visualizer->addPlane(plane_coeff);

	visualizer->addArrow(pc_line1.at(1),pc_line1.at(0),1.0,0.0,0.0,false,"pc1");
	visualizer->addArrow(pc_line2.at(1),pc_line2.at(0),0.0,1.0,0.0,false,"pc2");
	//visualizer->addLine(pc_line3.at(0),pc_line3.at(1),"pc3");

	//visualizer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, "cube");
	visualizer->spinOnce(1);

	}
	*/






}

/*PointCloud::Ptr fruit_rgbd_filter::segment_capsicum(PointCloud::Ptr input_cloud){

    PointCloud::Ptr cloud_segmented (new PointCloud);

    capsicumDetector.segmentation(capsicum_model, input_cloud, *cloud_segmented, true);

    return cloud_segmented;
}*/

void fruit_rgbd_filter::image_callback(const sensor_msgs::Image::ConstPtr imageColor){
    cv::Mat color;

    readImage(imageColor, color);
    cv::imshow(OPENCV_WINDOW, color);
    cv::waitKey(3);
}


void fruit_rgbd_filter::readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud){

      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  }


void fruit_rgbd_filter::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fruit_rgbd");

  fruit_rgbd_filter fp;

  fp.start();

  return 0;
}
