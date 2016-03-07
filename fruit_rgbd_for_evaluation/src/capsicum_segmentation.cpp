
#include "capsicum_segmentation.h"


//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

static const std::string OPENCV_WINDOW = "Image window";
const std::string cloudName = "Output";

capsicum_segmentation::capsicum_segmentation():it_(nh_)
{
    //cv::namedWindow(OPENCV_WINDOW);
   // cv::namedWindow(OPENCV_WINDOW2);

    first = true;

    std::string topicColor = "/camera/rgb/image_raw";
    std::string topicDepth = "/camera/depth/image_raw";
    std::string topicCloudRGB = "/camera/depth_registered/points";
    std::string topicCloudDepth = "/camera/depth/points";

    /*capsicum_model.hue_mean = 84;
    capsicum_model.saturation_mean = 0.866;//221;
    capsicum_model.value_mean = 0.47;//121;
    capsicum_model.hue_var = 20.2;
    capsicum_model.saturation_var = 5.43; //1368;
    capsicum_model.value_var = 2.95; //753;*/

    capsicum_model.hue_mean = 91.59;
    capsicum_model.saturation_mean = 212.2;
    capsicum_model.value_mean = 99.6;
    capsicum_model.hue_var = pow(2.18,2); //650
    capsicum_model.saturation_var = pow(40.80,2); //412;
    capsicum_model.value_var = pow(33.14,2); //641;

    capsicumDetector = capsicum_detector(capsicum_model);

    //subCloudRGB = nh_.subscribe(topicCloudRGB, 1);
    subCloudRGB =  nh_.subscribe(topicCloudRGB, 1, &capsicum_segmentation::callback,this);
    //subCloudRGB = new message_filters::Subscriber<sensor_msgs::PointCloud2ConstPtr>(nh_,topicCloudRGB, 1);
//    subCloudDepth = new message_filters::Subscriber<sensor_msgs::PointCloud2ConstPtr>(nh_,topicCloudDepth,1);

    subImageColor = new image_transport::SubscriberFilter(it_, topicColor, 1);
    subImageDepth = new image_transport::SubscriberFilter(it_, topicDepth, 1);

    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(2), *subImageColor, *subImageDepth);
//    syncCloud = new message_filters::Synchronizer<SyncPolicyCloud>(SyncPolicyCloud(2), *subCloudRGB, *subCloudDepth);

//    sync->registerCallback(boost::bind(&capsicum_segmentation::callback, this, _1, _2));
//    syncCloud->registerCallback(boost::bind(&capsicum_segmentation::callback, this, _1, _2));

    segmented_rgb_pub = it_.advertise("/capsicum/rgb/image_segmented", 1 );
    segmented_depth_pub = it_.advertise("/capsicum/depth/image_segmented", 1 );
//    capsicum_publisher = nh_.advertise

}

capsicum_segmentation::~capsicum_segmentation()
{
    cv::destroyWindow(OPENCV_WINDOW);

}

void capsicum_segmentation::start(){

    //float wait_time = 0.0;
    //cv::setMouseCallback(OPENCV_WINDOW, mouse_click, this);
    //bool success = false;
    //bool capsicumAttached = false;
    //bool reset = false;

    ros::spin();

    //ros::AsyncSpinner spinner(2);
    //spinner.start();


}

//void capsicum_segmentation::callback(const sensor_msgs::PointCloud2ConstPtr msgCloudRGB, const sensor_msgs::PointCloud2ConstPtr msgCloudDepth){

//    PointCloud::Ptr cloud(new PointCloud);
//    sensor_msgs::Image imageRGB,imageDepth;

//    pcl::toROSMsg(*msgCloudRGB,imageRGB);
//    pcl::toROSMsg(*msgCloudRGB,imageDepth);


//    segmented_rgb_pub.publish(imageRGB);
//    segmented_depth_pub.publish(imageDepth);
//    /*cv::Mat color,depth,segmentedColor;

//    depth_frame_id = imageDepth->header.frame_id;
//    color_frame_id = imageColor->header.frame_id;

//    readImage(imageDepth, color);
//    readImage(imageColor, depth);



//    capsicumDetector.filterImage(color,color);
//    capsicumDetector.segmentImage(color,segmentedColor,0.0);


//    pcl_pub.publish(cvImagetoMsg(color,sensor_msgs::image_encodings::BGR8));*/

//}


void capsicum_segmentation::callback(const sensor_msgs::PointCloud2ConstPtr msgCloud){

    PointCloud::Ptr cloud(new PointCloud);
    capsicum_segmentation::readCloud(msgCloud,cloud);
    cv::Mat rgb_image(cloud->height,cloud->width,CV_8UC3);
    cv::Mat depth_image(cloud->height,cloud->width,CV_16UC1);

    cv::Mat rgb_filtered, segmented_rgb, depth_image_segmented;

    if(cloud->isOrganized()){
        for(int i = 0; i < cloud->height; i++){
            for(int j = 0; j < cloud->width; j++){
                rgb_image.at<cv::Vec3b>(i,j)[0] = cloud->at(j,i).b;
                rgb_image.at<cv::Vec3b>(i,j)[1] = cloud->at(j,i).g;
                rgb_image.at<cv::Vec3b>(i,j)[2] = cloud->at(j,i).r;

                float depth = cloud->at(j,i).z;
                if(!isnan(depth) && depth > 0){
                    depth_image.at<uint16_t>(i,j) = depth*1000;
                }else{
                    depth_image.at<uint16_t>(i,j) = 0;
                }
                //rgb_image.at<cv::Vec3b>(i,j)[1] = cloud->points.at<PointT>(i,j).g;
            }
        }
    }

    capsicumDetector.filterImage(rgb_image,rgb_filtered);
    capsicumDetector.segmentImage(capsicum_model,rgb_filtered,segmented_rgb,1.0);

    depth_image.copyTo(depth_image_segmented,segmented_rgb);

    //cv::imshow(segmented_rgb);

    //segmented_rgb_pub.publish(image);
    /*cv::Mat color,depth,segmentedColor;

    depth_frame_id = imageDepth->header.frame_id;
    color_frame_id = imageColor->header.frame_id;

    readImage(imageDepth, color);
    readImage(imageColor, depth);*/


    segmented_rgb_pub.publish(cvImagetoMsg(rgb_image,sensor_msgs::image_encodings::BGR8));
    segmented_depth_pub.publish(cvImagetoMsg(depth_image_segmented,sensor_msgs::image_encodings::TYPE_16UC1));

}


//void capsicum_segmentation::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth){

//    cv::Mat color,depth,segmentedColor;

//    depth_frame_id = imageDepth->header.frame_id;
//    color_frame_id = imageColor->header.frame_id;

//    readImage(imageDepth, color);
//    readImage(imageColor, depth);

//    capsicumDetector.filterImage(color,color);
//    capsicumDetector.segmentImage(color,segmentedColor,0.0);

//    segmented_rgb_pub.publish(cvImagetoMsg(color,sensor_msgs::image_encodings::BGR8));

//}


void capsicum_segmentation::image_callback(const sensor_msgs::Image::ConstPtr imageColor){
    cv::Mat color;

    readImage(imageColor, color);
    cv::imshow(OPENCV_WINDOW, color);
    cv::waitKey(3);
}


void capsicum_segmentation::readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud){

      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  }


void capsicum_segmentation::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }

sensor_msgs::ImagePtr capsicum_segmentation::cvImagetoMsg(cv::Mat &image,std::string encoding){

    static int header_sequence_id = 0;

    cv_bridge::CvImage* cv_msg_ptr = new cv_bridge::CvImage();
    std_msgs::Header header;

    header.seq = header_sequence_id++;
    header.stamp = ros::Time::now();
    header.frame_id = depth_frame_id;

    cv_msg_ptr = new cv_bridge::CvImage(header,encoding,image);

    return cv_msg_ptr->toImageMsg();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fruit_segmentation");
  //ros::NodeHandle nh_("~");

  capsicum_segmentation cap_seg;

  cap_seg.start();

  return 0;
}
