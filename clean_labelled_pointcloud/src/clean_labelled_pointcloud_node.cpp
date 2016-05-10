#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>

class CleanPointCloud
{
public:
  CleanPointCloud(ros::NodeHandle nh_){

    segmented_sub_=nh_.subscribe("/segmentation_node/segmented_pointcloud",1,&CleanPointCloud::clean_callback,this);
    clean_pub_=nh_.advertise<sensor_msgs::PointCloud2>("object_pointcloud",1);
  }
  ~CleanPointCloud(){}

  void clean_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr)
  {

    std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> > cluster_map;

    pcl::PointCloud<pcl::PointXYZL > labeled_cloud;
    pcl::fromROSMsg(*cloud_ptr, labeled_cloud);

    BOOST_FOREACH(pcl::PointXYZL point, labeled_cloud)
    {
      if(point.label==0) continue;
      //check if cluster map exists for label
      //if not: create it
      std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> >::iterator iter;

      iter=cluster_map.find(point.label);

      if (iter == cluster_map.end()){
        pcl::PointCloud<pcl::PointXYZL> cluster;
        cluster.push_back(point);
        cluster_map.insert(std::make_pair(point.label,cluster));
        ROS_INFO_STREAM(point.label);
      }//just add the point
      else{
        iter->second.push_back(point);
      }

    }

    std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> >::iterator iter;
    iter=cluster_map.begin();

    sensor_msgs::PointCloud2 clean_cloud;

    for (;iter!=cluster_map.end();iter++){

      if (iter->first != 1) continue;

      pcl::toROSMsg(iter->second,clean_cloud);

    }
    clean_pub_.publish(clean_cloud);
  }

private:

    ros::NodeHandle nh_;
        ros::Subscriber segmented_sub_;
        ros::Publisher clean_pub_;

};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "clean_labelled_pointcloud_node");
  ros::NodeHandle nh_;

  CleanPointCloud cpl(nh_);

  while (ros::ok()){
    ros::spinOnce();
  }

}
