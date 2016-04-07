#include "shelf_scan.h"
#include "shelf_scan_node.h"
#include "build_kinfu/shelf_scan_srv.h"


bool shelf_scanning(build_kinfu::shelf_scan_srv::Request  &req,
  build_kinfu::shelf_scan_srv::Response &res){

    static tf::TransformListener tf_listener;

    ROS_INFO_STREAM("Shelf pose recieved: " << req.initial_shelf_pose);

    shelf_scan shelf_scan(req.move_group.data, req.initial_shelf_pose);

    try{

      shelf_scan.generatePath();
      shelf_scan.execute();

    }catch(tf::TransformException ex){

      ROS_ERROR("%s",ex.what());
      res.success.data = false;
      return false;

    }

    res.success.data = true;
    return true;

  }


  int main(int argc, char** argv)
  {

    ros::init(argc, argv, "shelf_scan_node");

    ros::NodeHandle nh_("~");

    ros::ServiceServer move_tf_service = nh_.advertiseService("/shelf_scan", &shelf_scanning);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO_STREAM("shelf_scan_node services started!");

    while (ros::ok()){

    }

    return 0;
  }
