#include "shelf_scan.h"
#include "shelf_scan_node.h"
#include "shelf_scan/shelf_scan_srv.h"


bool shelf_scan_callback(shelf_scan::shelf_scan_srv::Request  &req,
  shelf_scan::shelf_scan_srv::Response &res){

    static tf::TransformListener tf_listener;

    ROS_INFO_STREAM("Shelf pose recieved: " << req.initial_shelf_pose);

    shelf_scanner shelf_scanner(req.move_group.data, req.initial_shelf_pose);


    bool success = shelf_scanner.execute();

    if (!success) {

      res.success.data = false;

    } else {

      res.success.data = true;

    }
    return true;
  }


  int main(int argc, char** argv)
  {

    ros::init(argc, argv, "shelf_scan_node");

    ros::NodeHandle nh_("~");

    ros::ServiceServer scan_service = nh_.advertiseService("/shelf_scan", &shelf_scan_callback);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO_STREAM("shelf_scan_node services started!");

    while (ros::ok()){

    }

    return 0;
  }
