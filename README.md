# README #

1.	Initialising the UR5

The only thing you need to do is turn the power button on and then go to the initialisation screen using the touch screen. I usually put in 0.2Kg for the load but 0.0kg is fine also and then just hit the start button a couple of times until you here that clicking sound of the arm unlatching its electrical brakes. Keep the UR5 tablet near you in order to hit the emergency button if needed.

2.	Start UR5 Ros node

Command: roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]

There is normally some errors thrown to the screen as the driver doesn’t support our version of the robot
It should be working if it ends in something like hello world message
If it has the waiting to program or something else you will either have to reboot the UR5 machine or ssh into it and run the ./stopurcontrol.sh script then the ./starturcontrol.sh script

3.	Set up the camera node

Command: roslaunch realsense_camera realsense_camera.launch

Other option is: roslaunch realsense_f200 realsense_image_viewer.launch   (I think this is right)

I think you are familiar with how unreliable this node is so try your best and fingers crossed it works. Alternatively you can try the realsense_camera_modified package by removing the CATKIN_IGNORE file.

4.	Launch the moveit node

Command: roslaunch capsicum_detection harvey_moveit.launch

This node is usually stable

5.	Launch the Kinect fusion and capsicum detection nodes

Command: roslaunch capsicum_detection capsicum_detection_no_realsense.launch

This is the kinect fusion ros node and the capsicum detection/segmentation node

6.	Launch the superquadric fitter node (unfortunately the launch file isn’t working, not sure why)

Command: rosrun superqaudric_fitter superquadric_fitter

7.	Launch the grasp experiment node 

Command: rosrun fruit_picking grasp_pose_experiment

