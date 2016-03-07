By Inkyu, enddl22@gmail.com

#Command lines

roscore
roslaunch realsense_camera realsense_frames.launch
roslaunch ur_gazebo ur5.launch limited:=true
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch limited:=true sim:=true config:=true
rosrun pc_sub_repub pc_sub_repub.py

and subscribe /camera/depth_registered/points_ in rviz (Note that there is the underscore at the end!!!)

#Some other useful command lindes

rosparam set use_sim_time true
rosbag filter 2015-10-29-13-04-18.bag 2015-10-29-13-04-18_wo_tf.bag "topic != '/tf'"



