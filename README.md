# README #

Prior: Compile and install [PCLapc](http://github.com/jamessergeant/PCLapc.git), librealsense, other dependencies

1. `sudo ln -s ~/librealsense/include/librealsense/ /usr/local/include`
1. In ros_ws/src: `git clone http://github.com/jamessergeant/JAMESapc.git`
2. Ensure all CustomPCLConfig.cmake files point to the correct PCL install location e.g. `~/PCLapc/install/share/pcl-1.8`
2. `cd ~/ros_ws`
3. `catkin_make`
4. `source devel/setup.bash`
5. Test: `roslaunch ros_kinfu ros_kinfu`
