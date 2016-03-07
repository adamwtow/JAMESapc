# README #

Prior: Compile and install [PCLapc](http://github.com/jamessergeant/PCLapc.git), other dependencies

1. In ros_ws/src: `git clone http://github.com/jamessergeant/HARVEYapc.git`
2. Ensure all CustomPCLConfig.cmake files point to the correct PCL install location
2. `cd ~/ros_ws`
3. `catkin_make`
4. `source devel/setup.bash`
5. Test: `roslaunch ros_kinfu ros_kinfu`
