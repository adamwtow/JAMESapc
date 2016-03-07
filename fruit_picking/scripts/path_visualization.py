#!/usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
#from geometry_msgs.msg import PoseArray
#from visualization_msgs.msg import MarkerArray
#from visualization_msgs.msg import Marker
group = None
robot = None
scene = None
display_trajectory_publisher = None

def displayPath(data):
    global group
    global robot
    global scene
    global display_trajectory_publisher
    
    rospy.loginfo('Got Poses')
#    robot = moveit_commander.RobotCommander()
#    scene = moveit_commander.PlanningSceneInterface()
#    group = moveit_commander.MoveGroupCommander('manipulator')
    trajectory_msg = moveit_msgs.msg.RobotTrajectory()      ## eef_step ## jump_threshold
      
#      fraction_complete = group.computeCartesianPath(waypoints, eef_step_size,  jump_threshold, trajectory_msg);
# The trajectory needs to be modified so it will include velocities as well.
# First to create a RobotTrajectory object
    current_state = robot.get_current_state()
#    rt = robot.RobotTrajectory(current_state.getRobotModel(), 'manipulator')
    
    waypoints = []
    
    # start with the current pose
    waypoints.append(group.get_current_pose().pose)
    
    for i in xrange(len(data.poses)):    
        waypoints.append(data.poses[i])
    # first orient gripper and move forward (+x)
#    wpose = geometry_msgs.msg.Pose()
#    wpose.orientation.w = 1.0
#    wpose.position.x = waypoints[0].position.x + 0.1
#    wpose.position.y = waypoints[0].position.y
#    wpose.position.z = waypoints[0].position.z
#    waypoints.append(copy.deepcopy(wpose))
    
    # second move down
#    wpose.position.z -= 0.10
#    waypoints.append(copy.deepcopy(wpose))
    
    # third move to the side
#    wpose.position.y += 0.05
#    waypoints.append(copy.deepcopy(wpose))

    
    (plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
    
    rospy.sleep(5)

    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan3)
    display_trajectory_publisher.publish(display_trajectory);
    
    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(5)
    
## Second get a RobotTrajectory from trajectory
#      rt.setRobotTrajectoryMsg(group.getCurrentState(), trajectory_msg)
    
          ## Thrid create a IterativeParabolicTimeParameterization object
#      trajectory_processing::IterativeParabolicTimeParameterization iptp;
    
          ## Fourth compute computeTimeStamps
#    bool success = iptp.computeTimeStamps(rt,velocity_scale);
#          ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    
          ## Get RobotTrajectory_msg from RobotTrajectory
#          rt.getRobotTrajectoryMsg(trajectory_msg);
    
          ## Finally plan and execute the trajectory
#          ur5_plan.trajectory_ = trajectory_msg;
#          ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction_complete * 100.0);
    
#          ur5.execute(ur5_plan);



if __name__ == '__main__':
    
    print "============ Starting Node"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('path_visulization',anonymous=True)
#    
    robot = moveit_commander.RobotCommander()   
#    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander('manipulator')
#    display_traj_pub = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,  queue_size=10)
    
    sub = rospy.Subscriber('/gripper_pose', geometry_msgs.msg.PoseArray, displayPath)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,  queue_size=10)
                                    
    rate = rospy.Rate(10) # 10hz    
    while not rospy.is_shutdown():
            rate.sleep()