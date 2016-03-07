#!/usr/bin/env python  
import roslib
import rospy
import numpy
import math
import tf
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg

pose_count = 0
clear_markers = False


def multiply_translation(trans1,rot1,trans2):
    
    transform1 = numpy.dot(tf.transformations.translation_matrix(trans1),tf.transformations.quaternion_matrix(rot1))
    translation = tf.transformations.translation_matrix(trans2)

    transform = numpy.dot(transform1,translation)
    pose = geometry_msgs.msg.Pose()
    rot = tf.transformations.quaternion_from_matrix(transform)
    trans = tf.transformations.translation_from_matrix(transform)
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    
    return pose

def callback_parser(data):
    global pose_count
    global clear_markers
    pose_count = pose_count + 1
    clear_markers = True
    rospy.loginfo('Got Poses')

if __name__ == '__main__':
    
    global pose_count
    global clear_markers
    print "============ Starting Node"
    rospy.init_node('end_effector_path',anonymous=True)

    listener = tf.TransformListener()
    end_effector_traj = []
    nPoints = 0
    
    marker_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=10)
    markerarray_pub = rospy.Publisher('/visualization_marker_array', visualization_msgs.msg.MarkerArray, queue_size=10)
    sub = rospy.Subscriber('/gripper_pose', geometry_msgs.msg.PoseArray, callback_parser)
    points_array = []
    pose_array = []
    
    
    rate = rospy.Rate(100) # 10hz  
    
    yellow = std_msgs.msg.ColorRGBA()
    blue = std_msgs.msg.ColorRGBA() 
    green = std_msgs.msg.ColorRGBA()
    red = std_msgs.msg.ColorRGBA()
    pose_color = std_msgs.msg.ColorRGBA()
    
    yellow.r = 1.0
    yellow.g = 192.0/255.0
    yellow.b = 0.0
    yellow.a = 1.0
    
    blue.r = 91.0/255.0
    blue.g = 155.0/255.0
    blue.b = 213.0/255.0                
    blue.a = 1.0                
    
    green.r = 112.0/255.0
    green.g = 173.0/255.0
    green.b = 71.0/255.0
    green.a = 1.0
    
    red.r = 239.0/255.0
    red.g = 87.0/255.0
    red.b = 87.0/255.0
    red.a = 1.0    
    
    while not rospy.is_shutdown():
        try:

            (trans,rot) = listener.lookupTransform('/world', '/ee_link', rospy.Time(0))
                      
            nPoints = nPoints + 1
            
#            pose.orientation.
#            pose.orientation.y = 0.0
#            pose.orientation.z = 0.0
            point = geometry_msgs.msg.Point()
            pose = geometry_msgs.msg.Pose()
            point.x = trans[0]
            point.y = trans[1]
            point.z = trans[2]
            
            pose.position = point
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]  
            
            translation = geometry_msgs.msg.Pose()
            toolpoint_pose = geometry_msgs.msg.Pose()
            
            translation_camera = [0.15,0.0,-0.05]
            translation_tool = [0.2,0.0,0.05]
            
            toolpoint_pose = multiply_translation(trans,rot,translation_tool)
            
            pose_array.append(toolpoint_pose)
            points_array.append(toolpoint_pose.position)
            
            if clear_markers:
                rospy.loginfo('Clearing Markers') 
                nPoints = 0
                points_array[:] = []
                pose_array[:] = []
                clear_markers = False
                
            if nPoints > 1:
                points = visualization_msgs.msg.Marker()
                line = visualization_msgs.msg.Marker()
                marker_array = visualization_msgs.msg.MarkerArray()
                
                points.header.frame_id = line.header.frame_id = '/world'
                points.header.stamp = line.header.stamp = rospy.get_rostime()
                points.id = (pose_count % 4) + 4                
                line.id = pose_count % 4
                points.type = points.SPHERE_LIST           
                line.type = line.LINE_STRIP
                points.action = line.action = line.ADD
                points.pose.orientation.w = line.pose.orientation.w = 1.0
                line.scale.x = line.scale.y = line.scale.z = 0.005
                points.color.a = line.color.a = 1.0
                points.scale.x = points.scale.y = points.scale.z = 0.01
                
                if pose_count % 4 == 0:
                    pose_color = red
                    line.color = blue
 
                elif pose_count % 4 == 1:
                    pose_color = red
                    line.color = yellow
                    
                elif pose_count % 4 == 2:
                    pose_color = red
                    line.color = green
                    
                elif pose_count % 4 == 3:
                    pose_color = red
                    line.color = blue
#                
                for i in xrange(0,len(points_array)):   
                    line.points.append(points_array[i])
                
                for i in xrange(0,len(pose_array),40): 
                    marker_arrow = visualization_msgs.msg.Marker()
                    marker_arrow.header.frame_id = '/world'
                    marker_arrow.header.stamp = rospy.get_rostime()            
                    marker_arrow.id = (pose_count % 4) + 8 + i
                    marker_arrow.type = marker_arrow.ARROW    
                    marker_arrow.action = marker_arrow.ADD
                    marker_arrow.pose.position = pose_array[i].position
                    marker_arrow.pose.orientation = pose_array[i].orientation
                    marker_arrow.scale.x = 0.02
                    marker_arrow.scale.y = 0.005
                    marker_arrow.scale.z = 0.005
                    marker_arrow.color = pose_color
#                    marker_arrow.color.r = 1.0
#                    marker_arrow.color.g = 0.0
#                    marker_arrow.color.b = 0.0
#                    marker_arrow.color.a = 1.0                    
#                    rospy.loginfo("%s",marker_arrow)
                    marker_array.markers.append(marker_arrow)
#                    points.points.append(points_array[i])
                    
                    
                    
                marker_pub.publish(line)
                markerarray_pub.publish(marker_array)
            
            
                        
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()
            
