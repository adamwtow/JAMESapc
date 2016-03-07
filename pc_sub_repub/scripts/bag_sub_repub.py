#!/usr/bin/env python
# Written by Inkyu, enddl22@gmail.com
# Input: PointCloud2
# Import required modules
import numpy
import roslib
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
import std_msgs.msg

def pc_callback(data):
    #print "pc_callback"
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'camera_depth_optical_frame'
    pc_msg = PointCloud2()
    pc_msg = data
    pc_msg.header = h
    pub_pointCloud.publish(pc_msg)

def image_callback(data):
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'camera_rgb_optical_frame'
    image_msg = Image()
    image_msg  = data
    image_msg .header = h
    pub_image.publish(image_msg)

def listener():
    topic = "/camera/depth_registered/points"
    topic_image = "/camera/rgb/image_raw"
    rospy.Subscriber(topic, PointCloud2, pc_callback)
    rospy.Subscriber(topic_image, Image, image_callback)
    rospy.spin()

# Main
if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    pub_pointCloud = rospy.Publisher('/camera/depth_registered/points_', PointCloud2, queue_size=10)
    pub_image = rospy.Publisher('/camera/rgb/image_raw_', Image, queue_size=10)
    listener()
