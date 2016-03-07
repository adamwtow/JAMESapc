#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


got_pose = None
pose_array = None

def callback_parser(data):
    global pose_array
    global got_pose
    pose_array = data
    got_pose = True
    rospy.loginfo('Got Poses')

def convert_array(data):
    
    marker_array = MarkerArray()
    
    for i in xrange(len(data.poses)):
        
        marker = Marker()         
        marker.header.frame_id = data.header.frame_id;
        marker.header.stamp = rospy.get_rostime();
        marker.id = i;
        marker.type = marker.ARROW;
        marker.action = marker.ADD;
        marker.pose = data.poses[i];
        marker.scale.x = 0.07;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
                
        marker_array.markers.append(marker)
    ##convert data to marker_array
    return marker_array
 

if __name__ == '__main__':
    global got_pose
    global pose_array
    
    rospy.init_node('marker_parser', anonymous=True)
    sub = rospy.Subscriber('/gripper_pose', PoseArray, callback_parser)
    pub = rospy.Publisher('/marker_poses', MarkerArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz    
    
    while not rospy.is_shutdown():
        if got_pose == True:
            ##rospy.loginfo('Converting Pose')
            pub.publish(convert_array(pose_array))
        rate.sleep()
    