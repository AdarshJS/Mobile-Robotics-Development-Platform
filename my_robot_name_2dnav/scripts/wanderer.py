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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import sys
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# import numpy as np
# from sympy import *
# from sympy.geometry import *

dist = [1000,1000,1000]
pos_r1 = [0,0,0]
pos_r2 = [0,0,0]
pos_r3 = [0,0,0]

def callback_odom1(data):
    global pos_r1
    q = data.pose.pose.orientation
    theta = 2.0*math.atan2(q.z,q.w)
    rospy.loginfo(rospy.get_caller_id()+" I heard %f,%f,%f",data.pose.pose.position.x, data.pose.pose.position.y, theta)
    pos_r1 = [data.pose.pose.position.x, data.pose.pose.position.y, theta]

def callback_odom2(data):
    global pos_r2
    q = data.pose.pose.orientation
    theta = 2.0*math.atan2(q.z,q.w)
    rospy.loginfo(rospy.get_caller_id()+" I heard %f,%f,%f",data.pose.pose.position.x, data.pose.pose.position.y, theta)
    pos_r2 = [data.pose.pose.position.x, data.pose.pose.position.y, theta]

def callback_odom3(data):
    global pos_r3
    q = data.pose.pose.orientation
    theta = 2.0*math.atan2(q.z,q.w)
    rospy.loginfo(rospy.get_caller_id()+" I heard %f,%f,%f",data.pose.pose.position.x, data.pose.pose.position.y, theta)
    pos_r3 = [data.pose.pose.position.x, data.pose.pose.position.y, theta]

def callback_scan1(data):
    global dist
    rospy.loginfo(rospy.get_caller_id()+" I heard %f", data.ranges[0])
    dist[0] = data.ranges[0]

def callback_scan2(data):
    global dist
    rospy.loginfo(rospy.get_caller_id()+" I heard %f", data.ranges[0])
    dist[1] = data.ranges[0]

def callback_scan3(data):
    global dist
    rospy.loginfo(rospy.get_caller_id()+" I heard %f", data.ranges[0])
    dist[2] = data.ranges[0]

def collision_check(posA, posB, posC):
    distance1 = pow((posA[0]-posB[0])**2+(posA[1]-posB[1])**2,0.5)
    distance2 = pow((posA[0]-posC[0])**2+(posA[1]-posC[1])**2,0.5)

    if min(distance1,distance2) < 2*0.15 and distance1<=distance2:
        return 1
    elif min(distance1,distance2) < 2*0.15 and distance2<=distance1:
        return 2
    else:
        return 0

def wanderer():
    global dist, pos_r1, pos_r2, pos_r3
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('wanderer', anonymous=True)

    rospy.Subscriber("/robot1/odom", Odometry, callback_odom1)
    rospy.Subscriber("/robot1/scan", LaserScan, callback_scan1)
    rospy.Subscriber("/robot2/odom", Odometry, callback_odom2)
    rospy.Subscriber("/robot2/scan", LaserScan, callback_scan2)
    rospy.Subscriber("/robot3/odom", Odometry, callback_odom3)
    rospy.Subscriber("/robot3/scan", LaserScan, callback_scan3)
    pub1 = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size = 10)
    pub2 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size = 10)
    pub3 = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size = 10)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cmd1 = Twist()
        c = collision_check(pos_r1,pos_r2,pos_r3)
        if dist[0] < 0.3:
            # Obstacle constraint
            cmd1.angular.z = 2
        elif c==1:
            # robot2 is closeby
            # if math.atan((pos_r2[1]-pos_r1[1])/(pos_r2[0]-pos_r1[0]))-
            cmd1.linear.x = -0.1
            cmd1.angular.z = 2
        elif c==2:
            # robot3 is closeby
            cmd1.linear.x = -0.1
            cmd1.angular.z = -2
        else:
            cmd1.linear.x = 0.08
        pub1.publish(cmd1)
        cmd2 = Twist()
        c = collision_check(pos_r2,pos_r3,pos_r1)
        if dist[1] < 0.3:
            # Obstacle constraint
            cmd2.angular.z = 2
        elif c==1:
            # robot3 is closeby
            cmd2.angular.z = 2
            cmd2.linear.x = -0.1
        elif c==2:
            # robot1 is closeby
            cmd2.linear.x = -0.1
            cmd2.angular.z = -2
        else:
            cmd2.linear.x = 0.08
        pub2.publish(cmd2)
        cmd3 = Twist()
        c = collision_check(pos_r3,pos_r1,pos_r2)
        if dist[2] < 0.3:
            # Obstacle constraint
            cmd3.angular.z = 2
        elif c==1:
            # robot1 is closeby
            cmd3.angular.z = 2
            cmd3.linear.x = -0.1
        elif c==2:
            # robot2 is closeby
            cmd3.linear.x = -0.1
            cmd3.angular.z = -2
        else:
            cmd3.linear.x = 0.08
        pub3.publish(cmd3)
        r.sleep()
        
if __name__ == '__main__':
    # if len(sys.argv) < 2:
    #     print "Usage: wanderer.py <robot_name> ", len(sys.argv)
    #     exit()
    # ns = sys.argv[1]
    # print ns
    wanderer()
