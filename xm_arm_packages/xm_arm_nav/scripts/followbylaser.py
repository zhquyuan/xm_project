#!/usr/bin/env python
import rospy
from roslib import message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Follower():
    def __init__(self):
        rospy.init_node('follow')