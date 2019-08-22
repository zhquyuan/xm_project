#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal, MoveBaseActionFeedback

class findPeople(State):
    