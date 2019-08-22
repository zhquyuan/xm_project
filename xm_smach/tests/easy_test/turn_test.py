#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import TurnDegree
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math
        
class TestTurn():
    def __init__(self):
        rospy.init_node('turn_test',anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('started test the turn')

        self.test_bool = False

        self.test_turn = StateMachine(outcomes=['succeeded','aborted','error'])

        with self.test_turn:
            self.test_turn.userdata.degree =  -3.1415926/2
            StateMachine.add('TURN',
                               TurnDegree(),
                               transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                               remapping={'degree':'degree'})
        
        out = self.test_turn.execute()
        rospy.loginfo(out)
        if out == 'succeeded':
            self.test_bool = True
    
    def shutdown(self):
        if self.test_bool == True:
            rospy.logerr('test succeeded')
        else:
            rospy.logerr('test failed')

if __name__ == "__main__":
    try:
        TestTurn()
    except:
        rospy.logerr('i am slliby!')