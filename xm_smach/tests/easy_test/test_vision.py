#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.store_lib import *
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math
        
class VisionTest():
    def __init__(self):
        rospy.init_node('test_vision',anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('------------start test----------------')
        self.test_bool = False
        self.test_vision = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.test_vision:
            StateMachine.add('RUN_NODE',
                                RunNode_obj(),
                                transitions={'succeeded':'WAIT','aborted':'aborted'})
            self.test_vision.userdata.rec = 20.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'succeeded','error':'error'})
            # StateMachine.add('CLOSENODE',
            #                     CloseKinect(),
            #                     transitions={'succeeded':'succeeded','aborted':'aborted'})


#            self.test_vision.userdata.pos_xm  =Pose()
#            StateMachine.add('GET_PEOPLE_POS',
#                                FindPeople().find_people_,
#                                transitions ={'invalid':'succeeded','valid':'error','preempted':'aborted'},
#                                remapping = {'pos_xm':'pos_xm'})
        
#        rospy.loginfo(self.test_vision.userdata.pos_xm)
        out = self.test_vision.execute()
#        if out =='error':
#            rospy.logerr('no position past!')
#            self.test_bool = False
#        else:
        self.test_bool = True
    
    def shutdown(self):
        if self.test_bool == True:
            rospy.loginfo('test succeeded')
        else:
            rospy.logerr('test failed')

if __name__ == "__main__":
    try:
        VisionTest()
    except NameError, e:
        rospy.logerr(e)
