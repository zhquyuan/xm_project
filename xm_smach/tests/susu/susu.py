#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.help_me_carry_lib import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math
import subprocess

class Susu():
    def __init__(self):
        rospy.init_node('susu')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        self.susu = StateMachine(['succeeded','aborted','error'])
        with self.susu:
            self.susu.userdata.rec = 10.0
            self.susu.userdata.sentences = 'how many women in this house?'
            self.susu.userdata.welcome = 'you are not my master!'
            self.susu.userdata.target = list()
            self.susu.userdata.action = list()
            StateMachine.add('Listen',
                                GetSignal(),
                                transitions={'succeeded':'See','aborted':'See','error':'error'})
            StateMachine.add('See',
                                RunNode(),
                                transitions={'succeeded':'Wait','aborted':'Wait'})
            StateMachine.add('Wait',
                                Wait(),
                                transitions={'succeeded':'Close','error':'error'},
                                remapping={'rec':'rec'})
            StateMachine.add('Close',
                                CloseKinect(),
                                transitions={'succeeded':'Welcome','aborted':'Welcome'})
            # StateMachine.add('Ask',
            #                     Speak(),
            #                     transitions={'succeeded':'Listen1','aborted':'Listen1','error':'error'})
            # StateMachine.add('Listen1',
            #                     GetSignal(),
            #                     transitions={'succeeded':'Welcome','aborted':'Welcome','error':'error'})
            # # StateMachine.add('Wait1',
            # #                     Wait(),
            # #                     transitions={'succeeded':'Welcome','error':'error'})
            StateMachine.add('Welcome',
                                Speak(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping={'sentences':'welcome'})     
        
        out = self.susu.execute()                  
        rospy.logerr(out)
    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn('xm is the best!')
        else:
            rospy.logwarn('sorry, master')
if __name__=='__main__':
    try:
        Susu()
    except Exception,e:
        rospy.logerr(e)