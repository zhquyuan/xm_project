#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from smach import State, StateMachine, UserData, Concurrence
from xm_smach.gpsr_lib import *
from geometry_msgs.msg import *
from xm_smach.common_lib import *
from xm_smach.help_me_carry_lib import *
from subprocess import *
class sm_Follow():
    def __init__(self):
        rospy.init_node('sm_Follow')

        self.trace = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'}},
#                                 outcome_cb = self.trace_out_cb,
                                 child_termination_cb = self.trace_child_cb)
        with self.trace:
            self.meta_follow = StateMachine(['succeeded','aborted'])
            with self.meta_follow:
                StateMachine.add('FOLLOW',
                                    SimpleFollow(),
                                    transitions = {'succeeded':'FINDPEOPLE','aborted':'aborted'})
            Concurrence.add('RUNNODE',RunNode())
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())

        out = self.trace.execute()
        print out
            
    def trace_child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            subprocess.call('xterm -e rosnode kill people_tracking &', shell = True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['FOLLOW']:
            rospy.logerr('the follow state meet error!')
            return True
        return False
    #     rospy.logerr('byebye')
        out = self.xm_follow.execute()
        if out == 'succeeded':
            rospy.logwarn('test succeeded')


if __name__ =="__main__":
    try:
        sm_Follow()
    except Exception,e:
        rospy.logerr(e)



