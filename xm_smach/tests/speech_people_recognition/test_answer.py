#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.speech_reco_lib import *
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math


class SpeechRecognition():
    def __init__(self):
        rospy.init_node('speech_recognition')
        rospy.on_shutdown(self.shutdown)
        self.find_people = StateMachine(outcomes = ['succeeded','aborted','error'])
        self.test_bool = False
        with self.find_people:
            self.answer = Iterator(outcomes=['succeeded','aborted','error'],
                                    input_keys=['people_condition'],
                                    output_keys=[],
                                    it =lambda:range(0,5),
                                    exhausted_outcome='succeeded')
            with self.answer:

                Iterator.set_contained_state('ANSWER_STATE',
                                                Anwser(),
                                                loop_outcomes = ['succeeded'])
            StateMachine.add('ANSWER',
                                self.answer,
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
    
        
        out = self.find_people.execute()
        if out  == 'succeeded':
            self.test_bool = True
            rospy.logwarn('test finished, all thing is done well!')
        else:
            rospy.logerr('test failed ,something goes wrong')

    def shutdown(self):
        if self.test_bool == False:
            rospy.logerr('test failed, there is no chicken dinner!')

if __name__ == "__main__":
    try:
        SpeechRecognition()
    except:
        rospy.logerr('test wrong!')

