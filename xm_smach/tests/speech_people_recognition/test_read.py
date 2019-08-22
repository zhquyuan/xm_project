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
            #这里用Pose类型代替传输三个数据
            #第一个是总人数
            #第二个是站着的人数
            #第三个是坐着的人数
            self.find_people.userdata.people_condition = {'All':0,'Female':0,'Male':0,'seatedM':0,'seatedF':0,'standM':0,'standF':0}
            self.find_people.userdata.sentences = ''
            StateMachine.add('GET_PEOPLE_NUM',
                                CountPeople(),
                                transitions={'succeeded':'GET_SENTENCE','aborted':'aborted'})
            StateMachine.add('GET_SENTENCE',
                                GetSentences(),
                                transitions={'succeeded':'SPEAK_SIZE','error':'error'},
                                remapping = {'sentences':'sentences'})
            StateMachine.add('SPEAK_SIZE',
                                Speak(),
                                transitions={'succeeded':'succeeded','error':'error'},
                                remapping={'sentences':'sentences'})
            
    
        
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

