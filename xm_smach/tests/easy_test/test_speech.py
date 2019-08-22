#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from subprocess import *
from geometry_msgs.msg import *
import math

class SpeechTest():
    def __init__(self):
        rospy.init_node('speech_test') 
        rospy.on_shutdown(self.shutdown)
        self.test = StateMachine(outcomes=['succeeded','aborted','error'])

        with self.test:
            self.test.userdata.target = list()
            self.test.userdata.action = list()
            self.test.userdata.task_num = 0
            self.test.userdata.sentences = 'i am shall mon'
            #得到命令
            StateMachine.add('GETTASK', 
                             GetTask(), 
                             transitions={'succeeded':'SPEAK','aborted':'aborted'},
                             remapping={'target':'target','action':'action','task_num':'task_num'})
            StateMachine.add('SPEAK',
                             Speak(),
                             transitions={'succeeded':'ANSWER','aborted':'aborted','error':'error'},
                             remapping={'sentences':'sentences'})
            StateMachine.add('ANSWER',
                             Anwser(),
                             transitions={'succeeded':'succeeded','aborted':'aborted'})
        rospy.loginfo(self.test.userdata.target)
        rospy.loginfo(self.test.userdata.action)
        rospy.loginfo(self.test.userdata.task_num)
        intro_server = IntrospectionServer('test', self.test, '/SM_ROOT')
        intro_server.start()
        out = self.test.execute()
        intro_server.stop()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)
if __name__=="__main__":
    try:
        SpeechTest()
    except:
        rospy.logerr('i am a slliby!')
