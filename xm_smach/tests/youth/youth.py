#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess  


class Youth():
    def __init__(self):
        rospy.init_node('Youth')
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('------------start test----------------')
        self.test_bool = False

        self.run = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.run:
            self.run.userdata.point1 = gpsr_target['diningroom']['pos']
            self.run.userdata.point2 = gpsr_target['bedroom']['pos']
            self.run.userdata.point3 = gpsr_target['kitchen']['pos']
            self.run.userdata.point4 = gpsr_target['door']['pos']

            StateMachine.add('NAV1',
                                NavStack(),
                                transitions={'succeeded':'NAV2','aborted':'NAV1','error':'error'},
                                remapping = {'pos_xm':'point1'})
            StateMachine.add('NAV2',
                                NavStack(),
                                transitions={'succeeded':'NAV3','aborted':'NAV2','error':'error'},
                                remapping={'pos_xm':'point2'})
            StateMachine.add('NAV3',
                                NavStack(),
                                transitions={'succeeded':'NAV4','aborted':'NAV3','error':'error'},
                                remapping={'pos_xm':'point3'})
            StateMachine.add('NAV4',
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'NAV4','error':'error'},
                                remapping={'pos_xm':'point4'})
          
        self.youth = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.youth:
            self.youth.userdata.rec = 10.0
            self.youth.userdata.go_point=Point(1.5,0.0,0.0)
            self.youth.userdata.arrive_sentences = 'fight for the bright tomorrow'
            StateMachine.add('RUN',
                                self.run,
                                transitions={'succeeded':'DOORDETECT','aborted':'aborted','error':'error'})
            StateMachine.add('DOORDETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOORDETECT','preempted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove_move(),
                                remapping ={'point':'go_point'},
                                transitions={'succeeded':'SPEAK_ARRIVE','aborted':'SPEAK_ARRIVE','error':'error'})
            StateMachine.add('SPEAK_ARRIVE',
                                Speak(),
                                remapping = {'sentences':'arrive_sentences'},
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
  
        out = self.youth.execute()
        print out
        if out =='succeeded':
            self.test_bool = True
    def shutdown(self):
        if self.test_bool == True:
            rospy.loginfo('test succeeded')
        else:
            rospy.logerr('test failed')

if __name__ == '__main__':
    try:
        Youth()
    except Exception,e:
        rospy.logerr(e)