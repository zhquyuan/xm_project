#!/usr/bin/env python
# encoding:utf8

# 语音与人脸识别项目测试程序最终版
# 将原本前后5个问题的分布改成了一直循环
# 没有加入麦克风阵列，等待改进
# 状态机顺序：开场白->等待测试人员排成一排->转身->人脸识别->汇报识别情况->请求提问->回答问题(10个)
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

        self.answer_sql = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.answer_sql:
            self.answer_sql.userdata.sentences='please ask me'
            StateMachine.add('GET_ANGLE',
                                Get_angle_sql(),
                                transitions={'succeeded':'TURN','aborted':'SPEAK'},
                                remapping={'angle':'angle'})
            StateMachine.add('TURN',
                                TurnDegree(),
                                transitions={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping={'degree':'angle'})
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'succeeded','error':'error'})
            # StateMachine.add('ANSWER',
            #                     Anwser(),
            #                     transitions={'succeeded':'succeeded','aborted':'aborted'})

        self.find_people = StateMachine(outcomes = ['succeeded','aborted','error'],input_keys = ['people_condition'],output_keys = ['people_condition'])
        self.test_bool = False

        with self.find_people:
            #这里用Pose类型代替传输三个数据
            #第一个是总人数
            #第二个是站着的人数
            #第三个是坐着的人数
            
            self.find_people.userdata.sentences = ''
            self.find_people.userdata.rec = 10.0
            StateMachine.add('RUNNODE',
                                RunNode_Num(),
                                transitions={'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'GET_PEOPLE_NUM','error':'error'})
            StateMachine.add('GET_PEOPLE_NUM',
                                CountPeople(),
                                transitions={'succeeded':'GET_SENTENCE','aborted':'aborted'})
            StateMachine.add('GET_SENTENCE',
                                GetSentences(),
                                transitions={'succeeded':'SPEAK_SIZE','error':'error'},
                                remapping = {'sentences':'sentences'})
            StateMachine.add('SPEAK_SIZE',
                                Speak(),
                                transitions={'succeeded':'CLOSE_KINECT','error':'error'},
                                remapping={'sentences':'sentences'})
            
            StateMachine.add('CLOSE_KINECT',
                                CloseKinect(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
            

        self.speech_rec = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.speech_rec:
            self.speech_rec.userdata.sentences = 'i want to play a riddle game'
            self.speech_rec.userdata.wait = 17.0
            self.speech_rec.userdata.degree = 3.1416
            self.speech_rec.userdata.people_condition = {}
            StateMachine.add('SPEAK_1',
                                Speak(),
                                transitions={'succeeded':'WAIT_1', 'aborted':'SPEAK_1', 'error':'error'},
                                remapping={'sentences':'sentences'})
            StateMachine.add('WAIT_1',
                                Wait(),
                                transitions={'succeeded':'TURN','error':'error'},
                                remapping={'rec':'wait'})
            StateMachine.add('TURN',
                                TurnDegree(),
                                transitions={'succeeded':'RECO','aborted':'','error':'error'},
                                remapping={'degree':'degree'})
            StateMachine.add('RECO',
                                self.find_people,
                                transitions={'succeeded':'SPEAK_2','aborted':'aborted','error':'error'})
                        
            self.speech_rec.userdata.sentences = 'who want to play a riddle game with me'
            StateMachine.add('SPEAK_2',
                                Speak(),
                                transitions={'succeeded':'WAIT_2', 'aborted':'SPEAK_2', 'error':'error'},
                                remapping={'sentences':'sentences'})
            self.speech_rec.userdata.wait = 5.0
            StateMachine.add('WAIT_2',
                                Wait(),
                                transitions={'succeeded':'ANSWER','error':'error'},
                                remapping={'rec':'wait'})


            StateMachine.add('ANSWER',
                                Anwser(),
                                transitions={'succeeded':'ANSWER','aborted':'ANSWER'})
            # self.answer = Iterator(outcomes=['succeeded','aborted','error'],
            #                         input_keys=['people_condition'],
            #                         output_keys=[],
            #                         it =lambda:range(0,5),
            #                         exhausted_outcome='succeeded')
            # with self.answer:

            #     Iterator.set_contained_state('ANSWER_STATE',
            #                                     Anwser(),
            #                                     loop_outcomes = ['succeeded'])
            # StateMachine.add('ANSWER',
            #                     self.answer,
            #                     transitions={'succeeded':'WAIT_3','aborted':'aborted','error':'error'})
            # StateMachine.add('WAIT_3',
            #                     Wait(),
            #                     transitions={'succeeded':'ANSWER2','error':'error'},
            #                     remapping={'rec':'wait'})
            # self.answer_again = Iterator(outcomes=['succeeded','aborted','error'],
            #                         input_keys=['people_condition'],
            #                         output_keys=[],
            #                         it =lambda:range(0,5),
            #                         exhausted_outcome='succeeded')
            # with self.answer_again:

            #     Iterator.set_contained_state('ANSWER_STATE',
            #                                     self.answer_sql,
            #                                     loop_outcomes = ['succeeded'])
            # StateMachine.add('ANSWER2',
            #                     self.answer_again,
            #                     transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

        
        
        out = self.speech_rec.execute()
        if out  == 'succeeded':
            self.test_bool = True
            rospy.logerr('test finished, all thing is done well!')

    def shutdown(self):
        if self.test_bool == False:
            
            rospy.logerr('test failed, there is no chicken dinner!')

if __name__ == "__main__":
    try:
        SpeechRecognition()
    except:
        rospy.logerr('test wrong!')
        subprocess.call("rosnode kill -a")

