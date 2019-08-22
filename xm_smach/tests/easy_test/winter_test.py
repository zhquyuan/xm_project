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
from xm_smach.target_gpsr import gpsr_target
import math
        
class WinterTest():
    def __init__(self):
        rospy.init_node('xm_winter_test') 
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('now we will start a long battle, game start!')

        self.smach_bool = False 

        self.xm_EnterRoom = StateMachine(outcomes=['succeeded','aborted','error'])
        #这部分移植学长代码
        with self.xm_EnterRoom:
            # 进门的状态机:门开关检测->等待->冲进房间->导航到指定地点->succeeded
            # if use arm, use this state
            # self.xm_EnterRoom.userdata.arm_mode_1 =0
            # self.xm_EnterRoom.userdata.arm_ps = PointStamped()
            # StateMachine.add('NAV_POSE',
            #                     ArmCmd(),
            #                     transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
            #                      remapping ={'arm_ps':'arm_ps','mode':'arm_mode_1'})

            # 下列是用到的userdata
            # 等待时间
            self.xm_EnterRoom.userdata.rec = 3.0
            # xm冲进房间的坐标
            self.xm_EnterRoom.userdata.go_point = Point(1.5,0.0,0.0)
            # 与发令者对话坐标
            self.xm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            # 自我介绍的话
            self.xm_EnterRoom.userdata.sentences = 'I am sil meng, you can command me' 
            
            
            # 检测门的状态是开门还是关门  
            StateMachine.add('DOOR_DETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'aborted'})

            # 在刷新建图后等待一段时间 
            # 使用userdata: rec
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})

            # 在开启导航之前首先让xm冲进房间
            # 使用userdata: go_point
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove(),
                                remapping ={'point':'go_point'},
                                transitions={'succeeded':'NAV_1','aborted':'NAV_1','error':'error'})

            # 到房间里的指定地点接受gpsr命令
            # 使用userdata: start_waypoint
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'SPEAK1','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})

            # 介绍自己
            # 使用userdata: sectences                  
            StateMachine.add('SPEAK1',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences'})
        
        
        self.xm_Nav = StateMachine(outcomes=['succeeded','aborted','error'],
                                   input_keys=['target', 'current_task'])
        with self.xm_Nav:
            # userdata 列表
            # xm的位置信息,由GetTarget()得到
            self.xm_Nav.userdata.pos_xm = Pose()
            # 这个target_mode == 1返回Pose()，如果== 0 返回名字
            self.xm_Nav.userdata.target_mode = 1
            #这里的不可能产生aborted
            StateMachine.add('GETTAGET_POSE',
                                GetTarget(),
                                transitions={'succeeded':'NAV_GO', 'aborted':'aborted', 'error':'error'},
                                remapping={'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'})
            #如果找不到路径继续执行NavStack的execute(),知道找到为止
            StateMachine.add('NAV_GO', 
                                NavStack(),
                                transitions={'succeeded':'SPEAK','aborted':'NAV_GO','error':'error'},
                                remapping={'pos_xm':'pos_xm'})
            #当运行命令发生错误时出现aborted情况
            self.xm_Nav.userdata.sentences = 'I have arrived here now, you can ask me some question'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'error':'error'},
                                remapping={'sentences':'sentences'})
            
        self.xm_Find = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys=['target','current_task'])
##########################################################################################################
###################         这里比较复杂,需要考虑一下好好写                 ###################################
##########################################################################################################
###################         检测到一个人,导航到这个人的面前                 ###################################
################## 在房间里转圈->直到找到人->得到人的坐标->导航到人->succeeded ##################################
#########################################################################################################
###################难点:在房间里转圈                                     ###################################
###################    转圈和寻找人要在一个容器里                          ###################################
###################    通过FindPeople().find_people_计算得到的xm位姿有可能无法导航到###########################
#########################################################################################################
###################注意:运行FindPeople()必须先RunNode(),上述部分问题可能在  ###################################
###################    people_tracking有解决,注意和图像组沟通             ###################################
#########################################################################################################
        with self.xm_Find:
#            self.xm_Find.userdata.degree = 45.0
            self.xm_Find.userdata.rec = 2.0
            # run the people_tracking node
            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'succeeded'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                remapping ={'rec':'rec'})
            #the data should be PointStamped() 
            # 这里必须先运行节点，也就是用RunNode()状态
            self.xm_Find.userdata.pos_xm  =Pose()
            StateMachine.add('GET_PEOPLE_POS',
                                FindPeople().find_people_,
                                transitions ={'invalid':'NAV_PEOPLE','valid':'SPEAK','preempted':'aborted'},
                                remapping = {'pos_xm':'pos_xm'})
            # this state will use the userdata remapping in the last state  
            StateMachine.add('NAV_PEOPLE',
                                NavStack(),
                                transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                remapping = {'pos_xm':'pos_xm'})
            self.xm_Find.userdata.sentences = 'you can ask me some question'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions = {'succeeded':'CLOSEKINECT','error':'error'},
                                remapping = {'sentences':'sentences'})

            # close the KinectV2
            StateMachine.add('CLOSEKINECT',
                                CloseKinect(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
            
            
#            StateMachine.add('TURN',
#                                TurnDegree(),
#                                transitions={'succeeded':'TURN','aborted':'aborted','error':'error'}
#                                remapping={'degree':'degree'})
#            self.xm_Find_people = Concurrence(outcomes = ['succeeded', 'people', 'error'],
#                                                default_outcome='succeeded',
#                                                child_termination_cb=self.turn_termination_cb,
#                                                outcome_cb = self.turn_outcome_cb)
#            with xm_Find_people:
#                Concurrence.add('TURN',
#                                TurnDegree(),
#                                remapping={'degree':'degree'})

#########################################################################################################            

        self.test = StateMachine(outcomes=['succeeded','aborted','error'])

        with self.test:
            # 测试总状态机
            # userdata 列表
            # 目标列表,用于获取坐标等操作
            self.test.userdata.target = list()
            # 动作列表,用于NextDo检测下一步的状态
            self.test.userdata.action = list()
            # 所有的测试数(动作数),由GetTask得到
            self.test.userdata.task_num = 0
            # 目前进行到的测试(动作)
            self.test.userdata.current_task = -1
            
            # 进门到指定地点
            StateMachine.add('ENTER_ROOM',
                                self.xm_EnterRoom,
                                transitions={'succeeded':'RECEIVE_TASK', 'aborted':'RECEIVE_TASK','error':'error'})
            # 得到命令,如果没有得到指令则重新运行这个状态
            StateMachine.add('RECEIVE_TASK', 
                                GetTask(), 
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASK','error':'error'},
                                remapping={'target':'target','action':'action','task_num':'task_num'})
            # 循环运行这个状态,得到下一步要做的事情
            StateMachine.add('GET_NEXT_TASK',
                                NextDo(),
                                transitions ={'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            'error':'error',
                                            'find':'FIND',
                                            'go':'GO',
                                            'answer':'ANSWER'},
                                remapping   ={'action':'action',
                                              'current_task':'current_task',
                                              'task_num':'task_num'})
            # 找到人并向他靠近,目前处于未完成状态
            StateMachine.add('FIND',
                                self.xm_Find,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK','error':'error'},
                                remapping={'target':'target','current_task':'current_task'})
            # 导航到某个房间
            StateMachine.add('GO',
                                self.xm_Nav, 
                                transitions={'succeeded':'ANSWER','aborted':'aborted', 'error':'error'},
                                remapping={'target':'target','current_task':'current_task'})
            # 回答问题
            StateMachine.add('ANSWER', 
                                Anwser(),
                                transitions={'succeeded':'succeeded','aborted':'aborted'})

        # 用于将状态机可视化
        self.intro_server = IntrospectionServer('test',self.test,'/SM_ROOT')
        self.intro_server.start()
        # 运行状态机
        out = self.test.execute()
        # 状态机运行结束后结束可视化
        self.intro_server.stop()
        if out == 'succeeded' :
            self.smach_bool = True
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')
        

if __name__ == "__main__":
    try:
        WinterTest()
    except NameError:
        rospy.logerr('i am a slliby!')
