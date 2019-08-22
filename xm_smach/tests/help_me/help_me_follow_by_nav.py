#!/usr/bin/env python
# encoding:utf8
# 识别指示->记忆操作员->发出启动信号->跟随操作员  ->记忆当前坐标->拿起手提袋->导航到指定地点->找到另一个人->请求帮助->发出准备引导的信号->导航记忆坐标->到达后发出信号
#                              ->接受停止指令                                                                          ->检测关门
# 推开物体时必须进行说明

# 1.找人的人的位置是否在机器人面前
# 2.开门方向一般会向外开(从外面进向里开，从里面出向外开)

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

class HelpMeCarry():
    def __init__(self):
        rospy.init_node('help_me_carry')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        self.trace = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'succeeded',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'}},
#                                 outcome_cb = self.trace_out_cb,
                                 child_termination_cb = self.trace_child_cb)
        with self.trace:
            self.meta_follow = StateMachine(['succeeded','aborted'])
            with self.meta_follow:
                StateMachine.add('FIND',
                                    FindPeople().find_people_,
                                    transitions = {'invalid':'NAV','valid':'FIND','preempted':'succeeded'},
                                    remapping={'pos_xm':'pos_xm'})
                StateMachine.add('NAV',
                                    NavStack(),
                                    transitions = {'succeeded':'FIND','aborted':'FIND','error':'aborted'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            Concurrence.add('RUNNODE',RunNode())
        
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
            # StateMachine.add('FINDWAY',
            #                     FindWay(),
            #                     transitions = {'succeeded':'NAV_PATH1','aborted':'NAV_GO','error':'NAV_GO'},
            #                     remapping={'way_path1':'way_path1','way_path2':'way_path2'})
            # StateMachine.add('NAV_PATH1',
            #                     NavStack(),
            #                     transitions={'succeeded':'NAV_PATH2','aborted':'NAV_PATH1','error':'error'},
            #                     remapping={'pos_xm':'way_path1'})
            # StateMachine.add('NAV_PATH2',
            #                     NavStack(),
            #                     transitions = {'succeeded':'NAV_GO','aborted':'NAV_PATH2','error':'error'},
            #                     remapping={'pos_xm':'way_path2'})
            #如果找不到路径继续执行NavStack的execute(),知道找到为止
            StateMachine.add('NAV_GO', 
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'NAV_GO','error':'error'},
                                remapping={'pos_xm':'pos_xm'})
        
        
        self.xm_Find = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys=['target','current_task'])
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
                                transitions ={'invalid':'NAV_PEOPLE','valid':'CLOSEKINECT','preempted':'aborted'},
                                remapping = {'pos_xm':'pos_xm'})
            # this state will use the userdata remapping in the last state  
            StateMachine.add('NAV_PEOPLE',
                                NavStack(),
                                transitions = {'succeeded':'CLOSEKINECT','aborted':'NAV_PEOPLE','error':'error'},
                                remapping = {'pos_xm':'pos_xm'})
            

            # close the KinectV2
            StateMachine.add('CLOSEKINECT',
                                CloseKinect(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
        self.pick_up = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.pick_up:
            self.pick_up.userdata.name = 'bag'
            self.pick_up.userdata.objmode = -1
            self.pick_up.userdata.arm_state = 'after_grasp_bag'
            self.pick_up.userdata.mode = 3
            self.pick_up.userdata.arm_ps = PointStamped()
            self.pick_up.userdata.arm_ps.header.frame_id ='base_link'
            self.pick_up.userdata.arm_ps.point.x =0.8
            self.pick_up.userdata.arm_ps.point.y =0.0
            self.pick_up.userdata.arm_ps.point.z =0.6 
            StateMachine.add('RUNNODE',
                                RunOBJNode(),
                                transitions= {'succeeded':'GET_POSITION','aborted':'aborted'})
            StateMachine.add('GET_POSITION',
                                FindObject(),
                                transitions = {'succeeded':'POS_JUSTFY','aborted':'GET_POSITION','error':'PICK_OBJ2'})
            self.pick_up.userdata.pose = Pose()
            StateMachine.add('POS_JUSTFY',
                                PosJustfy(),
                                remapping={'object_pos':'object_pos','pose':'pose'},
                                transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_TO',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG2','aborted':'NAV_TO','error':'error'},
                                remapping ={"pos_xm":'pose'})
            StateMachine.add('RUNNODE_IMG2',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_AGAIN','aborted':'aborted'})                    
            StateMachine.add('FIND_AGAIN',
                                FindObject(),
                                transitions ={'succeeded':'PICK_OBJ','aborted':'FIND_AGAIN','error':'PICK_OBJ2'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            StateMachine.add('PICK_OBJ',
                                ArmCmd(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                ##这里的aborted比赛时可以改成到READY_NAV
                                remapping = {'arm_ps':'object_pos'})
            StateMachine.add('PICK_OBJ2',
                                ArmCmd(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
            # self.pick_up.userdata.sentences = 'xiao meng can not find the thing'
            # StateMachine.add('SPEAK',
            #                     Speak(),
            #                     transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
            # StateMachine.add('READY_NAV',
            #                     ChangeArmState(),
            #                     transitions= {'succeeded':'CLOSEKINECT_OBJ','aborted':'aborted'})
            # StateMachine.add('CLOSEKINECT_OBJ',
            #                     Close_OBJ(),
            #                     transitions={'succeeded':'succeeded','aborted':'aborted'})
        
        self.sm_Place = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Place:
            # # place_ps please specified due to the scene
            # self.sm_Place.userdata.place_ps = PointStamped()
            # self.sm_Place.userdata.place_ps.header.frame_id ='base_link'
            # self.sm_Place.userdata.place_ps.point.x =0.8
            # self.sm_Place.userdata.place_ps.point.y =0.0
            # self.sm_Place.userdata.place_ps.point.z =0.6 
            # self.sm_Place.userdata.objmode = 2
            # # without moveit, if is just place it in a open space
            # self.sm_Place.userdata.arm_mode_1 =2         
            StateMachine.add('PLACE',
                                ArmCmd2(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
        
        self.help_me = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.help_me:
            #这里可以预先定义一个值，识别错误后仍可以继续进行
            self.help_me.userdata.target = list()
            self.help_me.userdata.action = list()
            self.help_me.userdata.current_task = 0
            self.help_me.userdata.start_sentences = 'i will follow you'
            self.help_me.userdata.help_sentences = 'pleas help carry the groceries into the house, thank you'
            self.help_me.userdata.nav_sentences = 'i will go to the car, please follow me, my master'
            self.help_me.userdata.target_name = 'door'
            self.help_me.userdata.rec = 3.0
            self.help_me.userdata.go_point = Point(-1.5,0.0,0.0)
            self.help_me.userdata.arrive_sentences = 'here is the car,my master'
            self.help_me.userdata.pos_car = gpsr_target['car']['pos']
            self.help_me.userdata.pos_door = gpsr_target['door']['pos']
            self.help_me.userdata.pos_out_door = gpsr_target['out_door']['pos']
            self.help_me.userdata.arm_mode_pick = 3
            self.help_me.userdata.arm_state_put = 'xm_place_bag' 
            self.help_me.userdata.arm_ps = PointStamped(Header(0,0,''),Point(0,0,0)) 
            self.help_me.userdata.open_door_sentences = 'please help me open the door'
            
            StateMachine.add('GET_START',    #开始喽，获取语音信号，如果你说‘follow me’，那action就是follow
                                GetSignal(),
                                transitions = {'succeeded':'SPEAK_START','aborted':'GET_START','error':'error'})

            StateMachine.add('SPEAK_START',
                                Speak(),
                                transitions = {'succeeded':'FOLLOW','aborted':'FOLLOW','error':'error'},
                                remapping = {'sentences':'start_sentences'})
            StateMachine.add('FOLLOW',
                                self.trace,
                                transitions = {'succeeded':'SET_POSITION','aborted':'FOLLOW'})
            # StateMachine.add('CLOSEKINECT',
            #                     CloseKinect(),
            #                     transitions = {'succeeded':'PICK','aborted':'PICK'})

            #####not finished#####
            StateMachine.add('SET_POSITION',
                                SetPose(),
                                transitions={'succeeded':'PICK','aborted':'PICK'},
                                remapping = {'pos_car':'pos_car'})
            StateMachine.add('PICK',
                                self.pick_up,
                                transitions = {'succeeded':'GET_TARGET','aborted':'GET_TARGET','error':'error'},
                                remapping = {'arm_mode':'arm_mode_pick'})
            StateMachine.add('GET_TARGET',
                                GetSignal(),
                                transitions = {'succeeded':'NAV_ROOM','aborted':'GET_TARGET','error':'error'})
            StateMachine.add('NAV_OUT_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'SIMPLE_MOVE1','aborted':'NAV_OUT_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_out_door'})

            # 在开启导航之前首先让xm冲进房间
            # 使用userdata: go_point
            StateMachine.add('SIMPLE_MOVE1',
                                SimpleMove_move(),
                                remapping ={'point':'go_point'},
                                transitions={'succeeded':'NAV_ROOM','aborted':'NAV_ROOM','error':'error'})
            
            StateMachine.add('NAV_ROOM',
                                self.xm_Nav,
                                transitions = {'succeeded':'PUT','aborted':'aborted','error':'error'})

            StateMachine.add('PUT',
                                self.sm_Place,
                                transitions = {'succeeded':'FIND','aborted':'FIND'},
                                remapping={'arm_state':'arm_state_put'})
            StateMachine.add('FIND',
                                self.xm_Find,
                                transitions = {'succeeded':'ASK_HELP','aborted':'aborted','error':'error'})
            StateMachine.add('ASK_HELP',
                                Speak(),
                                transitions = {'succeeded':'SPEAK_NAV','aborted':'SPEAK_NAV','error':'error'},
                                remapping = {'sentences':'help_sentences'})
            
            StateMachine.add('SPEAK_NAV',
                                Speak(),
                                transitions = {'succeeded':'SET_TARGET','aborted':'NAV_DOOR','error':'error'},
                                remapping = {'sentences':'nav_sentences'})
            StateMachine.add('SET_TARGET',
                                SetTarget(),
                                transitions = {'succeeded':'NAV_DOOR','error':'error'})
            StateMachine.add('NAV_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'DOORDETECT','aborted':'NAV_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_door'})
            #遇到问题是Doordetect是否要和导航同时运行
            #将导航分为两部分，一个是导航到门，一个是导航到车
            StateMachine.add('DOORDETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'SPEAK_OPEN_DOOR','preempted':'aborted'})
            StateMachine.add('SPEAK_OPEN_DOOR',
                                Speak(),
                                transitions={'succeeded':'DOORDETECT2','aborted':'aborted','error':'error'},
                                remapping={'sentences':'open_door_sentences'})
            StateMachine.add('DOORDETECT2',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOORDETECT2','preempted':'aborted'})

            # 在刷新建图后等待一段时间 
            # 使用userdata: rec
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})

            # 在开启导航之前首先让xm冲进房间
            # 使用userdata: go_point
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove_move(),
                                remapping ={'point':'go_point'},
                                transitions={'succeeded':'NAV_CAR','aborted':'NAV_CAR','error':'error'})
            #####not finished#####
            StateMachine.add('NAV_CAR',
                                self.xm_Nav,
                                transitions={'succeeded':'SPEAK_ARRIVE','aborted':'NAV_CAR','error':'error'},
                                remapping={'pos_xm':'pos_car'})
            StateMachine.add('SPEAK_ARRIVE',
                                Speak(),
                                remapping = {'sentences':'arrive_sentences'},
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
        # self.navigation = Concurrence(outcomes=['succeeded','aborted'],
        #                               default_outcome = ['aborted'],
        #                               input_keys = ['car_position'],
        #                               outcome_cb = self.navigation_child_cb,
        #                               child_termination_cb = self.navigation_out_cb)
        # with self.navigation:
        #     Concurrence.add('NAV')
        #     Concurrence.add('DOORDETECT')
        # self.help_me = StateMachine(outcomes=['succeeded','aborted','error'])

        try:
            out = self.help_me.execute()
        except Exception,e:
            rospy.logerr('test failed , loser loser, don\'t have dinner')
            rospy.logerr(e)
        if out == 'succeeded':
            rospy.logwarn('All the test done well')
            self.smach_bool = False
        else:
            rospy.logerr('Sorry master, xm is failed.')
    
    def trace_child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            subprocess.call('xterm -e touch /home/ye/Recognition/kinect2/close_image_test &', shell = True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['FOLLOW']:
            rospy.logerr('the follow state meet error!')
            return True
        return False 
    
    # def trace_out_cb(self, outcome_map):
    #     if outcome_map['STOP'] == 'succeeded':
    #         rospy.logwarn('stop trace!')
    #         subprocess.call('rosnode kill people_tracking')
    #         return 'succeeded'
    #     else:
    #         return 'aborted'

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn('xm is the best!')
        else:
            rospy.logwarn('sorry, master')


if __name__ == '__main__':
    try:
        HelpMeCarry()
    except Exception,e:
        rospy.logerr('xm will do better next time')
        rospy.logerr(e)

