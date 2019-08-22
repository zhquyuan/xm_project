#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from subprocess import *
from geometry_msgs.msg import *
import math
class WinterTest():
    def __init__(self):
        rospy.init_node('xm_winter_test') 
        rospy.on_shutdown(self.shutdown)
        self.xm_EnterRoom = StateMachine(outcomes=['succeeded','aborted','error'])
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
            self.xm_EnterRoom.userdata.go_point = Point(-0.1,0.0,0.0)
            # 与发令者对话坐标
            self.xm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            # 自我介绍的话
            self.xm_EnterRoom.userdata.sentences = 'I am sil meng, you can command me' 
            


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
        
        # 运行状态机
        out = self.xm_EnterRoom.execute()
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
    except:
        rospy.logerr('i am a sillby!')
