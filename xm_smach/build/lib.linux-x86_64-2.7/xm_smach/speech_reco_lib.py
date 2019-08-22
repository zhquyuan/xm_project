#!/usr/bin/env python
# encoding:utf8

# 用于语音、人物识别所用的类
# 用于对语音与人脸识别项目的辅助状态
# 创建于2018年3月2日 yzy
import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math

# 将识别数据转化成一句话
class GetSentences(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','error'],
                            input_keys = ['people_condition'],
                            io_keys = ['sentences'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'people_condition')
        except:
            rospy.logerr('there is no param transiformed')
            return 'error'
        else:
                    
            userdata.sentences = 'There are %d people , %d gentlemen and %d ladies.'%(userdata.people_condition['All'] , 
                                                                                        userdata.people_condition['Male'], 
                                                                                        userdata.people_condition['Female'])
            print userdata.sentences
            return 'succeeded'

#　收集识别数据记录在字典中
class CountPeople(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        io_keys=['people_condition'])

    def execute(self,userdata):
        
        ret = os.access('result.txt',os.R_OK)
        if ret == True:
            result = open("result.txt","r")
                
            userdata.people_condition['All'] = int(result.readline()[4:])
            userdata.people_condition['Female'] = int(result.readline()[7:])
            userdata.people_condition['Male'] = int(result.readline()[5:])
            userdata.people_condition['seatedM'] = int(result.readline()[8:])
            userdata.people_condition['seatedF'] = int(result.readline()[8:])
            userdata.people_condition['standM'] = int(result.readline()[7:])
            userdata.people_condition['standF'] = int(result.readline()[7:])
            userdata.people_condition['seated_elder'] = int(result.readline()[13:])
            userdata.people_condition['seated_adult'] = int(result.readline()[13:])
            userdata.people_condition['seated_younger'] = int(result.readline()[15:])
            userdata.people_condition['stand_elder'] = int(result.readline()[12:])
            userdata.people_condition['stand_adult'] = int(result.readline()[12:])
            userdata.people_condition['stand_younger'] = int(result.readline()[14:])

            print userdata.people_condition
            result.close()
            return 'succeeded'
        else:
            rospy.logerr('the file can not be opened')
            return 'aborted'

class RunNode_Num(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision people_num &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'
