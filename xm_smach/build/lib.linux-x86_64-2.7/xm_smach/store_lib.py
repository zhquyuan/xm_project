import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool,Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os
from xm_smach.target_gpsr import *

class RunNode_obj(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision all_object &',shell =True)
        except:
            rospy.logerr('all_object node error')
            return 'aborted'
        return 'succeeded'

class GetObject_list(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                        output_keys = ['object_list'])
    def execute(self,userdata):
        try:
            test_open = os.access('out.txt',os.R_OK)
            if test_open == True:
                self.obj_file = open('out.txt','r')
                self.obj_list = list()
                for line in self.obj_file:
                    self.obj_list.append(str(line))

                userdata.object_list = self.obj_list
                return 'succeeded'

            else:
                rospy.logerr('cannot open the out.txt')
                return 'aborted'

        except Exception,e:
            rospy.logerr(e)
            return 'aborted'
                
class Get_object(State):
    def __init__(self):
        State.__init__(self, outcomes = ['succeeded','finish','error'],
                        input_keys=['object_list','current_task'],
                        output_keys = ['current_obj'])
    
    def execute(self , userdata):
        try:
            getattr(userdata,'object_list')
            getattr(userdata,'current_task')
        except:
            rospy.logerr('no params specified')
            return 'error'
        else:
            if userdata.object_list.length() > userdata.current_task:
                userdata.current_obj = userdata.object_list[userdata.current_task]
                userdata.current_task+=1
                return 'succeeded'
            else:
                return 'finish'

class Get_position(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','error'],
                        input_keys = ['position_list','current_obj'],
                        output_keys = ['arm_ps'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'position_list')
            self.name = userdata.current_obj
        except:
            rospy.logerr('no params specified')
            return 'error'
        else:
            mode = gpsr_target[self.name]['mode']
            userdata.arm_ps = userdata.position_list[mode]
            return 'succeeded'