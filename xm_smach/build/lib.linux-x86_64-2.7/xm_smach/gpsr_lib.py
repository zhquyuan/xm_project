#!/usr/bin/env python
# encoding:utf8
from xm_smach.target_gpsr import gpsr_target
import rospy
import tf
import actionlib
from math import *
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool,Header
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import subprocess
import math        

# state decide the smach of gpsr ,it is used for choosing the meta-smach due to the order of speech
# 这个状态用于决定gpsr的状态机,它用于通过语命令选择一个元状态机
class NextDo(State):###跳转action
    def __init__(self):
        State.__init__(self, 
                    outcomes=['succeeded','aborted','go','find','talk','follow','pick','place','error'],
                    input_keys =['action','task_num'],
                    io_keys =['current_task'])
        
    def execute(self, userdata):
        try:
            action = userdata.action
            current_task = userdata.current_task
            task_num = userdata.task_num
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_task+=1

        # 测试运行成功
        # current_task目前测使的次数
        # task_num 测试总次数
        if userdata.current_task ==  task_num:
            return 'succeeded'
        current_action =  action[userdata.current_task]
        if current_action == 'go':
            return 'go'
        elif current_action == 'find':
            return 'find'
        elif current_action == 'follow':
            return 'follow'
        elif current_action == 'grasp':
            return 'pick'
        elif current_action == 'talk':
            return 'talk'
        elif current_action == 'place':
            return 'place'
        else:
            # no avaiable action find
            # userdata.current_task_out -1
            userdata.current_task -=1
            return 'aborted'
  
#   function as the name of state -_-
#   在 人\说话者\物品\位置 中进行切换
class PersonOrPosition(State):### switch the goal among person , speaker, thing, position
    def __init__(self):
        State.__init__(self, 
                        outcomes=['error','person','position'],
                        input_keys=['target','current_task'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'current_task')
        except:
            rospy.logerr('No param specified')
            return 'error'
        self.target = userdata.target[userdata.current_task]
        if self.target=='person' or self.target=='speaker':
            return 'person'
        else :
            return 'position'


# 用于找到人并进行跟随的状态
# 用于人和xm的位置
# xm必须面向人
# 这个状态要持续5个循环，因为相机可能一开始找不到人
'''@param max_checks the number of messages to receive and evaluate. If cond_cb returns False for any
               of them, the state will finish with outcome 'invalid'. If cond_cb returns True for 
               all of them, the outcome will be 'valid' '''
# here we want to use the features of MonitorState mentioned above
# 这里我们想用上述MonitorState
class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])

        self.tf_listener = tf.TransformListener()

# 如果相机找到人,这个状态将会返回False
# 相反,如果在五个循环里相机都没找到人,将会返回True
# msg传入主题的数据
    def people_cb(self,userdata,msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
            #如果得到人的坐标信息返回移动的位置
                if self.get_distance(self.tmp_pos) >= 0.5 and self.get_distance(self.tmp_pos)<=2.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except:
                rospy.logerr(e)
                return False
            
        else:
            raise Exception('MsgNotFind')
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        pos_xm.point.x = 0
        pos_xm.point.y = 0
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
#返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
#        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
#        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        person_x = person_x - 0.6*cos(angle)
        person_y = person_y - 0.6*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0

        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

 
# simple state used for get meaning from the speech node
# command mean invoke the speech node to return the meaning from the order
# 用于从对话中得到含义的简单状态
# 命令行是启动speech node来返回命令的含义
class GetTask(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        io_keys=['target','action','task_num'])
                        
        self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
                
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')            
            getattr(userdata, 'task_num')
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            self.speech_client.wait_for_service(timeout=10)
        #command = 2用于gpsr
            response =self.speech_client.call(command =2)
        except:
            rospy.logerr('wrong in call the service')
            return 'error'
        if response.action == 'stop':
            rospy.logerr('response wrong!')
            return 'aborted'
        if response.num > 0:
            userdata.task_num = response.num
            userdata.action = response.action
            userdata.target= response.target
            print(userdata.task_num)
            print(userdata.action)
            print(userdata.target)
            if userdata.action[1] == 'grasp':
                userdata.action.append('place')
                userdata.task_num += 1
            return 'succeeded'
        else:
            return 'aborted'


# 用于得到特定目标信息的状态
# 当 mode == 0 的时候返回目标的名字
# 当 mode == 1 是返回目标的位置
class GetTarget(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                            input_keys =['target','current_task','mode'],
                            output_keys =['current_target'])
    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified ')
            return 'error'
        if userdata.mode ==0:
            # due to the cv may need a different name-style from the speech_node
            # this should be modify
            # 由于cv可能需要一个来自speech_node的不同的name-style
            # 这个需要被修改
            userdata.current_target = userdata.target[userdata.current_task]
        elif userdata.mode ==1:
            userdata.current_target = gpsr_target[userdata.target[userdata.current_task]]['pos']
        else:
            return 'aborted'
        return 'succeeded'

# close the Kinect use a simple executable file
# 用一个可执行文件关闭关闭Kinect的状态
class CloseKinect(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)
        except:
            return 'aborted'
        return 'succeeded'

class CloseKinect_img(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            
            subprocess.call("touch /home/ye/Recognition/kinect2/close_image_test &" , shell = True)

            # subprocess.call("xterm -e rosnode kill /object_detection &" , shell = True)
        except:
            return 'aborted'
        return 'succeeded'

# the state is used for invoking the speech service to get the stop-signal
# 这个状态用于请求speech service来得到停止信号
class StopFollow(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
        self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    
    def execute(self,userdata):
        try:
            subprocess.call("xterm -e rosrun xm_speech xm_speech_client_demo.py &", shell = True)
        except:
            return 'aborted'
        
        self.speech_client.wait_for_service(timeout=10)
        res= self.speech_client.call(command=4)
        if res.num>0:
            return 'succeeded'

# 运行跟随人的节点
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision people_tracking &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'

class RunNode_img(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision image_test &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'


# get the detect Pose where can make the find-object task execute best
# 得到检测的位置能够使find-object测试运行得更好 
class GetPos(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['target','current_task','mode'],
                        output_keys =['pose'])

    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            last_task = userdata.current_task -1
            last_target = str(userdata.target[last_task]) +'_table' +'_' +str(userdata.mode)
            print last_target
            userdata.pose = gpsr_target[last_target]['pos']
            return 'succeeded'
            #这根本不会返回aborted吧
            if False:
                return 'aborted'
class StopByCount(State):
    def __init__(self):
        State.__init__(self,outcomes=['continue','stop','error'],
                       io_keys=['counter'],
                       input_keys=['count_num'])

    def execute(self, userdata):
        try:
            getattr(userdata,'counter')
        except:
            rospy.logerr('No param specified')
            return 'error'
        else:
            self_counter = userdata.counter
            self_count_num = userdata.count_num
            if self_counter >= self_count_num:
                userdata.counter = 0
                return 'stop'
            else:
                userdata.counter +=1
                return 'continue'

class CheckTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','continue','error'],
                        io_keys=['current_turn','task_num','current_task'],
                        input_keys=['turn'])
    def execute(self,userdata):
        try:
            self.current_turn = userdata.current_turn
            self.turn = userdata.turn
            rospy.logwarn(self.current_turn)
            rospy.logwarn(self.turn)
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        if self.current_turn > self.turn:
            rospy.logwarn('xm finish the turn')
            return 'succeeded'
        else:
            rospy.logwarn('finish one turn')
            userdata.current_turn += 1
            userdata.task_num = 0
            userdata.current_task = -1
            return 'continue'

