#!/usr/bin/env python
# encoding:utf8
# this module only contants the simple states usually used, the smach_ros we will directly use in the scripts
# userdata of smach-container includes the whole userdata interface ,you can achieve the different interface by try-except function
# no param specified error should be raise by state itself so it is easy for us to debug
import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
#import xm_speech.srv
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

# 导航的简单状态
# 在whoiswho中使用较多
# 要注意到一个很重要的细节:
# 如果导航到人, 位置信息来源于 cv ,所以类型是 PointStamped,
# 但是如果导航到地点, 路径点经常是特定的, 所以类型是 Pose
# 这里没有导航到人和导航到点的区分
class NavStack(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['pos_xm'])
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.tf_listener = tf.TransformListener()

        

    def execute(self, userdata):
        # 当mode ==1 时xm导航到人, 需要xm面对这人
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            # 等待60s到action服务可以使用
            self.nav_client.wait_for_server(rospy.Duration(60))
            return self.nav_thing(userdata.pos_xm)


    def nav_thing(self,pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pos_xm
        self.nav_client.send_goal(goal)
        nav_counter = 0
        # if nav stack is failed, we can plan again
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter<50:
            nav_counter +=1
            # 如果navstack状态被占用，我们将停止nav-task
            # 所以我们需要有一个多出来的preempted输出来完成这个状态
            # 向高层状态机跳出
            # 不要忘记刷新preempted标志来使容器的execute函数更安全
            # TODO:add the outcome 'preempted' and modify all the python-snippet
            # 下面代码：如果被占用取消目标并返回succeeded
            # if self.preempt_requested():
            #     self.nav_client .cancel_goal()
            #     return 'succeeded'
            # else:
            #     pass
            rospy.sleep(0.5)
        #如果到达目标地点返回succeeded,否则返回aborted
        if self.nav_client.get_goal_status_text() == 'Goal reached.' :
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'
    
#  xm_speak的简单状态,目前使用espeak,很难听-_-
class Speak(State):
    def __init__(self):
        State.__init__(self, 
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['sentences'])
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    
    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try :
            self.string_ = str(userdata.sentences)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else: 
            try:
                # subprocess - Subprocesses with accessible I/O streams
                # 创建一个子进程,父进程等待子进程完成,返回退出信息
                # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(self.string_)
                rospy.sleep(2.0)

                speech_bool = self.speak_client.call(self.string_)
                if speech_bool.flag == 1:
                    subprocess.call(["play","tts_sample.wav"])
                elif speech_bool.flag == 0:
                    subprocess.call("espeak -vf5 -s 100 '%(a)s'"%{'a':str(self.string_)} , shell = True)
                else:
                    rospy.logerr('the response error')
                    return 'error'
            except:
                return 'aborted'
            else:
                return 'succeeded'

# 用于获取 /move 服务来判断xm位置的简单状态
# it contains all the /move service call state

class SimpleMove_move(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        input_keys=['point'])

        self.move_client = rospy.ServiceProxy('/directMove/move',xm_Move)
        #等待服务25秒
        self.move_client.wait_for_service(timeout= 25.0)

    def execute(self,userdata):
        #注意在编译的时候,ros会自动把srv文件加Request变成头文件
        #获取MoveRequest类型
        move_value = xm_MoveRequest()
        try:
            #point由高层部状态机传输
            move_value.position = userdata.point
        except:
            rospy.logerr('No param specified')
            return 'error'
        print str(move_value)
        try:
            result =self.move_client.call(move_value)
        except:
            rospy.logerr('Goes wrong when call the /move')
        if result.arrived ==True :
            return 'succeeded'
        else:
            return 'aborted'
    
# armcmd 的3中不同的姿势（蛤）应该被写进一个统一的状态
# nav_ps 只是用于让机械臂知道这个动作是nav_pose而不是error
# 所以nav_ps 必须被定义为true
# nav_ps is just used for making the arm understand this action is nav_pose instead of error,
# so it must is specified as true
class ArmCmd(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=[ 'succeeded', 'aborted', 'error'],
                        io_keys=['mode','arm_ps','objmode'])
        self.xm_arm_client = rospy.ServiceProxy('arm_stack', xm_PickOrPlace)
        # 确保用于机械臂pick_or_place的PointStamped(抓取点的位置)在底盘base_footprint的坐标上
        # make sure that the PointStamped used for arm pick_or_place is in the frame base_footprint
        self.tf_listener = tf.TransformListener()
        
    def execute(self, userdata):
        rospy.loginfo('Moving arm')
        try:
            getattr(userdata,'mode')
            getattr(userdata,'arm_ps')
            getattr(userdata,'objmode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            try:
                req = xm_PickOrPlaceRequest()
                req.action = userdata.mode
                self.arm_ps_ =PointStamped()
                if userdata.arm_ps.header.frame_id is not '':
                    rospy.loginfo('frame checked')
                    self.arm_ps_ = self.tf_listener.transformPoint('base_link',userdata.arm_ps)
                else:
                    pass
                self.arm_ps_.point.x += 0.00
                self.arm_ps_.point.y += 0.00
                self.arm_ps_.point.z += -0.05
                req.goal_position = self.arm_ps_
                req.objmode = userdata.objmode
                rospy.logwarn(req)
                res =self.xm_arm_client.call(req)
                if res.result ==True:
                    return 'succeeded'
                else:
                    return 'aborted'
            except Exception,e:
                rospy.logerr(e)
                return 'aborted'

# 延续(苟)一段时间的简单状态,rec为延续(苟)的时间
class Wait(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error'],
        input_keys=['rec'])

    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:  
            rospy.sleep(userdata.rec)
            return "succeeded"

# monitor state 用于检测门是否关着以及xm是否想去屋子里^_^
class DoorDetect():
    def __init__(self):
        self.door_detect_ = MonitorState('DoorState', Bool, self.door_cb,max_checks =1)

    def door_cb(self,userdata,msg):
        if msg.data == True:
            #门开着刷新建图，刷新后返回
            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
            req = EmptyRequest()
            res = clear_client.call(req)
            return False
        else:
            return True

# 回答人们提出的问题
# the anwser is spell error ->_<-
# 单词拼错了,emmm,这是学长的锅,与我无关
# 3月２日更新代码，这里在调用服务的时候得到了返回值，其中1是不用回答2-7各对应一个问题
# 2---was the sitting person male or female
# 3---how many women are in the crowd
# 4---tell me if the sitting person was a woman 
# 5---tell me th number of elders in the crowd
# 6---was the sitting person boy or girl
# 7---tell me the number of males in the crowd
# 传入的peopel_condition参数是视觉识别的人群情况
class Anwser(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted'],
                            input_keys = ['people_condition'])
        self.anwser_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self,usedata):
        # try:
            
        #     subprocess.call('xterm -e rosrun xm_speech xm_speech_client_18_4.py &', shell=True)
        #     # res =self.anwser_client.call(command =1)
        # except:
        #     rospy.logerr('call the client error')
        #     return 'aborted'
        # else:
            
        self.anwser_client.wait_for_service(timeout =10)
        self.rec = self.anwser_client.call(command =1)
        self.string_ = ''
        self.peo_con = usedata.people_condition
        if self.rec.num >1:
            if self.rec.num == 2:
                if self.peo_con['seatedM']+self.peo_con['seatedF'] == 1 :
                    if self.peo_con['seatedM'] ==1:
                        self.string_ = 'the sitting person is a male'
                    else:
                        self.string_ = 'the sitting person is a female'
                else:
                    self.string_ = 'the sitting people is not one'
            elif self.rec.num == 3:
                if self.peo_con['Female'] <= 1:
                    self.string_ = 'There is %d woman'%(self.peo_con['Female'])
                else:
                    self.string_ = 'There are %d women'%(self.peo_con['Female'])
            elif self.rec.num ==4 :
                if self.peo_con['seatedF'] == 1 and self.peo_con['seatedM']+self.peo_con['seatedF'] ==1:
                    self.string_ = 'Yes ,the sitting person is woman'
                elif self.peo_con['seatedM']+self.peo_con['seatedF'] >1:
                    self.string_ = 'No,the sitting people are more than one'
                elif self.peo_con['seatedM']+self.peo_con['seatedF'] == 0 :
                    self.string_ = 'No,There is no sitting people'
                else:
                    self.string_ = 'The sitting people is a man'
            elif self.rec.num == 5:
                if self.peo_con['seated_elder']+self.peo_con['stand_elder'] >0:
                    self.string_ = 'There is %d elder '%(self.peo_con['seated_elder']+self.peo_con['stand_elder'])
                else:
                    self.string_ = 'There is no elder ,you are all yonger'
            elif self.rec.num == 6:
                if self.peo_con['seatedM']+self.peo_con['seatedF'] == 1 :
                    if self.peo_con['seatedM'] ==1:
                        self.string_ = 'the sitting person is a boy'
                    else:
                        self.string_ = 'the sitting person is a girl'
                else:
                    self.string_ = 'the sitting people is not one'
            elif self.rec.num == 7:
                if self.peo_con['Male'] <= 1:
                    self.string_ = 'There is %d man'%(self.peo_con['Male'])
                else:
                    self.string_ = 'There are %d men'%(self.peo_con['Male'])
            else:
                self.string_ = 'sorry,please ask again'
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                speech_bool = self.speak_client.call(self.string_)
                if speech_bool.flag == 1:
                    subprocess.call(["play","tts_sample.wav"])
                elif speech_bool.flag == 0:
                    subprocess.call("espeak -vf5 -s 100 '%(a)s'"%{'a':str(self.string_)} , shell = True)
                else:
                    rospy.logerr('the response error')
                    return 'aborted'
            except Exception,e:
                rospy.logerr(e)
            rospy.sleep(2.5)
            return 'succeeded'
        else:
            return 'succeeded'


# state use for object reco
# because in different task smach ,this state may make different tasks,so donnot 
# write this state class in the common_lib.py
class FindObject(State):
    def __init__(self):
        State.__init__(self,
                         outcomes=['succeeded','aborted','error'],
                         input_keys=['name'],
                         output_keys =['object_pos'],
                         io_keys=['objmode'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()
       
    def execute(self, userdata):
        goal = Point()
        try:
            name = userdata.name
        except:
            rospy.logerr('No param specified')
            return 'error'
        #i从1到5
        if name == 'tooth brush':
            userdata.objmode = 0
            rospy.logerr('hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh')
        else:
            userdata.objmode = 1
            rospy.logerr('ssssssssssssssssssssssssssssssssssss'+str(userdata.objmode))

        for i in range(5):
            try:

                self.xm_findobject.wait_for_service(timeout=30.0)
                req = xm_ObjectDetectRequest()
                req.object_name = name
                rospy.logwarn(name)
                rospy.logwarn(req)
                res = self.xm_findobject.call(req)
                if len(res.object)!=0:
                    break
            except Exception,e:
                rospy.logerr(e)
                return 'aborted'
        rospy.logwarn(res.object[0])
        if i == 2:
            rospy.logerr('result wrong')
            return 'aborted'
        if res.object[0].pos.point.x == -1.000 and res.object[0].pos.point.y == -1.000 and res.object[0].pos.point.z == -1.000:
            rospy.logerr('find nothing')
            return 'error'
        #   object_pos 是 PointStamped 类型
        object_pos = PointStamped()
        object_pos.point.x = res.object[0].pos.point.z -0.11
        object_pos.point.y = res.object[0].pos.point.x -0.125
        object_pos.point.z = 0.917-res.object[0].pos.point.y 
        object_pos.header.frame_id = 'base_link'

       #将物品的位置信息传输到userdata
        userdata.object_pos = object_pos
     
        # userdata.object_pos = res.object[0].pos
        # output_keys cannot be read     
        # print userdata.object_pos
        return 'succeeded'

# this state is used for justfy the position when need to execute arm-stack
# 这个状态在运行arm-stack是用于判断位置
# the PointStamped() object_pos have been justfied in the last state, so donnot do it again in this state
# PointStamped类型的object_pos已经在上一个状态(FindObject)中被判断过了，所以在这个状态不再判断
class PosJustfy(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys=['object_pos'],
                        output_keys =['pose'])

        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata,'object_pos')
        except:
            rospy.logerr('No params specified')
            return 'error'
        
        object_pos = self.tf_listener.transformPoint('base_link',userdata.object_pos)
        # data deal
        # object_pos传输从左到右第一物品的位置见FindObject
        pos_xm = object_pos
        #坐标位置x,y
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y

        angle = atan2(person_y, person_x)
        person_x = person_x - 0.7*cos(angle)
        person_y = person_y -0.7*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y = person_y
        pos_xm.point.z = 0
        # init the stamped of the Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        q_angle = quaternion_from_euler(0,0,angle)
        q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = q
        #全局坐标系和base_link间的关系变换发布
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
        # if error can directly raise the interupt
        # 如果发生错误将直接打断程序
        rospy.logwarn('wait for tf succeeded ')    

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    
        
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        userdata.pose = Pose(pos_xm.point,qs.quaternion)
        return 'succeeded'

# 控制xm旋转
# degree是旋转半角
class TurnDegree(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                        input_keys=['degree'])
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
    def execute(self,userdata):
        try:
            getattr(userdata, 'degree')
        except:
            rospy.logerr('No param specified!')
            return 'error'

        else:
            # if self.cmd_vel.get_num_connections() == 0:
            #     rospy.logerr('This state did nothing!')
            #     return 'succeeded'
            goal_angle = userdata.degree
            angular_speed = goal_angle/abs(goal_angle)
            rospy.logwarn(angular_speed)
            self.turn = Twist()
            self.turn.linear.x = 0.0
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = angular_speed

            
            angular_duration = goal_angle/angular_speed
            #发布频率
            rate = 50
            r = rospy.Rate(rate)
            try:
                #用1s进行旋转
                rospy.logwarn(angular_duration*rate)
                ticks = 87+abs(int(angular_duration*rate))*2
                for i in range(ticks):
                    self.cmd_vel.publish(self.turn)
                    rospy.logwarn(i)
                    r.sleep()
                rospy.sleep(1.0)
                return 'succeeded'
            except:
                rospy.logerr('meet question when publish Twist')
                return 'aborted'

class FindWay(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','error'],
                        input_keys = ['target','current_task'],
                        output_keys=['way_path1','way_path2'])
        self.way_req = rospy.ServiceProxy('house', xm_house)
    
    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'current_task')
        except:
            rospy.logerr('No param specified')
            return 'error'
        else:
            try:
                req = self.way_req.call(True)

                path1 = req.name +'_door_'+'out'
                path2 = userdata.target[userdata.current_task]+'_door_'+'in'

                userdata.way_path1 = gpsr_target[path1]['pos']
                userdata.way_path2 = gpsr_target[path2]['pos']

                rospy.logwarn(path1)
                rospy.logwarn(path2)
                return 'succeeded'
            except Exception,e:
                rospy.logerr(e)
                return 'aborted'
            


# class SimpleFollow():
#     def __init__(self):
#         self.find_people_ = MonitorState('follow',
#                                         xm_FollowPerson,
#                                         self.people_cb,
#                                         max_checks =5,
#                                         input_keys=['twist_xm'])
#         self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
# # 如果相机找到人,这个状态将会返回False
# # 相反,如果在五个循环里相机都没找到人,将会返回True
# # msg传入主题的数据
#     def people_cb(self,userdata,msg):
#         if msg is not None:
#             try:
#                 self.tmp_pos = msg.position
#                 rospy.logwarn(self.tmp_pos)
#             #如果得到人的坐标信息返回移动的位置
#                 self.twist_xm = self.data_deal(self.tmp_pos)
#             except:
#                 rospy.logerr('deal data wrong')
#                 return True
#             else:
#                 rospy.logwarn('follow deal succeed')
#                 rospy.loginfo('i will move')
#                 goal = self.twist_xm
#                 rospy.logwarn(goal) 
#                 publish_twist = Twist()
#                 publish_twist.linear.x = goal.linear.x
#                 publish_twist.angular.z = goal.angular.z
#                 #self.cmd_vel.publish(publish_twist)
#                 ticks = int(50)
#                 r = rospy.Rate(50)
#                 while True:
#                     rospy.logerr('publish######################################')
#                     self.cmd_vel.publish(publish_twist)
#                     r.sleep()
#                 return False
                
            
#         else:
#             raise Exception('MsgNotFind')

#     def data_deal(self,pos_xm):
#         # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
#         # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
#         person_x = pos_xm.point.z
#         person_y = pos_xm.point.x
        
#         #计算方位角
#         angle = atan2(person_y, person_x)
# #        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
# #        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
#         person_x = person_x - 0.6*cos(angle)
#         person_y = person_y - 0.6*sin(angle)
        
#         twist_xm = Twist()
#         twist_xm.angular.z = angle
#         twist_xm.linear.x = hypot(person_x,person_y)
#         return twist_xm


'''
class SimpleFollow():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['degree','distance'])

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
                self.data_deal(self.tmp_pos,userdata)
            except:
                return 'aborted'
            else:
                rospy.logwarn('follow deal succeed')




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

    def data_deal(self,pos_xm,userdata):
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
        
        distance = hypot(person_x,person_y)

        userdata.degree = angle
        userdata.distance = distance
'''            
