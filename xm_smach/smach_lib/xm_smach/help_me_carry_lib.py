#!usr/bin/env python
# encoding:utf8
import rospy
from smach import State,UserData,StateMachine
from xm_msgs.srv import *
from geometry_msgs.msg import *
from xm_msgs.msg import *
from math import *
import subprocess
from xm_smach.common_lib import *
import tf
from xm_smach.target_gpsr import *
from xm_arm_nav.xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_nav.xm_arm_moveit_level import *
# from xm_arm_nav.xm_arm_bag import *

# class GetTarget_Help(State):
#     def __init__(self):
#         State.__init__(self,outcomes=['succeeded','aborted'])
#     def execute(self,userdate):
        
        
class WaitForSpeak(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'succeeded'

class Close_OBJ(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            subprocess.call('xterm -e rosnode kill image_test &',shell=True)
            return 'succeeded'
        except:
            rospy.logerr('kill node wrong')
            return 'aborted'
class ChangeArmState(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                        input_keys=['arm_state'])
    def execute(self,userdata):
        try:
            self.arm = userdata.arm_state
            xm_arm_moveit_name(arm)
            return 'succeeded'
        except:
            rospy.logerr('move arm to ready to nav wrong')
            return 'aborted'
class RunOBJNode(State):
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted'])
    def execute(self, userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision image_test &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'
#这个状态是获取语音信号
class GetSignal(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['succeeded','aborted','error'],
                        io_keys = ['target','action'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)

    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')
        except Exception,e:
            rospy.logerr('No param transform')
            return 'error'
        try:
            self.target_client.wait_for_service(timeout = 10)
            self.response = self.target_client.call(command = 5)
            # if userdata.action[0] == 'stop':
            #     rospy.logerr('response wrong!')
            #     return 'aborted'
            userdata.target = self.response.target
            userdata.action = self.response.action
            rospy.logwarn(self.response.action)
            
            if len(self.response.target) != 0:
                return 'succeeded'
            elif self.response.action[0] == 'follow':
                return 'succeeded'
            else:
                return 'aborted'
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'

#
class SetTarget(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['succeeded','error'],
                        input_keys = ['target_name'],
                        io_keys = ['target'])
    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'target_name')
        except:
            rospy.logerr('No param transform')
            return 'error'
        else:
            userdata.target[0] = userdata.target_name
            return 'succeeded'

        
class CheckStop(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['stop','aborted'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)

    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                self.target_client.wait_for_service(timeout = 10)
                self.response = self.target_client.call(command = 5)
                self.action = self.response.action
                if self.response.action[0] == 'stop':
                    subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)

                    return 'stop'
                else:
                    continue
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'
        
class SimpleFollow(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','preempt'])
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
        self.twist_xm = Twist()
# 如果相机找到人,这个状态将会返回False
# 相反,如果在五个循环里相机都没找到人,将会返回True
# msg传入主题的数据
    def execute(self,userdata):
        if self.preempt_requested():
            return 'preempt'
        try:
            self.xm_position = rospy.Subscriber('follow',xm_FollowPerson,
                                                    callback = self.people_cb,queue_size=1)
            while not rospy.is_shutdown():
                if self.preempt_requested():
                    return 'preempt'
                else:
                    self.cmd_vel.publish(self.twist_xm)
            return 'succeeded'
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'

                    
    def people_cb(self, msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
            #如果得到人的坐标信息返回移动的位置
                self.twist_xm = self.data_deal(self.tmp_pos)
            except:
                rospy.logerr('deal data wrong')
            else:
                rospy.logwarn('follow deal succeed')
                rospy.loginfo('i will move')
                rospy.logwarn(self.twist_xm)
                
            
        else:
            raise Exception('MsgNotFind')

    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        distence = person_x
        angle = atan2(person_y,person_x)
        twist_xm = Twist()


        if  distence < 1.5 and (angle < -0.30 or angle > 0.30):
            twist_xm.angular.z = angle * 0.50
            twist_xm.linear.x = 0.000      
        if distence > 0.7 and distence < 1.5 and (angle < 0.2 or angle > -0.2):
            #计算方位角
    #         angle = atan2(person_y, person_x)
    # #        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
    # #        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
    #         person_x = person_x - 0.6*cos(angle)
    #         person_y = person_y - 0.6*sin(angle)
            twist_xm.angular.z = angle*0.60
            twist_xm.linear.x = distence * 0.2
        
        elif distence > 0.7 and distence < 1.5 and (angle >= 0.2 or angle <= -0.2):
            twist_xm.angular.z = 0.0
            twist_xm.linear.x = distence*0.2
        else:
            twist_xm.angular.z = 0.0
            twist_xm.linear.x = 0.0            
        return twist_xm

class SetPose(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'],
                        io_keys = ['pos_car'])
        self.tf_listener = tf.TransformListener()
    def execute(self , userdata):
        try:
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
            rospy.logwarn('wait for tf succeeded')

            (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
            # xm_Point = self.tf_listener.transformPoint('map','base_link')
            # xm_Quaternion = self.tf_listener.transformQuaternion('map','base_link')

            rospy.logwarn(trans)
            rospy.logwarn(rot)
            # gpsr_target['car']['pos'] = Pose(trans,rot)
            userdata.pos_car = Pose(trans,rot)
            return 'succeeded'
        
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'

class SetArmPose(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'],
                        io_keys = 'pos_car')
        self.tf_listener = tf.TransformListener()
    def execute(self , userdata):
        try:
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
            rospy.logwarn('wait for tf succeeded')

            (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
            # xm_Point = self.tf_listener.transformPoint('map','base_link')
            # xm_Quaternion = self.tf_listener.transformQuaternion('map','base_link')

            rospy.logwarn(trans)
            rospy.logwarn(rot)
            userdata.pos_car = Pose(trans,rot)
            return 'succeeded'
        
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'

class ArmCmd2(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self, userdata):
        
        try:
            joints2 =[0.0313406735659,-1.53585672379,2.09977054596,-1.88381099701,-1.57]
            lifting_controller_level(-0.0789094045758)
            arm_controller_level(joints2)
            joints3 = [-2.49428339885e-05,-1.44365561008,-0.0446576103568,2.18499946594,-1.57]
            lifting_controller_level(-0.0403053089976)
            arm_controller_level(joints3)
            return 'succeeded'
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'
        
