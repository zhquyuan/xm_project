#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.help_me_carry_lib import *
from xm_smach.store_lib import *
from geometry_msgs.msg import *
from geometry_msgs.msg._Pose import Pose
from xm_smach.target_gpsr import gpsr_target
import math
import subprocess

class Store():

    def __init__(self):
        rospy.init_node('Store')
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('------------start test----------------')
        self.test_bool = False
        """[State]
           [Name]kinect_rec图像识别(识别所有物体)
        """

        self.kinect_rec = StateMachine(outcomes=['succeeded','aborted','error'],
                                            output_keys=['object_list'])                             
        with self.kinect_rec:
            self.kinect_rec.userdata.rec = 20.0
            StateMachine.add('RUNNODE',
                                RunNode_obj(),
                                transitions = {'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'GETLIST','error':'error'})
            StateMachine.add('GETLIST',
                                GetObject_list(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted'},
                                remapping = {'object_list':'object_list'})

        """[State]
           [Name]sm_Pick
           [function]识别某个物体调整位姿并抓取
        """
        self.sm_Pick = StateMachine(outcomes=['succeeded','aborted','error'],
                                            input_keys = ['current_obj'])
        with self.sm_Pick:
            self.sm_Pick.userdata.target_mode =0
            self.sm_Pick.userdata.objmode = -1
            self.sm_Pick.userdata.rec = 10.0
            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'FIND_OBJECT','error':'error'})
            
            self.sm_Pick.userdata.object_pos = PointStamped()
            self.sm_Pick.userdata.objmode = -1
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'FIND_OBJECT','error':'SPEAK'},
                                remapping ={'name':'current_obj','object_pos':'object_pos','objmode':'objmode'})
            
            #making the xm foreward the object may make the grasping task easier  
            self.sm_Pick.userdata.pose = Pose()
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
                                transitions ={'succeeded':'PICK','aborted':'FIND_AGAIN','error':'SPEAK'},
                                remapping ={'name':'current_obj','object_pos':'object_pos','objmode':'objmode'})
            self.sm_Pick.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_Pick.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
        
        self.xm_Place = StateMachine(outcomes=['succeeded','aborted','error'],
                                        output_keys = ['position_list'],
                                        input_keys = ['current_obj','position_list'])
        with self.xm_Place:
            self.xm_Place.userdata.mode = 2
            self.xm_Place.userdata.objmode = 2
            StateMachine.add('GET_POSITION',
                                Get_position(),
                                transitions = {'succeeded':'PLACE','error':'error'},
                                remapping = {'arm_ps':'arm_ps'})
            
            StateMachine.add('PLACE',
                                ArmCmd(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
        
        """[State]
           [Name]grasp_from_list
           [function]控制状态机，从列表里依次提取是sm_Pick和sm_Place的上层状态机
        """
        self.grasp_from_list = StateMachine(outcomes=['succeeded','aborted','error'],
                                                input_keys = ['object_list'])
        with self.grasp_from_list:
            self.grasp_from_list.userdata.current_task = 0
            self.grasp_from_list.userdata.pos_xm_place = gpsr_target['place']['pos']
            self.grasp_from_list.userdata.pos_xm_pick = gpsr_target['pick']['pos']
            self.grasp_from_list.userdata.position_list = {1:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.670 )),
                                                           2:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.355 )),
                                                           3:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.040 )),
                                                           4:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.000 ))}


            StateMachine.add('GETOBJ',
                                Get_object(),
                                transitions = {'succeeded':'GRASP','finish':'succeeded','error':'error'},
                                remapping = {'current_obj':'current_obj'})
            StateMachine.add('GRASP',
                                self.sm_Pick,
                                transitions={'succeeded':'MOVE_PLACE','aborted':'aborted','error':'error'})
        
            StateMachine.add('MOVE_PLACE',
                                NavStack(),
                                transitions={'succeeded':'PLACE','aborted':'MOVE_PLACE','error':'error'},
                                remapping = {'pos_xm':'pos_xm_place'})
            StateMachine.add('PLACE',
                                self.xm_Place,
                                transitions={'succeeded':'MOVE_PICK','aborted':'MOVE_PICK','error':'error'})
            StateMachine.add('MOVE_PICK',
                                NavStack(),
                                transitions={'succeeded':'GETOBJ','aborted':'MOVE_PICK','error':'error'},
                                remapping = {'pos_xm':'pos_xm_pick'})
       
        self.store = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.store:
            self.store.userdata.rec = 3.0
            StateMachine.add('RECO',
                                self.kinect_rec,
                                transitions={'succeeded':'GRASP_FROM_LIST','aborted':'aborted','error':'error'},
                                remapping={'object_list':'object_list'} )
            # StateMachine.add('WAIT',
            #                     Wait(),
            #                     transitions = {'succeeded':'GRASP_FROM_LIST','error':'error'})
            StateMachine.add('GRASP_FROM_LIST',
                                self.grasp_from_list,
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

        try:
            out = self.store.execute()
        except Exception,e:
            rospy.logerr('test failed , loser loser, don\'t have dinner')
            rospy.logerr(e)
        if out == 'succeeded':
            rospy.logwarn('All the test done well')
            self.smach_bool = False
        else:
            rospy.logerr('Sorry master, xm is failed.')
    
            
    def shutdown(self):
        if self.test_bool == True:
            rospy.loginfo('test succeeded')
        else:
            rospy.logerr('test failed')
      
if __name__ == "__main__":
    try:
        Store()
    except Exception,e:
        rospy.logerr(e)