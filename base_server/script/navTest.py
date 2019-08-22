#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal, MoveBaseActionFeedback
from target_gpsr import *
#from geometry_msgs.msg import *
#from geometry_msgs.msg._Pose import Pose


class NavStack(State):
    def __init__(self):
        State.__init__(self,
                    outcomes=['succeeded','aborted','error','None'],
                    input_keys=['xm_goal'])
        self.nav_client = actionlib.SimpleActionClient("move_base",MoveBaseAction)

    def execute(self,userdata):
        try:
            getattr(userdata,'xm_goal')
        except:
            rospy.logerr('please give me a goal')
        else:
            self.nav_client.wait_for_server(rospy.Duration(30))
            return self.navBegin(userdata.xm_goal)

    def navBegin(self,xm_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = xm_goal
        self.nav_client.send_goal(goal)
        nav_counter = 0
        while self.nav_client.get_state()!=GoalStatus.SUCCEEDED and nav_counter<50:
            nav_counter+=1
            rospy.sleep(0.5)

class NavInsist():
    def __init__(self):
        rospy.init_node('xm_nav_test')
        rospy.on_shutdown(self.shutdown)
        self.xm_nav = StateMachine(outcomes=['succeeded','aborted','error','None'])
        with self.xm_nav:
            #gpsr_target={
            #    'livingroom': {'pos': Pose(Point(5.809, -5.834, 0.000),Quaternion(0.000, 0.000, -0.743, 0.670)), 'mode': 1 },
            #    'diningroom_table_1': {'pos': Pose(Point(4.069, -1.749, 0.000),Quaternion(0.000, 0.000, 1.000, 0.005)), 'mode': 1 }
            #}
            self.xm_nav.userdata.xm_goal=gpsr_target['livingroom']['pos']
            StateMachine.add('NAV_1',
                            NavStack(),
                            transitions={'succeeded':'NAV_2','aborted':'aborted','error':'error','None':'NAV_1'})
            self.xm_nav.userdata.xm_goal=gpsr_target['livingroom']['pos']
            StateMachine.add('NAV_2',
                            NavStack(),
                            transitions={'succeeded':'succeeded','aborted':'aborted','error':'error','None':'NAV_2'})
        run = self.xm_nav.execute()
        if run == 'succeeded' :
            self.smach_bool = True

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":    
    try:
        NavInsist()
    except:
        var=1
        while var==1:
            try:
                NavInsist()
            except:
                rospy.logerr('haha')