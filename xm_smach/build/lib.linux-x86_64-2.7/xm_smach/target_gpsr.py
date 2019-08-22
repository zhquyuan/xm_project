#!/usr/bin/env python 
#encoding:utf8 

"""

	it is a stuff list for GPSR 
	it contains the pose of all the stuff
        
"""
from geometry_msgs.msg import *
from geometry_msgs.msg._Pose import Pose


# mode = 1 在桌子上
# mode = 2 在架子上
gpsr_target={
'speaker': {'pos': Pose(Point(6.089 , 2.031, 0.000), Quaternion(0.000, 0.000, -0.688, 0.726)), 'mode': 1 },
'end': {'pos': Pose(Point(7.829, 3.997, 0.000),Quaternion(0.000, 0.000, 0.000, 1.000)), 'mode': 1 },
'Gray': {'pos': Pose(), 'mode': 1 },
'David': {'pos': Pose(), 'mode': 1 },
'Daniel': {'pos': Pose(), 'mode': 1 },
'Jack': {'pos': Pose(), 'mode': 1 },
'Jenny': {'pos': Pose(), 'mode': 1 },
'Michael': {'pos': Pose(), 'mode': 1 },
'Lucy': {'pos': Pose(), 'mode': 1 },
'Peter': {'pos': Pose(), 'mode': 1 },
'Tom': {'pos': Pose(), 'mode': 1 },
'Jordan': {'pos': Pose(), 'mode': 1 },

'kitchen_table_1': {'pos': Pose(Point(2.390, -6.656, 0.000),Quaternion(0.000, 0.000, 0.999, -0.036)), 'mode': 1 },
'kitchen_table_2': {'pos': Pose(Point(2.390, -6.656, 0.000),Quaternion(0.000, 0.000, 0.999, -0.036)), 'mode': 1 },
####################                   winter_test                                 ################################
'kitchen': {'pos': Pose(Point(-0.728, 2.851, 0.000),Quaternion(0.000, 0.000, 1.000, -0.018)), 'mode': 1 },
'bedroom': {'pos': Pose(Point(5.960 , -2.122 , 0.000), Quaternion(0.000, 0.000, 0.003, 1.000)), 'mode': 1 },########gai
'livingroom': {'pos': Pose(Point(4.443 , 1.068, 0.000),Quaternion(0.000, 0.000, 1.000, 0.032)), 'mode': 1 },##########gai
'diningroom':{'pos':Pose(Point(0.001, 0.000 , 0.000), Quaternion(0.000, 0.000, 0.005, 1.000)), 'mode': 1 },
###################################################################################################################
'diningroom_table_1': {'pos': Pose(Point(5.600, -7.019, 0.000),Quaternion(0.000, 0.000, -0.670, 0.742)), 'mode': 1 },
'diningroom_table_2': {'pos': Pose(Point(4.231, -2.501, 0.000),Quaternion(0.000, 0.000, 0.997, -0.080)), 'mode': 1 },

'bedroom_table_1': {'pos': Pose(Point(7.435, -1.896, 0.000),Quaternion(0.000, 0.000, 0.019, 1.000)), 'mode': 1 },
'bedroom_table_2': {'pos': Pose(Point(7.435, -1.896, 0.000),Quaternion(0.000, 0.000, 0.019, 1.000)), 'mode': 1 },

'init_pose':{'pos': Pose(Point(9.122, 1.874, 0.000),Quaternion(0.000, 0.000, -0.048, 0.999)), 'mode': 1 },


'livingroom_table_1': {'pos': Pose(Point(6.159, -6.388, 0.000),Quaternion(0.000, 0.000, -0.045, 0.999)), 'mode': 1 },
'livingroom_table_2': {'pos': Pose(Point(6.386, -7.761, 0.000),Quaternion(0.000, 0.000, 0.000, 1.000)), 'mode': 1 },

'cooking_table': {'pos': Pose(Point(3.058, 5.605, 0.000),Quaternion(0.000, 0.000, 0.707, 0.708)), 'mode': 1 },
'TV_table': {'pos': Pose(Point(4.138, 2.179, 0.000),Quaternion(0.000, 0.000, -0.639, 0.770)), 'mode': 1 },
'book-cabinet': {'pos': Pose(Point(10.200, 1.910, 0.000),Quaternion(0.000, 0.000, 0.724, 0.690)), 'mode': 1 },
'sprite': {'pos': Pose(), 'mode': 1 },
'red-bull': {'pos': Pose(), 'mode': 1 },
'tea': {'pos': Pose(), 'mode': 1 },
'juice': {'pos': Pose(), 'mode': 1 },
'coffee': {'pos': Pose(), 'mode': 1 },
'biscuit': {'pos': Pose(), 'mode': 1 },
'chips': {'pos': Pose(), 'mode': 1 },
'roll-paper': {'pos': Pose(), 'mode': 1 },
'toothpaste': {'pos': Pose(), 'mode': 1 },

#########################      help_me_carry          #############
'door':{'pos' : Pose(Point(6.4602, 2.969 , 0.000), Quaternion(0.000,0.000,0.683,0.730)) , 'mode' : 1},
'out_door':{'pos':Pose(Point(6.728, 4.698,0.000), Quaternion(0.000, 0.000, -0.720, 0.694)),'mode': 1},
'car':{'pos':Pose(Point(7.967, 9.313, 0.000),Quaternion(0.000, 0.000, 0.713, 0.701)),'mode':1}
}
