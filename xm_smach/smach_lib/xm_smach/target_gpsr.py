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
'speaker': {'pos': Pose(Point(2.411 , 0.221, 0.000), Quaternion(0.000, 0.000, 0.016, 1.000)), 'mode': 1 },
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
'kitchen': {'pos': Pose(Point(3.506, 5.711, 0.000),Quaternion(0.000, 0.000, 1.000, 0.004)), 'mode': 1 },
'bedroom': {'pos': Pose(Point(9.803 , 2.299 , 0.000), Quaternion(0.000, 0.000, 0.724, 0.690)), 'mode': 1 },########gai
'diningroom': {'pos': Pose(Point(6.460 , 1.796, 0.000),Quaternion(0.000, 0.000, 1.000, 0.014)), 'mode': 1 },##########gai
'livingroom':{'pos':Pose(Point(7.323, 5.440 , 0.000), Quaternion(0.000, 0.000, -0.717, 0.697)), 'mode': 1 },
'kitchen_door_in':{'pos':Pose(Point(1.736,2.014,0.000),Quaternion(0.000,0.000,0.989,0.145)),'mode':1},
'kitchen_door_out':{'pos':Pose(Point(1.945,2.107,0.000),Quaternion(0.000,0.000,-0.084,0.996)),'mode':1},
'livingroom_door_in':{'pos':Pose(Point(2.908,1.733,0.000),Quaternion(0.000,0.000,0.027,1.000)),'mode':1},
'livingroom_door_out':{'pos':Pose(Point(2.786,1.672,0.000),Quaternion(0.000,0.000,1.000,0.015)),'mode':1},
'bedroom_door_in':{'pos':Pose(Point(3.095,-1.780,0.000),Quaternion(0.000,0.000,0.003,1.000)),'mode':1},
'bedroom_door_out':{'pos':Pose(Point(2.964,-1.772,0.000),Quaternion(0.000,0.000,0.999,-0.048)),'mode':1},
'diningroom_door_in':{'pos':Pose(Point(2.040,-1.013,0.000),Quaternion(0.000,0.000,0.998,-0.065)),'mode':1},
'diningroom_door_out':{'pos':Pose(Point(2.197,-0.955,0.000),Quaternion(0.000,0.000,0.219,0.976)),'mode':1},
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


'soap':{'pos': Pose(), 'mode': 1 },
'red bull':{'pos': Pose(), 'mode': 2 },
'coconut':{'pos': Pose(), 'mode': 2 },
'green bean':{'pos': Pose(), 'mode': 2 },
'cola':{'pos': Pose(), 'mode': 2 },
'tooth brush':{'pos': Pose(), 'mode': 1 },
'milk tea':{'pos': Pose(), 'mode': 2 },
'paper':{'pos': Pose(), 'mode': 1 },

#########################      help_me_carry          #############
'door':{'pos' : Pose(Point(1.462, 0.122 , 0.000), Quaternion(0.000,0.000,0.998,0.062)) , 'mode' : 1},
'out_door':{'pos':Pose(Point(0.000 , 0.000 ,0.000), Quaternion(0.000, 0.000, 0.000 , 1.000)),'mode': 1},
'car':{'pos':Pose(Point(0.000 , 0.000 , 0.000),Quaternion(0.000, 0.000, 0.000 , 1.000)),'mode':1},
########################   store ############################

'place':{'pos':Pose(Point(-0.396, 0.45102,0.000),Quaternion(0.000,0.000,0.631,0.776)),'mode':1},
'pick':{'pos':Pose(Point(0.000, 0.000, 0.000),Quaternion(0.000 ,0.000,0.000,1.000))},

'exit':{'pos' : Pose(Point(11.017, 7.470 , 0.000), Quaternion(0.000,0.000,0.723,0.690)) , 'mode' : 1}

}
