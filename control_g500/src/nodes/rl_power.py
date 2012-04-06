#! /usr/bin/env python

#  This node simulate the controller for the new arm
#  Created on: 28/03/2012
#  Author: arnau

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy

#import messages
from auv_msgs.msg import *
from nav_msgs.msg import Odometry

# Python imports
from numpy import *

# Custom imports
import cola2_lib

# How knows  ?????
from rl_algorithm import RlAlgorithm

 
    
class RlPoWER( RlAlgorithm ) :
    
#    def __init__(self,name):
#        super(name)


    
    def rl_main(self):
        roslib.loginfo("S'ha engeta ")
        roslib.loginfo("El nomebre de parametres es: %s", self.n_params)




if __name__ == '__main__':
    try:
        rospy.init_node('rl_PoWER')
        rl_PoWER = RlPoWER(rospy.get_name())
        
        #self.rl_main()
       
    except rospy.ROSInterruptException: pass
