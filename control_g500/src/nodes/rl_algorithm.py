#! /usr/bin/env python

#  This is a abstrac Class for the Reinforcement Leragning Algorithm
#  Created on: 6/04/2012
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

 
    
class RlAlgorithm :
    def __init__(self,name):
        """ Controls the velocity and pose of an AUV  """
        self.name = name
        
        # Load parameters
        self.getConfig()
        self.enable = self.is_enabled


    def getConfig(self) :
        """ Load parameters from the rosparam server """
        if rospy.has_param("rl_algorithm/n_params") :
            self.n_params = rospy.get_param("rl_algorithm/n_params")
#array(rospy.get_param("tritech_igc_gyro/tf"))
        else:
            rospy.logfatal("rl_algorithm/nParmas param not found")

        if rospy.has_param("rl_algorithm/initial_params") :
            params = array(rospy.get_param("rl_algorithm/initial_params"))
            if ( self.n_params == len(params) ) :
                self.initial_params = params
            else:
                rospy.logfatal("rl_algorithm/initial_params mismatch length with n_parasm(%s)", self.n_params )
        else:
            rospy.logfatal("rl_algorithm/initial_params param not found")

        if rospy.has_param("rl_algorithm/initial_variance") :
            initial_variance = array(rospy.get_param("rl_algorithm/initial_variance")) 
            if ( self.n_params == len(initial_variance) ) :
                self.initial_variance = initial_variance
            else:
                rospy.logfatal("rl_algorithm/initial_variance mismatch length with n_parasm(%s)", self.n_params )
        else:
            rospy.logfatal("rl_algorithm/initial_variance param not found")
        
        if rospy.has_param("rl_algorithm/adaptative_variance") :
            self.adaptative_variance = rospy.get_param("rl_algorithm/adaptative_variance")
        else:
            rospy.logfatal("rl_algorithm/adaptative_variance param not found")

        if rospy.has_param("rl_algorithm/variance_decay") :
            self.variance_decay = rospy.get_param("rl_algorithm/variance_decay")
        else:
            rospy.logfatal("rl_algorithm/variance_decay param not found")

        if rospy.has_param("rl_algorithm/importance_sampling_top_count") :
            self.importance_sampling_top_count = rospy.get_param("rl_algorithm/importance_sampling_top_count")
        else:
            rospy.logfatal("rl_algorithm/importance_sampling_top_count")

        if rospy.has_param("rl_algorithm/n_trials") :
            self.n_trials = rospy.get_param("rl_algorithm/n_trials")
        else:
            rospy.logfatal("rl_algorithm/n_trials param not found")

            
#self.force_max = array( rospy.get_param("velocity_controller/force_max"))



        
if __name__ == '__main__':
    try:
        rospy.init_node('rl_Algorithm')
        r = rospy.Rate(10)
        rl_Algorithm = RlPoWER(rospy.get_name())
        while not rospy.is_shutdown():
            velocity_controller.iterate()
            r.sleep()
    except rospy.ROSInterruptException: pass
