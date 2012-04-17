#! /usr/bin/env python

#  This node implement the algorithm of PoWER.
#  It implements the abstract class RlAlgorithm
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


    
#    def rl_main(self):
#        rospy.loginfo("S'ha engeta ")
#        rospy.loginfo("El nomebre de parametres es: %s", self.n_params)


    def rl_function(self,trials,trial):
        """PoWER Implementation"""
        if trial == 0 : # do initialization
            new_policy = self.initial_params
            #print " els parametres inicials son " + str(self.initial_params)
            
        else :
            #build the lookup table for the importance sampling
            s_Return = hstack( ( array([trials.reward[0:trial]]).T ,  array([range(trial)]).T  )  )
            index = argsort(s_Return[:,0])
            s_Return = s_Return[index,:] # all previous trials sortide by reward ASC

            #update the policy parameters
            param_nom = zeros(self.n_params)
            param_dnom = 0
        
            #calculate the expectations (the normalization is taken care of by the division)
            #as importance sampling we take the 10 best rollouts

            for i in xrange( min(self.importance_sampling_top_count, trial) ) :
                j = s_Return[s_Return.shape[0]-i-1, 1]# get the trial number for the best sigma trials 
                #calculate the exploration with respect to the current (i.e. last trial) parameters
                temp_explore = trials.policy[:,j] - trials.policy[:,trial-1]
                #always have the same exploration variance,
                #and assume that always only one basis functions is active we get these simple sums
                param_nom = param_nom + temp_explore*trials.reward[j]
                param_dnom = param_dnom + trials.reward[j]

            #update the parameters
            new_policy = trials.policy[:,trial-1] + param_nom/(param_dnom+1e-10)
            #add noise using the decayed variance
            variance = self.initial_variance * self.variance_decay**(trial-1); 
            
            #start decaying the noise from 3nd trial
            new_policy = new_policy + ((variance**0.5)*random.rand(self.n_params)) 
           
        return new_policy


    def returnOfTrial(self,policy,n_steps) :
        return ( 10000 - (policy[0]**2 + policy[1]**2) )
    
    
    



if __name__ == '__main__':
    try:
        rospy.init_node('rl_PoWER')
        rl_PoWER = RlPoWER(rospy.get_name())
        
        rl_PoWER.rl_main()
       
    except rospy.ROSInterruptException: pass
