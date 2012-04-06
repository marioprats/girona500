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

# Imports for ploting
#import matplotlib
#matplotlib.use('GTKAgg') # do this before importing pylab
import matplotlib.pyplot as plt


# Custom imports
import cola2_lib


 
    
class RlAlgorithm(object) :
    def __init__(self,name):
        """ Controls the velocity and pose of an AUV  """
        self.name = name
        
        # Load parameters
        self.getConfig() 
        self.enable = self.is_enabled


    class Trials:
        def __init__(self,ntrials,nparams):
           self.policy = zeros((nparams,ntrials))
           self.reward = zeros(ntrials)


    def getConfig(self) :
        """ Load parameters from the rosparam server """
        if rospy.has_param(self.name +"/n_params") :
            self.n_params = rospy.get_param(self.name +"/n_params")
#array(rospy.get_param("tritech_igc_gyro/tf"))
        else:
            rospy.logfatal(self.name +"/nParmas param not found")

        if rospy.has_param(self.name +"/initial_params") :
            params = array(rospy.get_param(self.name +"/initial_params"))
            if ( self.n_params == len(params) ) :
                self.initial_params = params
            else:
                rospy.logfatal(self.name +"/initial_params mismatch length with n_parasm(%s)", self.n_params )
        else:
            rospy.logfatal(self.name +"/initial_params param not found")

        if rospy.has_param(self.name +"/initial_variance") :
            initial_variance = array(rospy.get_param(self.name +"/initial_variance")) 
            if ( self.n_params == len(initial_variance) ) :
                self.initial_variance = initial_variance
            else:
                rospy.logfatal(self.name +"/initial_variance mismatch length with n_parasm(%s)", self.n_params )
        else:
            rospy.logfatal(self.name +"/initial_variance param not found")
        
        if rospy.has_param(self.name +"/adaptative_variance") :
            self.adaptative_variance = rospy.get_param(self.name +"/adaptative_variance")
        else:
            rospy.logfatal(self.name +"/adaptative_variance param not found")

        if rospy.has_param(self.name +"/variance_decay") :
            self.variance_decay = rospy.get_param(self.name +"/variance_decay")
        else:
            rospy.logfatal(self.name +"/variance_decay param not found")

        if rospy.has_param(self.name +"/importance_sampling_top_count") :
            self.importance_sampling_top_count = rospy.get_param(self.name +"/importance_sampling_top_count")
        else:
            rospy.logfatal(self.name +"/importance_sampling_top_count")

        if rospy.has_param(self.name +"/n_trials") :
            self.n_trials = rospy.get_param(self.name +"/n_trials")
        else:
            rospy.logfatal(self.name +"/n_trials param not found")

        if rospy.has_param(self.name +"/n_steps") :
            self.n_steps = rospy.get_param(self.name +"/n_steps")
        else:
            rospy.logfatal(self.name + "/n_steps param not found")        


        if rospy.has_param(self.name +"/is_enabled") :
            self.is_enabled = rospy.get_param(self.name +"/is_enabled")
        else:
            rospy.logfatal(self.name + "/is_enabled param not found")    
            
#self.force_max = array( rospy.get_param("velocity_controller/force_max"))

            
    def rl_function(self,trials,trial) :
        """ Abstarc Method """
        pass


    def returnOfTrial(self,policy,n_steps) :
        """ Abstract Method """
        pass


        
    def rl_main(self): 
        #r = rospy.Rate(10)
                
        trials = self.Trials(self.n_trials,self.n_params)

        for trial in xrange(self.n_trials): 
            if rospy.is_shutdown() :
                break

            trials.policy[:,trial] = self.rl_function(trials,trial)
            trials.reward[trial] = self.returnOfTrial(trials.policy[:,trial], self.n_steps)   
        
        self.doPlots(trials)
    
    def doPlots(self,trials):
        plt.plot(range(len(trials.reward)), trials.reward )
        plt.xlabel('Trials')
        plt.ylabel('Reward')
        plt.show()

        
#if __name__ == '__main__':
#    try:
#        rospy.init_node('rl_Algorithm')

#        rl_Algorithm = RlAlgorithm(rospy.get_name())
           
#        rl_main()

#    except rospy.ROSInterruptException: pass
