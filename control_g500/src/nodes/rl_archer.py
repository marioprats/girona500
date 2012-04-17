#! /usr/bin/env python

#  This node implements the algorithm of ARCHER.
#  It implements the abstract class RLAlgorithm. 
#  Created on: 14/04/2012
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

from rl_algorithm import RlAlgorithm

 
    
class RlARCHER( RlAlgorithm ) :
    
#    def __init__(self,name):
#        super(name)


    
#    def rl_main(self):
#        rospy.loginfo("S'ha engeta ")
#        rospy.loginfo("El nomebre de parametres es: %s", self.n_params)

    class Trials:
        def __init__(self,ntrials,nparams):
           self.policy = zeros((nparams,ntrials))
           self.reward = zeros(ntrials)
           self.rewardARCHER = zeros(ntrials)


    def rl_main(self): 
        #r = rospy.Rate(10)
                
        trials = self.Trials(self.n_trials,self.n_params)

        for trial in xrange(self.n_trials): 
            if rospy.is_shutdown() :
                break
            trials.policy[:,trial] = self.rl_function(trials,trial)
            trials.reward[trial] = self.returnOfTrial(trials.policy[:,trial], self.n_steps) 
            trials.rewardARCHER[trial] = self.returnOfTrialARCHER(trials.policy[:,trial], self.n_steps) 

        self.doPlots(trials)

    def rl_function(self,trials,trial):

        """ARCHER Implementation"""
        if trial == 0 : # do initialization
            new_policy = self.initial_params
            #print " els parametres inicials son " + str(self.initial_params)
        else if trial-1 < self.importance_sampling_top_count :
            #not enoug trials to use ARCHER yet
            #calulate the decayed variance
            variance = self.initial_variance * self.variance_decay**(trial-1)
            new_policy = self.initial_params + ((variance**0.5)*random.rand(self.n_params))
        else :
            #build the lookup table for the importance sampling
            s_Return = hstack( ( array([trials.reward[0:trial]]).T ,  array([range(trial)]).T  )  )
            index = argsort(s_Return[:,0])
            s_Return = s_Return[index,:] # all previous trials sortide by reward ASC


            #pick the top 3 best trials, select their indicies
            idxA = s_Return(s_Return.shape[0],1)
            idxB = s_Return(s_Return.shape[0]-1,1)
            idxC = s_Return(s_Return.shape[0]-2,1) # is the trial with the smallest reward from the three

            #make sure that we always use the last trial params as one of A,B,C
            #if the last trial is not among the top 3 best, then replace C with the last trial
            if trial-1 != idxA and trial-1 != idxB and trial-1 != idxC :
                idxC = trial-1 # replace C with the last trials

            #educated parametres
           
        return new_policy


    # Educated guess for next RL params - based on assumption of local linearity or param-effect space
    def educatedGuessABCtoO ( Aparam, Bparam, Cparam, Aresult, Bresult, Cresult, Oresult ) :
        failed = False
        if (Aresult == Oresult) :
            #I assume Aresult is the closest to Oresult among the 3 A, B and C, so I only check it
            Oparam = Aparam
            return [ Oparam, failed ]
        if (Aparam == Bparam) or (Aparam == Cparam) or (Bparam == Cparam) :
            rospy.logwarn('Cannot do educated guess cause some parameters are equal!')
            Oparam = Aparam # just return one of the params, cause cannot do anything,
            # I assume Aresult is the closest to Oresult, so I return Aparam to be used
            return [ Oparam, failed ]

        #check if the A, B and C lie on a line (or almost lie on a line)
        smallest_angle = smallestAngleOfTriangle(Aresult, Bresult, Cresult) 
  
  % check if the point almost lie on a line, if so return Aparam
  if smallest_angle < 1
    warning('The points lie on a line!');
    Oparam = Aparam; % TODO: + noise
    failed = 1;
    return;
  end

        return [Oparam failed]


  def smallestAngleOfTriangle (a, b, c) :

      #rad2deg(angleBetweenVectors([3 0], [0 5]))
      #rad2deg(angleBetweenVectors([6 0], [sqrt(2)/2 sqrt(2)/2]))
  
      ab = b - a;
      ac = c - a;
      bc = c - b;
  
      angleA = rad2deg(angleBetweenVectors(ab, ac))
      angleB = rad2deg(angleBetweenVectors(-ab, bc))
      angleC = rad2deg(angleBetweenVectors(-ac, -bc))
  
      #sum = angleA+angleB+angleC;
      #sum
      #smallest_angle = min([angleA, angleB, angleC, 180-angleA, 180-angleB, 180-angleC]);
      smallest_angle = min(angleA, angleB, angleC)
      return smallest_angle


  def angleBetweenVectors(a, b) :
      #HERE I'M HERE !!!!
      # AQUI
      #El dot es el producta es calar de dos vectors ull!!!
      cosAngle = dot(a,b)/(norm(a)*norm(b));
      angle = acos(cosAngle);



    def returnOfTrial(self,policy,n_steps) :
        return ( 10000 - (policy[0]**2 + policy[1]**2) )
    
    
    



if __name__ == '__main__':
    try:
        rospy.init_node('rl_ARCHER')
        rl_ARCHER = RlPoWER(rospy.get_name())
        
        rl_ARCHER.rl_main()
       
    except rospy.ROSInterruptException: pass
