#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy

# Msgs imports
from auv_msgs.msg import *
from nav_msgs.msg import Odometry

# Python imports
#import pylab
from numpy import *

# Custom imports
import cola2_lib

class VelocityController :
    def __init__(self, name):
        """ Controls the velocity and pose of an AUV  """
        self.name = name
        
        # Load parameters
        self.getConfig()
    
        # Input data 
        self.v = zeros(6)
        self.desired_velocity = zeros(6)
        self.resp = BodyVelocityReq()
        self.ek_velocity_1 = zeros(6)
        self.eik_velocity_1 = zeros(6)
        self.past_time = rospy.Time.now()
        
        # Create publisher
        self.pub_tau = rospy.Publisher("/control_g500/velocity_to_force_req", BodyForceReq)
    
        # Create Subscriber
#        rospy.Subscriber("/navigation_g500/odometry", Odometry, self.updateOdometry)
        rospy.Subscriber("/navigation_g500/nav_sts", NavSts, self.updateNavSts)
        rospy.Subscriber("/control_g500/merged_body_velocity_req", BodyVelocityReq, self.updateResponse)
        

    def getConfig(self) :
        """ Load parameters from the rosparam server """
        #TODO: Check that the parameters are in the param server
        self.force_max = array( rospy.get_param("velocity_controller/force_max"))
        self.pid_velocity_feed_forward_force = array( rospy.get_param("velocity_controller/pid_velocity_feed_forward_force") )
        kp = array( rospy.get_param("velocity_controller/pid_velocity_kp") )
        ki = array( rospy.get_param("velocity_controller/pid_velocity_ki") )
        kd = array( rospy.get_param("velocity_controller/pid_velocity_kd") )
        sat = array( rospy.get_param("velocity_controller/pid_velocity_sat") )
        self.pid_velocity = cola2_lib.PidConfig(kp, ki, kd, sat)
        
    
    def updateOdometry(self, odom):
        self.v[0] = odom.twist.twist.linear.x
        self.v[1] = odom.twist.twist.linear.y
        self.v[2] = odom.twist.twist.linear.z
        self.v[3] = odom.twist.twist.angular.x
        self.v[4] = odom.twist.twist.angular.y
        self.v[5] = odom.twist.twist.angular.z
    ##    print "v: \n" + str(_v)
    
    
    def updateNavSts(self, nav_sts):
        self.v[0] = nav_sts.body_velocity.x
        self.v[1] = nav_sts.body_velocity.y
        self.v[2] = nav_sts.body_velocity.z
        self.v[3] = nav_sts.orientation_rate.roll
        self.v[4] = nav_sts.orientation_rate.pitch
        self.v[5] = nav_sts.orientation_rate.yaw
     ##   print "v: " + str(around(_v,2))
    
    
    def updateResponse(self, resp):
        self.desired_velocity[0] = resp.twist.linear.x
        self.desired_velocity[1] = resp.twist.linear.y
        self.desired_velocity[2] = resp.twist.linear.z
        self.desired_velocity[3] = resp.twist.angular.x
        self.desired_velocity[4] = resp.twist.angular.y
        self.desired_velocity[5] = resp.twist.angular.z
        self.resp = resp
        #print "Received response: \n" + str(_sp)
         
         
    def iterate(self): 
    #   Main loop
        rospy.loginfo("desired_velocity: %s", str(self.desired_velocity))
        rospy.loginfo("current velocity: %s", str(self.v))
        
        # Apply PIDs to obtain tau
        # Compute real period
        now = rospy.Time.now()
        real_period_ = (now - self.past_time) #nano seconds to seconds
        self.past_time = now
        rospy.loginfo("real_period: %s", str(real_period_.to_sec()))
        
        # Compute TAU velocity
        tau_velocity = zeros(6)
        [tau_velocity, self.ek_velocity_1, self.eik_velocity_1]  = cola2_lib.computePid6Dof(self.desired_velocity, self.v, self.pid_velocity.kp, 
                                                self.pid_velocity.ki, self.pid_velocity.kd, self.pid_velocity.sat,
                                                self.ek_velocity_1, self.eik_velocity_1, real_period_.to_sec())
        
        # Compute TAU and publish it for the navigator
        tau = (tau_velocity +  self.pid_velocity_feed_forward_force) * self.force_max
        rospy.loginfo("Tau: %s", str(tau))
        
        data = BodyForceReq()
        data.header.stamp = now
        data.header.frame_id = "vehicle_frame"
        data.goal.requester = "/control_g500/velocity_controller"
        data.goal.id = 0
        data.goal.priority = 10
        data.wrench.force.x = tau[0]
        data.wrench.force.y = tau[1]
        data.wrench.force.z = tau[2]        
        data.wrench.torque.x = tau[3]
        data.wrench.torque.y = tau[4]
        data.wrench.torque.z = tau[5]
        
        data.disable_axis.x = self.resp.disable_axis.x
        data.disable_axis.y = self.resp.disable_axis.y
        data.disable_axis.z = self.resp.disable_axis.z
        data.disable_axis.roll = self.resp.disable_axis.roll
        data.disable_axis.pitch = self.resp.disable_axis.pitch
        data.disable_axis.yaw = self.resp.disable_axis.yaw
        
        self.pub_tau.publish(data)


if __name__ == '__main__':
    try:
        rospy.init_node('velocity_controller')
        r = rospy.Rate(10)
        velocity_controller = VelocityController(rospy.get_name())
        while not rospy.is_shutdown():
            velocity_controller.iterate()
            r.sleep()
    except rospy.ROSInterruptException: pass
