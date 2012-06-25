#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy

# Msgs imports
from std_msgs.msg import Float64MultiArray
from control_g500.msg import *
from auv_msgs.msg import *

# Python imports
from numpy import *

# Custom imports
import cola2_lib

class ThrusterAllocator :
    
    def __init__(self, name):
        """ Controls the velocity and pose of an AUV  """
        self.name = name
        
    #   Load parameters
        self.getConfig()
        
    #   Create publisher
        self.pub = rospy.Publisher("/control_g500/thrusters_data", Float64MultiArray)
      
    #   Create Subscriber
    #    rospy.Subscriber("/control_g500/pose_to_force_req", BodyForceReq, self.updateBodyForceReq)
        rospy.Subscriber("/control_g500/velocity_to_force_req", BodyForceReq, self.updateBodyForceReq)
                  
                  
    def loadMatrix(self, name, rows, cols) :
        """ Load a matrix with id 'name' and size (name_rows x name_cols) from a YAML file """
        m = rospy.get_param( name )
        m = array(m).reshape(rows, cols)
        return matrix(m)
    
    
    def getConfig(self) :
        """ Load parameters from the rosparam server """
        self.vehicle_name = rospy.get_param("low_level_controller/vehicle_name")
        self.n_actuators = rospy.get_param("low_level_controller/n_actuators")
        self.apl = array( rospy.get_param("low_level_controller/actuators_polynomial_linearization") )
        self.actuator_control_matrix = self.loadMatrix("low_level_controller/actuator_control_matrix", 6, self.n_actuators)
        self.acm_inv = self.actuator_control_matrix.I
        
    
    def scaleAndSature(self, v, min_max, acm) :
        """ This function is a generalized way to merge surge and yaw. Right now is not used."""
        #If there is a DoF that is controlled by more than one thruster pointing to different directions
        #and at least one of the thrusters is saturated, all of them are scaled
        for dof in range(6) : #for each DoF
            th_count = 0 #number of thruster for this DoF
            direction_count = 0.0 #to check if the direction is always the same (valid directions: 1 or -1)
            saturate = False
            saturate_value = 0.0
            for th in range(len(v)) : #for each thruster
                if acm[dof, th] != 0.0 : #if the thruster controls this DoF
                    th_count += 1
                    direction_count += acm[dof, th]
                    if abs(v[th]) > min_max :
                        saturate = True
                        if abs(v[th]) > saturate_value : saturate_value = abs(v[th]) - min_max
            if (th_count > 1 and abs(direction_count)/th_count != 1 and saturate ) :
                #Scale thrusters value
                for th in range(len(v)) :
                    if acm[dof, th] != 0.0 :
                        #print "Scale th " + str(th) + " from " + str(v[th])
                        if v[th] > 0.0 : v[th] = v[th] - saturate_value
                        else : v[th] = v[th] + saturate_value
                        #print "to " + str(v[th])
        
        #Saturate
        i = nonzero(v > min_max)
        v[i] = min_max
        i = nonzero(v < -min_max)
        v[i] = -min_max        
        return v


    def mergeSurgeYaw(self, v):
        """ If the composition of Surge (v[0]) and Yaw (v[5]) overrides the max_fore parameter, 
            Yaw is respected and Surge reduced. """ 
        #TODO: max_force is set manually and represent the number of thrusters shared for surge and yaw
        # This value is related with param 'velocity_controller/force_max'
        max_force = 2
        if abs(v[0]) + abs(v[5]) > max_force:
            if v[0] > 0.0:
                v[0] = max_force - abs(v[5])
            else:
                v[0] = -max_force + abs(v[5])
        return v
    
    
    def computeThrusterSetpoint(self, f, poly) :
        #For the flies
        th_value = 0.0
        change_sign = False
        
        ret = zeros( len(f) )
        for th in range( len(f) ) :
            if f[th] < 0.0 :
                th_value = abs(f[th])
                change_sign = True
            else :
                th_value = f[th]
                change_sign = False
                
            for i in range( len(poly) ) :
                ret[th] = ret[th] + pow( th_value, i ) * poly[i]
                
            if change_sign: 
                ret[th] = ret[th] * -1.0
                
        return ret
    
    
    def updateBodyForceReq(self, w) :
        # Input data 
        body_force_req = zeros(6)
        
        if not w.disable_axis.x : 
            body_force_req[0] = w.wrench.force.x
        if not w.disable_axis.y : 
            body_force_req[1] = w.wrench.force.y
        if not w.disable_axis.z : 
            body_force_req[2] = w.wrench.force.z
        if not w.disable_axis.roll : 
            body_force_req[3] = w.wrench.torque.x
        if not w.disable_axis.pitch : 
            body_force_req[4] = w.wrench.torque.y
        if not w.disable_axis.yaw : 
            body_force_req[5] = w.wrench.torque.z
        
        rospy.loginfo("Body Force Request: %s", str(body_force_req))
        
        #Merge Surge and Yaw DoF giving priority to YAW
        body_force_req = self.mergeSurgeYaw(body_force_req)
        rospy.loginfo("Surge, Yaw merged: %s", str(body_force_req))
        
        # Computes the force to be done for each actuator
        f = self.acm_inv * matrix(body_force_req).T
        f = squeeze(asarray(f)) #matrix to array
        rospy.loginfo("force per thruster: %s", str(f))
        
        #Linearize compute thruster setpoints  
        setpoint = self.computeThrusterSetpoint(f, self.apl)
        rospy.loginfo("thrusters linearized: %s", str(setpoint))
        
        #Saturate setpoint to [-1, 1]
        setpoint = cola2_lib.saturateValue(setpoint, 1.0)
        rospy.loginfo("thrusters saturated: %s", str(setpoint))
        
        # Log and send computed data
        thrusters = Float64MultiArray()
        thrusters.data= setpoint
        
        #publish
        self.pub.publish(thrusters)
       

if __name__ == '__main__':
    try:
        rospy.init_node('thruster_allocator')
        thruster_allocator = ThrusterAllocator(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
