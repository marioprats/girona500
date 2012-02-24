#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('navigation_g500')
import rospy
import tf
from tf.transformations import euler_from_quaternion
import tf_conversions.posemath as pm
import PyKDL
import math
import cola2_lib
from ekf_g500 import EKFG500

# Msgs imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from navigation_g500.msg import *
from sensor_msgs.msg import Imu
from auv_msgs.msg import NavSts


# Python imports
#import pylab
from numpy import *

class Localization :
    def __init__(self, name):
        """ Merge different navigation sensor values  """
        self.name = name
        
        # Create Odometry msg
        self.odom = Odometry()
        
        # self.listener = tf.TransformListener()
        self.getConfig()
        
        #Create static transformations
        self.dvl_tf = self.computeTf(self.dvl_tf)
        self.imu_tf = self.computeTf(self.imu_tf)        
        self.svs_tf = self.computeTf(self.svs_tf)
        
        #Initialize flags
        self.init = False
        self.imu_data = False
        self.gps_data = not self.gps_update
        self.init_north = 0.0
        self.init_east = 0.0
        self.last_prediction = rospy.Time.now()
        self.altitude = -1.0
        
        #Init Kalman filter
        self.ekf = EKFG500(self.model_covariance, 
                           self.gps_position_covariance, 
                           self.dvl_velocity_covariance)
        
        # Create publisher
        self.pub_odom = rospy.Publisher("/navigation_g500/odometry", Odometry)
        self.pub_nav_sts = rospy.Publisher("/navigation_g500/nav_sts", NavSts)
        
        # Create Subscriber
        rospy.Subscriber("/navigation_g500/teledyne_explorer_dvl", TeledyneExplorerDvl, self.updateTeledyneExplorerDvl)
        rospy.Subscriber("/navigation_g500/valeport_sound_velocity", ValeportSoundVelocity, self.updateValeportSoundVelocity)
        rospy.Subscriber("/navigation_g500/imu", Imu, self.updateImu)
        if self.gps_update :
            rospy.Subscriber("/navigation_g500/fastrax_it_500", FastraxIt500Gps, self.updateGps)

        
    def updateGps(self, gps):
        if not self.gps_data :
            self.gps_data = True
            self.init_north = gps.north
            self.init_east = gps.east
        
        self.odom.pose.pose.position.x = gps.north
        self.odom.pose.pose.position.y = gps.east
        
        # Update EKF
        if self.makePrediction():
            z = array([gps.north, gps.east])
            self.ekf.gpsUpdate(z)
            self.publishData()
        
        
    def updateTeledyneExplorerDvl(self, dvl):
        if dvl.bi_status == "A" and dvl.bi_error > -32.0:
            if abs(dvl.bi_x_axis) < self.dvl_max_v and abs(dvl.bi_y_axis) < self.dvl_max_v and abs(dvl.bi_z_axis) < self.dvl_max_v : 
                v = PyKDL.Vector(dvl.bi_x_axis, dvl.bi_y_axis, dvl.bi_z_axis)
            else : 
                v = PyKDL.Vector(.0, .0, .0)
        elif dvl.wi_status == "A" and dvl.wi_error > -32.0:
            if abs(dvl.wi_x_axis) < self.dvl_max_v and abs(dvl.wi_y_axis) < self.dvl_max_v and abs(dvl.wi_z_axis) < self.dvl_max_v : 
                v = PyKDL.Vector(dvl.wi_x_axis, dvl.wi_y_axis, dvl.wi_z_axis)
            else :
                v = PyKDL.Vector(.0, .0, .0)          
        else :
                v = PyKDL.Vector(.0, .0, .0)              
        
        #Rotate DVL velocities and Publish
        #Compte! EL DVL no es dextrogir i s'ha de negar la Y
        vr = self.dvl_tf.M * v
        self.odom.twist.twist.linear.x = vr[0]
        self.odom.twist.twist.linear.y = -vr[1]
        self.odom.twist.twist.linear.z = vr[2]
        self.altitude = dvl.bd_range
        
        # Update EKF
        if self.makePrediction():
            #Compte! EL DVL no es dextrogir i s'ha de negar la Y
            z = array([vr[0], -vr[1], vr[2]])
            self.ekf.dvlUpdate(z)
            self.publishData()
        
    
    def updateValeportSoundVelocity(self, svs):
        svs_data = PyKDL.Vector(.0, .0, svs.pressure)
        angle = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                                          self.odom.pose.pose.orientation.y,
                                                          self.odom.pose.pose.orientation.z,
                                                          self.odom.pose.pose.orientation.w])
        vehicle_rpy = PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
        svs_trans = self.svs_tf.p
        svs_trans = vehicle_rpy * svs_trans
        svs_data = svs_data + svs_trans
        self.odom.pose.pose.position.z = svs_data[2] 


    def updateImu(self, imu):
        if not self.imu_data :
            angle = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
            imu_data =  PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
            imu_data = self.imu_tf.M * imu_data
            angle = imu_data.GetRPY()   
            self.last_imu_orientation = angle
            self.last_imu_update = imu.header.stamp
            self.imu_data = True
        else:
            angle = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
            imu_data =  PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
            imu_data = self.imu_tf.M * imu_data
            angle = imu_data.GetRPY()   
            angle_quaternion = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2])
            self.odom.pose.pose.orientation.x = angle_quaternion[0]
            self.odom.pose.pose.orientation.y = angle_quaternion[1]
            self.odom.pose.pose.orientation.z = angle_quaternion[2]
            self.odom.pose.pose.orientation.w = angle_quaternion[3]
            
            # Derive angular velocities from orientations ####################### 
            period = (imu.header.stamp - self.last_imu_update).to_sec()
            self.odom.twist.twist.angular.x = cola2_lib.normalizeAngle(angle[0] - self.last_imu_orientation[0]) / period 
            self.odom.twist.twist.angular.y = cola2_lib.normalizeAngle(angle[1] - self.last_imu_orientation[1]) / period 
            self.odom.twist.twist.angular.z = cola2_lib.normalizeAngle(angle[2] - self.last_imu_orientation[2]) / period 
            self.last_imu_orientation = angle
            self.last_imu_update = imu.header.stamp          
            #####################################################################
            
            if self.makePrediction():
                self.ekf.updatePrediction()
                self.publishData()

        
    def makePrediction(self):
        if not self.init :
            if self.imu_data and self.gps_data:
                self.last_prediction = rospy.Time.now()
                self.ekf.initialize(array([self.init_north, self.init_east, 0.0, 0.0, 0.0]))
                self.init = True
            return False
        else :
            angle = euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                           self.odom.pose.pose.orientation.y,
                                           self.odom.pose.pose.orientation.z,
                                           self.odom.pose.pose.orientation.w])
            now = rospy.Time.now()
            t = (now - self.last_prediction).to_sec()
            self.last_prediction = now
            self.ekf.prediction(angle, t)
            return True
        
            
    def publishData(self):
        if self.init:
            x = self.ekf.getStateVector()
            angle = euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                           self.odom.pose.pose.orientation.y,
                                           self.odom.pose.pose.orientation.z,
                                           self.odom.pose.pose.orientation.w])            
            
        #   Create header    
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = self.name
            self.odom.child_frame_id = "girona500"
                       
            #Fil Nav status topic
            nav = NavSts()
            nav.header = self.odom.header
            nav.position.north = x[0]
            nav.position.east = x[1]
            nav.position.depth = self.odom.pose.pose.position.z
            nav.body_velocity.x = x[2]
            nav.body_velocity.y = x[3]
            nav.body_velocity.z = x[4]
            nav.orientation.roll = angle[0]
            nav.orientation.pitch = angle[1]
            nav.orientation.yaw = angle[2]
            nav.orientation_rate.roll = self.odom.twist.twist.angular.x
            nav.orientation_rate.pitch = self.odom.twist.twist.angular.y
            nav.orientation_rate.yaw = self.odom.twist.twist.angular.z
            nav.altitude = self.altitude
            
            #Publish topics
            self.pub_nav_sts.publish(nav)
            self.pub_odom.publish(self.odom)
            
            #Publish TF
            br = tf.TransformBroadcaster()
            br.sendTransform((0.0, 0.0, 0.0),                   
                             tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                             self.odom.header.stamp,
                             self.odom.header.frame_id,
                             self.odom.child_frame_id)
        else :
            self.last_time = rospy.Time.now()
            self.init = True
    
    
    def computeTf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
#        rospy.loginfo("Rotation: %s", str(r))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
#        rospy.loginfo("Vector: %s", str(v))
        frame = PyKDL.Frame(r, v)
#        rospy.loginfo("Frame: %s", str(frame))
        return frame
        
        
    def getConfig(self):
        if rospy.has_param("teledyne_explorer_dvl/tf") :
            self.dvl_tf = array(rospy.get_param("teledyne_explorer_dvl/tf"))
        else:
            rospy.logfatal("teledyne_explorer_dvl/tf param not found")

        if rospy.has_param("tritech_igc_gyro/tf") :
            self.imu_tf = array(rospy.get_param("tritech_igc_gyro/tf"))
        else:
            rospy.logfatal("tritech_igc_gyro/tf param not found")

        if rospy.has_param("valeport_sound_velocity/tf") :
            self.svs_tf = array(rospy.get_param("valeport_sound_velocity/tf"))
        else:
            rospy.logfatal("valeport_sound_velocity/tf param not found")
            
#       Sensors & model covariance
        if rospy.has_param("localization/dvl_covariance") :
            self.dvl_velocity_covariance = array(rospy.get_param('localization/dvl_covariance'))
        else:
            rospy.logfatal("localization/dvl_covariance param not found")
        
        if rospy.has_param("localization/gps_covariance") :
            self.gps_position_covariance = rospy.get_param('localization/gps_covariance')
        else:
            rospy.logfatal("localization/gps_covariance param not found")
            
        if rospy.has_param("localization/model_covariance") :
            self.model_covariance = rospy.get_param('localization/model_covariance')
        else:
            rospy.logfatal("localization/model_covariance param not found")
        
        if rospy.has_param("localization/dvl_max_v") :
            self.dvl_max_v = rospy.get_param('localization/dvl_max_v')
        else:
            rospy.logfatal("localization/dvl_max_v not found")
            
        if rospy.has_param("localization/gps_update"):
            self.gps_update = rospy.get_param('localization/gps_update')
        else:
            rospy.logfatal("localization/gps_update not found")



if __name__ == '__main__':
    try:
        #   Init node
	rospy.init_node('localization')
        localization = Localization(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
