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
from std_srvs.srv import Empty
#from safety_g500.msg import NavSensorsStatus

# Python imports
from numpy import *

class Localization :
    def __init__(self, name):
        """ Merge different navigation sensor values  """
        self.name = name
        
        # Create Odometry msg
        self.odom = Odometry()
        self.nav = NavSts()
        
        # self.listener = tf.TransformListener()
        self.getConfig()
        
        #Create static transformations
        self.dvl_tf = self.computeTf(self.dvl_tf_data)
        self.imu_tf = self.computeTf(self.imu_tf_data)        
        self.svs_tf = self.computeTf(self.svs_tf_data)
        
        #Initialize flags
        self.init = False
        self.imu_data = False
        self.gps_data = not self.gps_update
        self.init_north = 0.0
        self.init_east = 0.0
        self.last_prediction = rospy.Time.now()
        self.altitude = -1.0
        
        #init last sensor update
        self.init_time = rospy.Time.now()
        self.dvl_last_update = self.init_time
        self.imu_last_update = self.init_time
        self.svs_last_update = self.init_time
        self.dvl_init = False
        self.imu_init = False
        self.svs_init = False
        self.gps_init_samples_list = []
        
        # In the simulator is not used the safety module
        # rospy.Timer(rospy.Duration(self.check_sensors_period), self.checkSensors)

        #Init Kalman filter
        self.ekf = EKFG500(self.model_covariance, 
                           self.gps_position_covariance, 
                           self.dvl_velocity_covariance)
        
        # Create publisher
        self.pub_odom = rospy.Publisher("/navigation_g500/odometry", Odometry)
        self.pub_nav_sts = rospy.Publisher("/navigation_g500/nav_sts", NavSts)
#Safety module is not used in the simulator        
#self.pub_nav_sensors_status = rospy.Publisher("/safety_g500/nav_sensors_status", NavSensorsStatus)
        
        # Create Subscriber
        rospy.Subscriber("/navigation_g500/teledyne_explorer_dvl", TeledyneExplorerDvl, self.updateTeledyneExplorerDvl)
        rospy.Subscriber("/navigation_g500/valeport_sound_velocity", ValeportSoundVelocity, self.updateValeportSoundVelocity)
        rospy.Subscriber("/navigation_g500/imu", Imu, self.updateImu)
        if self.gps_update :
            rospy.Subscriber("/navigation_g500/fastrax_it_500_gps", FastraxIt500Gps, self.updateGps)

        #Create services
        self.reset_navigation = rospy.Service('/navigation_g500/reset_navigation', Empty, self.resetNavigation)
   

    def resetNavigation(self, req):
        rospy.loginfo("%s: Reset Navigation", self.name)    
        x = self.ekf.getStateVector()
        x[0] = 0.0
        x[1] = 0.0
        self.ekf.initialize(x)
        
        
#    def checkSensors(self, event):
#        rospy.loginfo("Check nav sensors")
#        if self.dvl_init and self.imu_init and self.svs_init and self.init:
#            now = rospy.Time.now()
#            nav_sensors_status = NavSensorsStatus()
#            nav_sensors_status.header.stamp = now
            
#            if (now - self.dvl_last_update).to_sec() > self.dvl_max_period_error:
    #            rospy.logerr("%s: DVL data error", self.name)
#                nav_sensors_status.dvl_status = False
#            else :
#                nav_sensors_status.dvl_status = True
                
#            if (now - self.imu_last_update).to_sec() > self.imu_max_period_error:
    #            rospy.logerr("%s: IMU data error", self.name)
#                nav_sensors_status.imu_status = False
#            else :
#                nav_sensors_status.imu_status = True
                
#            if (now - self.svs_last_update).to_sec() > self.svs_max_period_error:
    #            rospy.logerr("%s: SVS data error", self.name)
#                nav_sensors_status.svs_status = False
#            else :
#                nav_sensors_status.svs_status = True
                
#            self.pub_nav_sensors_status.publish(nav_sensors_status)
#        else :
#            if (rospy.Time.now() - self.init_time).to_sec() > self.max_init_time:
#                rospy.logfatal("%s: Not all nav sensors have been initialized!", self.name)
#                rospy.logfatal("%s: DVL init: %s", self.name, self.dvl_init)
#                rospy.logfatal("%s: SVS init: %s", self.name, self.svs_init)
#                rospy.logfatal("%s: IMU init: %s", self.name, self.imu_init)
#                rospy.logfatal("%s: EKF filter init: %s", self.name, self.init)
                
                
          
    def updateGps(self, gps):
        if gps.data_quality >= 1:
            if not self.gps_data :
                self.gps_init_samples_list.append([gps.north, gps.east])
                if len(self.gps_init_samples_list) >= self.gps_init_samples:
                    self.gps_data = True
                    [self.init_north, self.init_east] = median(array(self.gps_init_samples_list), axis=0)
                    rospy.loginfo('%s, GPS init data: %sN, %sE', self.name, self.init_north, self.init_east)
            else:
                distance = sqrt((self.nav.position.north - gps.north)**2 + 
                                (self.nav.position.east - gps.east)**2)
                #rospy.loginfo("%s, Distance: %s", self.name, distance)
                if distance < 5.0:
                    # If the error between the filter and the GPS is small, update the kalman filter
                    self.odom.pose.pose.position.x = gps.north
                    self.odom.pose.pose.position.y = gps.east
                    
                    # Update EKF
                    if self.makePrediction():
                        z = array([gps.north, gps.east])
                        self.ekf.gpsUpdate(z)
                        self.publishData()
                        
                elif distance < 50.0:
                    #If the error is big, reinitialize the navigation filter if GPS is very far
                    #self.init = False
                    #self.gps_data = False
                    #rospy.loginfo("%s, Filter need to be reinitialized", self.name)
                    distance = 0
                else:
                    #If the error is huge, we assume that the GPS data is not valid
                    #rospy.loginfo("%s, Invalid GPS data received:\n", self.name)
                    distance = 0
        
    def updateTeledyneExplorerDvl(self, dvl):
        self.dvl_last_update = rospy.Time.now()
        self.dvl_init = True
        
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
        vr2 = array([vr[0], -vr[1], vr[2]])
        
        #Ara ja tenim la velocitat lineal en el DVL representada en eixos de vehicle
        #falta calcular la velocitat lineal al centre del vehicle en eixos del vehicle
        angular_velocity = array([self.odom.twist.twist.angular.x,
                                  self.odom.twist.twist.angular.y,
                                  self.odom.twist.twist.angular.z])
        distance = self.dvl_tf_data[0:3]
        vr2 = vr2 - cross(angular_velocity, distance)
        
#        print "linear velocity (dvl): " + str(vr2)
#        print "angular velocity (g500): " + str(angular_velocity)
#        print "distance: (dvl->g500)" + str(distance)
#        print "linear velocity (g500): " + str(vr2)
        
        #Copy data to odometry message
        self.odom.twist.twist.linear.x = vr2[0]
        self.odom.twist.twist.linear.y = vr2[1]
        self.odom.twist.twist.linear.z = vr2[2]
        self.altitude = dvl.bd_range
        
        # Update EKF
        if self.makePrediction():
            #Compte! EL DVL no es dextrogir i s'ha de negar la Y
            z = array([vr2[0], vr2[1], vr2[2]])
            self.ekf.dvlUpdate(z)
            self.publishData()
        
    
    def updateValeportSoundVelocity(self, svs):
        self.svs_last_update = rospy.Time.now()
        self.svs_init = True
        
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
        self.imu_last_update = rospy.Time.now()
        self.imu_init = True
        
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
            self.odom.header.frame_id = "girona500"
            self.odom.child_frame_id = "world"

        # ALERTA PERILL TALL DE CODI PER FER UNA XAPUSA
        # ARNAU!!!!
            if self.gps_data:
                self.odom.pose.pose.position.x = x[0]
                self.odom.pose.pose.position.y = x[1]

            #Fil Nav status topic
            self.nav.header = self.odom.header
            self.nav.position.north = x[0]
            self.nav.position.east = x[1]
            self.nav.position.depth = self.odom.pose.pose.position.z
            self.nav.body_velocity.x = x[2]
            self.nav.body_velocity.y = x[3]
            self.nav.body_velocity.z = x[4]
            self.nav.orientation.roll = angle[0]
            self.nav.orientation.pitch = angle[1]
            self.nav.orientation.yaw = angle[2]
            self.nav.orientation_rate.roll = self.odom.twist.twist.angular.x
            self.nav.orientation_rate.pitch = self.odom.twist.twist.angular.y
            self.nav.orientation_rate.yaw = self.odom.twist.twist.angular.z
            self.nav.altitude = self.altitude
            
            #Publish topics
            self.pub_nav_sts.publish(self.nav)
            self.pub_odom.publish(self.odom)
            
            #Publish TF
            br = tf.TransformBroadcaster()
            br.sendTransform((self.nav.position.north, self.nav.position.east, self.nav.position.depth),                   
                             tf.transformations.quaternion_from_euler(self.nav.orientation.roll, self.nav.orientation.pitch, self.nav.orientation.yaw),
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
            self.dvl_tf_data = array(rospy.get_param("teledyne_explorer_dvl/tf"))
        else:
            rospy.logfatal("teledyne_explorer_dvl/tf param not found")

        if rospy.has_param("tritech_igc_gyro/tf") :
            self.imu_tf_data = array(rospy.get_param("tritech_igc_gyro/tf"))
        else:
            rospy.logfatal("tritech_igc_gyro/tf param not found")

        if rospy.has_param("valeport_sound_velocity/tf") :
            self.svs_tf_data = array(rospy.get_param("valeport_sound_velocity/tf"))
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
        
        if rospy.has_param("localization/gps_init_samples"):
            self.gps_init_samples = rospy.get_param('localization/gps_init_samples')
        else:
            rospy.logfatal("localization/gps_init_samples not found")
            
        if rospy.has_param("localization/check_sensors_period"):
            self.check_sensors_period = rospy.get_param('localization/check_sensors_period')
        else:
            rospy.logfatal("localization/check_sensors_period not found")
            
        if rospy.has_param("localization/dvl_max_period_error"):
            self.dvl_max_period_error = rospy.get_param('localization/dvl_max_period_error')
        else:
            rospy.logfatal("localization/dvl_max_period_error not found")

        if rospy.has_param("localization/svs_max_period_error"):
            self.svs_max_period_error = rospy.get_param('localization/svs_max_period_error')
        else:
            rospy.logfatal("localization/csvs_max_period_error not found")
            
        if rospy.has_param("localization/imu_max_period_error"):
            self.imu_max_period_error = rospy.get_param('localization/imu_max_period_error')
        else:
            rospy.logfatal("localization/imu_max_period_error not found")
            
        if rospy.has_param("localization/max_init_time") :
            self.max_init_time = rospy.get_param("localization/max_init_time")
        else:
            rospy.logfatal("localization/max_init_time not found in param list")
            
if __name__ == '__main__':
    try:
        #   Init node
        rospy.init_node('localization')
        localization = Localization(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
