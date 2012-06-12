#!/usr/bin/env python

# ROS imports
import math

# Python imports
from numpy import *


class EKFG500 :
    def __init__(self, q_var, gps_var, dvl_bottom_var, dvl_water_var):
        self.x = zeros(5)
        self.P = zeros((5, 5))
        self.gps_h = self.gpsH()
        self.gps_r = self.gpsR(gps_var)
        self.gps_v = eye(2)
        self.dvl_h = self.dvlH()
        self.dvl_bottom_r = self.dvlR(dvl_bottom_var)
        self.dvl_water_r = self.dvlR(dvl_water_var)
        self.dvl_v = eye(3)
        self.Q = self.computeQ(q_var)
        self.init = False
        

    def f(self, x_1, u, t):
        roll = u[0]
        pitch = u[1]
        yaw = u[2]
        x1 = x_1[0]
        y1 = x_1[1]
        vx1 = x_1[2]
        vy1 = x_1[3]
        vz1 = x_1[4]
        x = zeros(5)
        
        # Compute Prediction Model with constant velocity
        x[0] = x1 + cos(pitch)*cos(yaw)*vx1*t - cos(roll)*sin(yaw)*vy1*t + sin(roll)*sin(pitch)*cos(yaw)*vy1*t + sin(roll)*sin(yaw)*vz1*t + cos(roll)*sin(pitch)*cos(yaw)*vz1*t
        x[1] = y1 + cos(pitch)*sin(yaw)*vx1*t + cos(roll)*cos(yaw)*vy1*t + sin(roll)*sin(pitch)*sin(yaw)*vy1*t - sin(roll)*cos(yaw)*vz1*t + cos(roll)*sin(pitch)*sin(yaw)*vz1*t
        x[2] = vx1
        x[3] = vy1        
        x[4] = vz1
        return x


    def computeA(self, u, t):
        roll = u[0]
        pitch = u[1]
        yaw = u[2]
        
        A = eye(5)
        A[0,2] = cos(pitch)*cos(yaw)*t
        A[0,3] = -cos(roll)*sin(yaw)*t + sin(roll)*sin(pitch)*cos(yaw)*t
        A[0,4] = sin(roll)*sin(yaw)*t + cos(roll)*sin(pitch)*cos(yaw)*t
        A[1,2] = cos(pitch)*sin(yaw)*t
        A[1,3] = cos(roll)*cos(yaw)*t + sin(roll)*sin(pitch)*sin(yaw)*t
        A[1,4] = -sin(roll)*cos(yaw)*t + cos(roll)*sin(pitch)*sin(yaw)*t
        return A
        
        
    def computeQ(self, q_var):
        Q = eye(3)
        return Q*q_var
    
    
    def computeW(self, u, t):
        roll = u[0]
        pitch = u[1]
        yaw = u[2]
        t2 = (t*t)/2
        
        W = zeros((5,3))
        W[0,0] = cos(pitch)*cos(yaw)*t2
        W[0,1] = -cos(roll)*sin(yaw)*t2 + sin(roll)*sin(pitch)*cos(yaw)*t2
        W[0,2] = sin(roll)*sin(yaw)*t2 + cos(roll)*sin(pitch)*cos(yaw)*t2
        W[1,0] = cos(pitch)*sin(yaw)*t2
        W[1,1] = cos(roll)*cos(yaw)*t2 + sin(roll)*sin(pitch)*sin(yaw)*t2
        W[1,2] = -sin(roll)*cos(yaw)*t2 + cos(roll)*sin(pitch)*sin(yaw)*t2
        W[2,0] = t
        W[3,1] = t
        W[4,2] = t
        return W
    
    
    def gpsH(self):
        gps_h = zeros((2,5))
        gps_h[0,0] = 1.0
        gps_h[1,1] = 1.0
        return gps_h
    
    
    def dvlH(self):
        dvl_h = zeros((3,5))
        dvl_h[0,2] = 1.0
        dvl_h[1,3] = 1.0
        dvl_h[2,4] = 1.0
        return dvl_h
    
    
    def gpsR(self, gps_var):
        gps_r = eye(2)
        return gps_r*gps_var
    
    
    def dvlR(self, dvl_var):
        dvl_r = eye(3)
        return dvl_r*dvl_var
        
        
    def initialize(self, x):
        self.x = x
        self.init = True
#        print "EKF Initialized"
        
        
    def prediction(self, u, t):
        A = self.computeA(u, t)
        W = self.computeW(u, t)
        self._x_ = self.f(self.x, u, t)
        self._P_ = dot(dot(A, self.P), A.T) + dot(dot(W, self.Q), W.T)
#        print "EKF Prediction"
        
 
    def gpsUpdate(self, z):
        ### gps update with z = [north, east] wrt world frame ###
        innovation = z - dot(self.gps_h, self._x_).T
        temp_K = dot(dot(self.gps_h, self._P_), self.gps_h.T) + dot(dot(self.gps_v, self.gps_r), self.gps_v.T)
        temp_K_I = squeeze(asarray(matrix(temp_K).I))
        K = dot(dot(self._P_, self.gps_h.T), temp_K_I)
        Ki = dot(K, innovation)
        self.x = self._x_ + Ki
        self.P = dot((eye(5)-dot(K, self.gps_h)), self._P_)
#        print "EKF GPS update"
        

    def dvlUpdate(self, z, velocity_respect_to = 'bottom'):
        ### dvl update with z = [vx, vy, vy] wrt vehicle frame ###
        innovation = z - dot(self.dvl_h, self._x_)
        
        if velocity_respect_to == 'bottom':
            temp_K = dot(dot(self.dvl_h, self._P_), self.dvl_h.T) + dot(dot(self.dvl_v, self.dvl_bottom_r), self.dvl_v.T)
        else:
            temp_K = dot(dot(self.dvl_h, self._P_), self.dvl_h.T) + dot(dot(self.dvl_v, self.dvl_water_r), self.dvl_v.T)
            
        temp_K_I = squeeze(asarray(matrix(temp_K).I))
        K = dot(dot(self._P_, self.dvl_h.T), temp_K_I)
        Ki = dot(K, innovation)
        self.x = self._x_ + Ki
        self.P = dot((eye(5)-dot(K, self.dvl_h)), self._P_)
#        print "EKF DVL update"
    
    
    def updatePrediction(self):
        self.x = self._x_
        self.P = self._P_
        
        
    def getStateVector(self):
        return self.x
            
if __name__ == '__main__':
    ekf_g500 = EKFG500()
    
