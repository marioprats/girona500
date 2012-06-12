#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('navigation_g500')
import rospy

# Msgs imports
from nav_msgs.msg import Odometry
from auv_msgs.msg import *

class VisualServoing :
    def __init__(self):
        rospy.init_node('visual_servoing')
        self.pub_bvr = rospy.Publisher("/control_g500/body_velocity_req", BodyVelocityReq)
        rospy.Subscriber("/g500_control", Odometry, self.updateControl)
        self.init = False
        
    def run(self):
        while not rospy.is_shutdown():
            if not self.init:
                bvr = BodyVelocityReq()
                bvr.goal.priority = GoalDescriptor.PRIORITY_LOW
                bvr.twist.linear.x = .0
                bvr.twist.linear.y = .0
                bvr.twist.linear.z = .0
                bvr.twist.angular.x = .0
                bvr.twist.angular.y = .0
                bvr.twist.angular.z = .0
                bvr.disable_axis.x = False
                bvr.disable_axis.y = False
                bvr.disable_axis.z = False
                bvr.disable_axis.roll = True
                bvr.disable_axis.pitch = True
                bvr.disable_axis.yaw = False
                self.pub_bvr.publish(bvr)
            rospy.sleep(0.1)
            
                
    def updateControl(self, req):
        self.init = True
        bvr = BodyVelocityReq()
        bvr.goal.priority = GoalDescriptor.PRIORITY_NORMAL
        bvr.twist.linear.x = req.twist.twist.linear.x * 10.0
        bvr.twist.linear.y = req.twist.twist.linear.y * 15.0
        bvr.twist.linear.z = req.twist.twist.linear.z * 10.0
        bvr.twist.angular.x = req.twist.twist.angular.x * 0.0
        bvr.twist.angular.y = req.twist.twist.angular.y * 0.0
        bvr.twist.angular.z = req.twist.twist.angular.z * 20.0
        bvr.disable_axis.x = False
        bvr.disable_axis.y = False
        bvr.disable_axis.z = False
        bvr.disable_axis.roll = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw = False
        rospy.loginfo("VisualServoing:\n%s", str(bvr))
        self.pub_bvr.publish(bvr)

if __name__ == '__main__':
    try:
        #   Init node
        visual_servoing = VisualServoing()
        visual_servoing.run()
    except rospy.ROSInterruptException: pass