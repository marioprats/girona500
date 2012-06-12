#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('safety_g500')
import rospy

from safety_g500.srv import *
from safety_g500.msg import *
import std_msgs.msg

class FakeMainBoardG500:
    def __init__(self, name):
        self.name = name
        self.battery_level = 93
          
        # Create publisher
        self.pub_battery = rospy.Publisher('/safety_g500/ocean_server_batteries', BatteryLevel)
        self.pub_internal_sensors = rospy.Publisher('/safety_g500/internal_sensors', InternalSensors)
        
         # Create Subscriber
        rospy.Subscriber('/safety_g500/heart_beat', std_msgs.msg.Empty, self.updateHeartBeat)
        
        # Create services
        self.digital_output = rospy.Service('/digital_output', DigitalOutput, self.digitalOutputSrv)
        self.mission_timeout = rospy.Service('/mission_timeout', MissionTimeout, self.missionTimeoutSrv)        
    
        # Init periodic check timer 
        rospy.Timer(rospy.Duration(1.0), self.pubInternalSensors)
        rospy.Timer(rospy.Duration(3.0), self.pubBattery)
       
        
    def updateHeartBeat(self, hb):
        # Todo if no heart beat is received in X seconds, abort mission
        pass
    

    def digitalOutputSrv(self, req):
        # Store which digital outputs are enabled and which are disabled
        ret = DigitalOutputResponse()
        ret.success = True
        return ret
    
    
    def missionTimeoutSrv(self, req):
        # Store mission timeout and when it lasts, abort the mission
        ret = MissionTimeoutResponse()
        ret.success = True
        return ret
    
    def pubInternalSensors(self, event):
        internal_sensors = InternalSensors()
        internal_sensors.header.stamp = rospy.Time.now()
        internal_sensors.id = "BAT"
        internal_sensors.humidity = 38
        internal_sensors.pressure = 1038
        internal_sensors.temperature = 31.2
        internal_sensors.water_detected = False
        self.pub_internal_sensors.publish(internal_sensors)
        
        
    def pubBattery(self, event):
        battery = BatteryLevel()
        battery.header.stamp = rospy.Time.now()
        battery.status = "DISCHARGING"
        battery.charge = self.battery_level
        battery.minutes = self.battery_level * 3.15
        battery.packs = 0
        battery.cycles = 0
        battery.full_charge = 0
        battery.full_discharge = 0
        battery.amps = -30.02
        battery.watts = 450.0
        battery.volts = 15.6
        battery.controller_1 = ['-', '-', '-', '-', '-', '-', '-', '-']
        battery.controller_2 = ['-', '-', '-', '-', '-', '-', '-', '-']
        battery.controller_3 = ['-', '-', '-', '-', '-', '-', '-', '-']
        self.pub_battery.publish(battery)
        
        # if puBattery is called every 3 seconds
        self.battery_level = self.battery_level - 0.014 
        
        
if __name__ == '__main__':
    try:
         rospy.init_node('fake_main_board_g500')
         fake_main_board_g500 = FakeMainBoardG500(rospy.get_name())
         rospy.spin()
    except rospy.ROSInterruptException: pass
