#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import time

# Commands Twist to a topic to conduct UMBmark test on the robot
class UMBmarkNode:
    def __init__(self, bot_name, wheel_speed):
        rospy.init_node("UMBmarkNode")
        rospy.loginfo("Starting UMBmarkNode as UMBmarkNode.")

        self.twist_pub = rospy.Publisher("/" + bot_name + "/cmd_vel", type, queue_size=10E6)
        self.wheel_speed = wheel_speed  # Wheel speed in cm/sec
        self.twist_msg = Twist()
        
    def run_test(self):
        # Go straight 4 meters
        self.twist_msg.angular.z = 0.
        self.twist_msg.linear.x = self.wheel_speed/10
        self.twist_pub.publish(self.twist_msg)
        

    


if __name__ == "__main__":
    name_node = UMBmarkNode("ghost")
    rospy.spin()